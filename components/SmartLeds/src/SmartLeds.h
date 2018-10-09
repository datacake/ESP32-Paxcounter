#pragma once

/*
 * A C++ driver for the WS2812 LEDs using the RMT peripheral on the ESP32.
 *
 * Jan "yaqwsx" Mrázek <email@honzamrazek.cz>
 *
 * Based on the work by Martin F. Falatic - https://github.com/FozzTexx/ws2812-demo
 */

/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <memory>
#include <cassert>

#if defined ( ARDUINO )
    extern "C" { // ...someone forgot to put in the includes...
        #include "esp32-hal.h"
        #include "esp_intr.h"
        #include "driver/gpio.h"
        #include "driver/periph_ctrl.h"
        #include "freertos/semphr.h"
        #include "soc/rmt_struct.h"
        #include <driver/spi_master.h>
    }
#elif defined ( ESP_PLATFORM )
    extern "C" { // ...someone forgot to put in the includes...
        #include <esp_intr.h>
        #include <driver/gpio.h>
        #include <freertos/FreeRTOS.h>
        #include <freertos/semphr.h>
        #include <soc/dport_reg.h>
        #include <soc/gpio_sig_map.h>
        #include <soc/rmt_struct.h>
        #include <driver/spi_master.h>
    }
    #include <stdio.h>
#endif

#include "Color.h"

namespace detail {

struct TimingParams {
    uint32_t T0H;
    uint32_t T1H;
    uint32_t T0L;
    uint32_t T1L;
    uint32_t TRS;
};

union RmtPulsePair {
    struct {
        int duration0:15;
        int level0:1;
        int duration1:15;
        int level1:1;
    };
    uint32_t value;
};

static const int DIVIDER = 4; // 8 still seems to work, but timings become marginal
static const int MAX_PULSES = 32; // A channel has a 64 "pulse" buffer - we use half per pass
static const double RMT_DURATION_NS = 12.5; // minimum time of a single RMT duration based on clock ns

} // namespace detail

using LedType = detail::TimingParams;

static const LedType LED_WS2812  = { 350, 700, 800, 600, 50000 };
static const LedType LED_WS2812B = { 350, 900, 900, 350, 50000 };
static const LedType LED_SK6812  = { 300, 600, 900, 600, 80000 };
static const LedType LED_WS2813  = { 350, 800, 350, 350, 300000 };

enum BufferType { SingleBuffer = 0, DoubleBuffer };

class SmartLed {
public:
    SmartLed( const LedType& type, int count, int pin, int channel = 0, BufferType doubleBuffer = SingleBuffer )
        : _timing( type ),
          _channel( channel ),
          _count( count ),
          _firstBuffer( new Rgb[ count ] ),
          _secondBuffer( doubleBuffer ? new Rgb[ count ] : nullptr ),
          _finishedFlag( xSemaphoreCreateBinary() )
    {
        assert( channel >= 0 && channel < 8 );
        assert( ledForChannel( channel ) == nullptr );

        DPORT_SET_PERI_REG_MASK( DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN );
        DPORT_CLEAR_PERI_REG_MASK( DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST );

        PIN_FUNC_SELECT( GPIO_PIN_MUX_REG[ pin ], 2 );
        gpio_set_direction( static_cast< gpio_num_t >( pin ), GPIO_MODE_OUTPUT );
        gpio_matrix_out( static_cast< gpio_num_t >( pin ), RMT_SIG_OUT0_IDX + _channel, 0, 0 );
        initChannel( _channel );

        RMT.tx_lim_ch[ _channel ].limit = detail::MAX_PULSES;
        RMT.int_ena.val |= 1 << ( 24 + _channel );
        RMT.int_ena.val |= 1 << ( 3 * _channel );

        _bitToRmt[ 0 ].level0 = 1;
        _bitToRmt[ 0 ].level1 = 0;
        _bitToRmt[ 0 ].duration0 = _timing.T0H / ( detail::RMT_DURATION_NS * detail::DIVIDER );
        _bitToRmt[ 0 ].duration1 = _timing.T0L / ( detail::RMT_DURATION_NS * detail::DIVIDER );

        _bitToRmt[ 1 ].level0 = 1;
        _bitToRmt[ 1 ].level1 = 0;
        _bitToRmt[ 1 ].duration0 = _timing.T1H / ( detail::RMT_DURATION_NS * detail::DIVIDER );
        _bitToRmt[ 1 ].duration1 = _timing.T1L / ( detail::RMT_DURATION_NS * detail::DIVIDER );

        if ( !anyAlive() )
            esp_intr_alloc( ETS_RMT_INTR_SOURCE, 0, interruptHandler, nullptr, &_interruptHandle );

        ledForChannel( channel ) = this;
    }

    ~SmartLed() {
        ledForChannel( _channel ) = nullptr;
        if ( !anyAlive() )
            esp_intr_free( _interruptHandle );
        vSemaphoreDelete( _finishedFlag );
    }

    Rgb& operator[]( int idx ) {
        return _firstBuffer[ idx ];
    }

    const Rgb& operator[]( int idx ) const {
        return _firstBuffer[ idx ];
    }

    void show() {
        _buffer = _firstBuffer.get();
        startTransmission();
        swapBuffers();
    }

    void wait() {
        xSemaphoreTake( _finishedFlag, portMAX_DELAY );
    }

private:
    static void initChannel( int channel ) {
        RMT.apb_conf.fifo_mask = 1;  //enable memory access, instead of FIFO mode.
        RMT.apb_conf.mem_tx_wrap_en = 1; //wrap around when hitting end of buffer
        RMT.conf_ch[ channel ].conf0.div_cnt = detail::DIVIDER;
        RMT.conf_ch[ channel ].conf0.mem_size = 1;
        RMT.conf_ch[ channel ].conf0.carrier_en = 0;
        RMT.conf_ch[ channel ].conf0.carrier_out_lv = 1;
        RMT.conf_ch[ channel ].conf0.mem_pd = 0;

        RMT.conf_ch[ channel ].conf1.rx_en = 0;
        RMT.conf_ch[ channel ].conf1.mem_owner = 0;
        RMT.conf_ch[ channel ].conf1.tx_conti_mode = 0;    //loop back mode.
        RMT.conf_ch[ channel ].conf1.ref_always_on = 1;    // use apb clock: 80M
        RMT.conf_ch[ channel ].conf1.idle_out_en = 1;
        RMT.conf_ch[ channel ].conf1.idle_out_lv = 0;
    }

    static void interruptHandler( void * ) {
        for ( int channel = 0; channel != 8; channel++ ) {
            auto self = ledForChannel( channel );
            if ( RMT.int_st.val & ( 1 << ( 24 + channel ) ) ) { // tx_thr_event
                if ( self ) self->copyRmtHalfBlock();
                RMT.int_clr.val |= 1 << ( 24 + channel );
            }
            else if ( RMT.int_st.val & ( 1 << ( 3 * channel ) ) ) { // tx_end
                if ( self ) self->endTransmission();
                RMT.int_clr.val |= 1 << ( 3 * channel );
            }
        }
    }

    void swapBuffers() {
        if ( _secondBuffer )
            _firstBuffer.swap( _secondBuffer );
    }

    void copyRmtHalfBlock() {
        int offset = detail::MAX_PULSES * _halfIdx;
        _halfIdx = !_halfIdx;
        int len = 3 - _componentPosition + 3 * ( _count - 1 );
        len = std::min( len, detail::MAX_PULSES / 8 );

        if ( !len ) {
            for ( int i = 0; i < detail::MAX_PULSES; i++) {
                RMTMEM.chan[_channel].data32[i + offset].val = 0;
            }
        }

        int i;
        for ( i = 0; i != len && _pixelPosition != _count; i++ ) {
            uint8_t val = _buffer[ _pixelPosition ].getGrb( _componentPosition );
            for ( int j = 0; j != 8; j++, val <<= 1 ) {
                int bit = val >> 7;
                int idx = i * 8 + offset + j;
                RMTMEM.chan[ _channel ].data32[ idx ].val = _bitToRmt[ bit & 0x01 ].value;
            }
            if ( _pixelPosition == _count - 1 && _componentPosition == 2 ) {
                RMTMEM.chan[ _channel ].data32[ i * 8 + offset + 7 ].duration1 =
                    _timing.TRS / ( detail::RMT_DURATION_NS * detail::DIVIDER );
            }

            _componentPosition++;
            if ( _componentPosition == 3 ) {
                _componentPosition = 0;
                _pixelPosition++;
            }
        }

        for ( i *= 8; i != detail::MAX_PULSES; i++ ) {
            RMTMEM.chan[ _channel ].data32[ i + offset ].val = 0;
        }
    }

    void startTransmission() {
        _pixelPosition = _componentPosition = _halfIdx = 0;
        copyRmtHalfBlock();
        if ( _pixelPosition < _count )
            copyRmtHalfBlock();
        xSemaphoreTake( _finishedFlag, 0 );
        RMT.conf_ch[ _channel ].conf1.mem_rd_rst = 1;
        RMT.conf_ch[ _channel ].conf1.tx_start = 1;
    }

    void endTransmission() {
        xSemaphoreGiveFromISR( _finishedFlag, nullptr );
    }

    static SmartLed*& ledForChannel( int channel ) {
        static SmartLed* table[ 8 ] = { nullptr };
        assert( channel < 8 );
        return table[ channel ];
    }

    static bool anyAlive() {
        for ( int i = 0; i != 8; i++ )
            if ( ledForChannel( i ) != nullptr ) return true;
        return false;
    }

    const LedType& _timing;
    int _channel;
    detail::RmtPulsePair _bitToRmt[ 2 ];
    int _count;
    std::unique_ptr< Rgb[] > _firstBuffer;
    std::unique_ptr< Rgb[] > _secondBuffer;
    Rgb *_buffer;
    intr_handle_t _interruptHandle;

    xSemaphoreHandle _finishedFlag;

    int _pixelPosition;
    int _componentPosition;
    int _halfIdx;
};

class Apa102 {
public:
    struct ApaRgb {
        ApaRgb( uint8_t r = 0, uint8_t g = 0, uint32_t b = 0, uint32_t v = 0xFF)
            : r( r ), g( g ), b( b ), v( 0xE0 | v )
        {}

        ApaRgb& operator=( const Rgb& o ) {
            r = o.r;
            g = o.g;
            b = o.b;
            return *this;
        }

        ApaRgb& operator=(const Hsv& o ) {
            *this = Rgb{ o };
            return *this;
        }

        uint8_t r, g, b, v;
    };

    static const int FINAL_FRAME_SIZE = 4;
    static const int TRANS_COUNT = 2 + 8;

    Apa102( int count, int clkpin, int datapin, BufferType doubleBuffer = SingleBuffer )
        : _count( count ),
          _firstBuffer( new ApaRgb[ count ] ),
          _secondBuffer( doubleBuffer ? new ApaRgb[ count ] : nullptr ),
          _initFrame( 0 )
    {
        spi_bus_config_t buscfg;
        memset( &buscfg, 0, sizeof( buscfg ) );
        buscfg.mosi_io_num = datapin;
        buscfg.miso_io_num = -1;
        buscfg.sclk_io_num = clkpin;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = 65535;

        spi_device_interface_config_t devcfg;
        memset( &devcfg, 0, sizeof( devcfg ) );
        devcfg.clock_speed_hz = 1000000;
        devcfg.mode=0;
        devcfg.spics_io_num = -1;
        devcfg.queue_size = TRANS_COUNT;
        devcfg.pre_cb = nullptr;

        auto ret=spi_bus_initialize( SPI_HOST, &buscfg, 0 );
        assert(ret==ESP_OK);

        ret=spi_bus_add_device( SPI_HOST, &devcfg, &_spi );
        assert(ret==ESP_OK);

        std::fill_n( _finalFrame, FINAL_FRAME_SIZE, 0xFFFFFFFF );
    }

    ~Apa102() {
        // ToDo
    }

    ApaRgb& operator[]( int idx ) {
        return _firstBuffer[ idx ];
    }

    const ApaRgb& operator[]( int idx ) const {
        return _firstBuffer[ idx ];
    }

    void show() {
        _buffer = _firstBuffer.get();
        startTransmission();
        swapBuffers();
    }

    void wait() {
        for ( int i = 0; i != _transCount; i++ ) {
            spi_transaction_t *t;
            spi_device_get_trans_result( _spi, &t, portMAX_DELAY );
        }
    }
private:
    void swapBuffers() {
        if ( _secondBuffer )
            _firstBuffer.swap( _secondBuffer );
    }

    void startTransmission() {
        for ( int i = 0; i != TRANS_COUNT; i++ ) {
            _transactions[ i ].cmd = 0;
            _transactions[ i ].addr = 0;
            _transactions[ i ].flags = 0;
            _transactions[ i ].rxlength = 0;
            _transactions[ i ].rx_buffer = nullptr;
        }
        // Init frame
        _transactions[ 0 ].length = 32;
        _transactions[ 0 ].tx_buffer = &_initFrame;
        spi_device_queue_trans( _spi, _transactions + 0, portMAX_DELAY );
        // Data
        _transactions[ 1 ].length = 32 * _count;
        _transactions[ 1 ].tx_buffer = _buffer;
        spi_device_queue_trans( _spi, _transactions + 1, portMAX_DELAY );
        _transCount = 2;
        // End frame
        for ( int i = 0; i != 1 + _count / 32 / FINAL_FRAME_SIZE; i++ ) {
            _transactions[ 2 + i ].length = 32 * FINAL_FRAME_SIZE;
            _transactions[ 2 + i ].tx_buffer = _finalFrame;
            spi_device_queue_trans( _spi, _transactions + 2 + i, portMAX_DELAY );
            _transCount++;
        }
    }

    spi_device_handle_t _spi;
    int _count;
    std::unique_ptr< ApaRgb[] > _firstBuffer, _secondBuffer;
    ApaRgb *_buffer;

    spi_transaction_t _transactions[ TRANS_COUNT ];
    int _transCount;

    uint32_t _initFrame;
    uint32_t _finalFrame[ FINAL_FRAME_SIZE ];
};
