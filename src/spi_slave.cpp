/**
 * Copyright 2018 Datacake GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "spi_slave.h"
#include "rcommand.h"

#include "driver/spi_slave.h"

// ---- Local defines ----------------------------------------------------------

#define SPI_SLAVE_BUFFER_SIZE   64

static uint8_t sendbuf[SPI_SLAVE_BUFFER_SIZE];
static uint8_t recvbuf[SPI_SLAVE_BUFFER_SIZE];

// ---- Local Functions --------------------------------------------------------

// Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
static void post_setup_callback(spi_slave_transaction_t *trans) 
{
    gpio_set_level( PIN_SPI_SLAVE_IRQ, 1 );
}

// Called after transaction is sent/received. We use this to set the handshake line low.
static void post_transport_callback(spi_slave_transaction_t *trans)
{
    gpio_set_level( PIN_SPI_SLAVE_IRQ, 0 );
}

// ---- Public Functions -------------------------------------------------------

/**
 * SPI slave with "Handshake" signal
 * 
 */
void spi_slave_init(void)
{
    // Configuration for the SPI bus
    spi_bus_config_t buscfg =
    {
        .mosi_io_num = PIN_SPI_SLAVE_MOSI,
        .miso_io_num = PIN_SPI_SLAVE_MISO,
        .sclk_io_num = PIN_SPI_SLAVE_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
        .flags = SPICOMMON_BUSFLAG_SLAVE || SPICOMMON_BUSFLAG_NATIVE_PINS || SPICOMMON_BUSFLAG_SCLK || SPICOMMON_BUSFLAG_MISO || SPICOMMON_BUSFLAG_MOSI
    };

    // Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = 
    {
        .spics_io_num =     PIN_SPI_SLAVE_SS,
        .flags =            0,
        .queue_size =       1,
        .mode =             0,
        .post_setup_cb =    post_setup_callback,
        .post_trans_cb =    post_transport_callback
    };

    // Configuration for the handshake line
    gpio_config_t io_conf = 
    {
        .pin_bit_mask = BIT( PIN_SPI_SLAVE_IRQ ),
        .mode =         GPIO_MODE_OUTPUT,
        .pull_up_en =   GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type =    GPIO_INTR_DISABLE
    };

    // Configure handshake line as output
    assert( gpio_config( &io_conf ) == ESP_OK);

    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    assert( gpio_set_pull_mode( PIN_SPI_SLAVE_MOSI, GPIO_PULLUP_ONLY ) == ESP_OK);
    assert( gpio_set_pull_mode( PIN_SPI_SLAVE_SCK, GPIO_PULLUP_ONLY ) == ESP_OK);
    assert( gpio_set_pull_mode( PIN_SPI_SLAVE_SS, GPIO_PULLUP_ONLY ) == ESP_OK);

    // Initialize SPI slave interface
    assert( spi_slave_initialize( VSPI_HOST, &buscfg, &slvcfg, 1 ) == ESP_OK);
}

void spi_slave_process( void )
{
    // did we receive something?
    if( recvbuf[0] != 0 )
    {
        // only processing remote commands
        if( recvbuf[1] == RCMDPORT )
        {
            rcommand( &recvbuf[2], recvbuf[0] - 2 );
        }

        // clear
        recvbuf[0] = 0;
    }
}

void spi_slave_send( MessageBuffer_t* message )
{
    // clear receive-buffer
    memset( recvbuf, 0, SPI_SLAVE_BUFFER_SIZE );

    // copy data to send-buffer
    int totalMessageLength = message->MessageSize + 2;
    sendbuf[ 0 ] = totalMessageLength;
    sendbuf[ 1 ] = message->MessagePort;
    memcpy( &sendbuf[ 2 ], message->Message, message->MessageSize );
    
    // setup transaction object
    spi_slave_transaction_t t;
    memset( &t, 0, sizeof(t) );
    t.length = totalMessageLength * 8;
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    // enqueue this transaction
    // if there is no room in the queue then nothing will be enqueued and
    // the data will be discarded!
    esp_err_t ret = spi_slave_transmit( VSPI_HOST, &t, 0 );

    if( ret != ESP_OK )
    {
        ESP_LOGW( "SPIS", "SPI Slave device queue was not empty", ret );
    }
}
