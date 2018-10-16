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

#include "spi_master.h"
#include "rcommand.h"

#include "globals.h"

#include "PacketEvent.h"
#include "driver/spi_master.h"

// ---- Local defines ----------------------------------------------------------

#define SPI_MASTER_BUFFER_SIZE  64
#define SPI_QUEUE_SIZE          1
#define SPI_DEVICE_COUNT        12

static uint8_t txBuffer[SPI_QUEUE_SIZE][SPI_MASTER_BUFFER_SIZE];
static uint8_t rxBuffer[SPI_QUEUE_SIZE][SPI_MASTER_BUFFER_SIZE];
static uint8_t cmd_ack;
static int queryDeviceNumber;
static spi_transaction_t transactions[SPI_QUEUE_SIZE];
static spi_device_handle_t device;
static SemaphoreHandle_t spiProcessSemaphore;
static gpio_num_t ChipselectLines[SPI_DEVICE_COUNT] =
{
    PIN_SPI_MASTER_SS1,
    PIN_SPI_MASTER_SS2,
    PIN_SPI_MASTER_SS3,
    PIN_SPI_MASTER_SS4,
    PIN_SPI_MASTER_SS5,
    PIN_SPI_MASTER_SS6,
    PIN_SPI_MASTER_SS7,
    PIN_SPI_MASTER_SS8,
    PIN_SPI_MASTER_SS9,
    PIN_SPI_MASTER_SS10,
    PIN_SPI_MASTER_SS11,
    PIN_SPI_MASTER_SS12
};

// ---- Local Functions --------------------------------------------------------

// Called after transaction is sent/received.
static void pre_transport_callback(spi_transaction_t* trans)
{
    // assert the correct Chipselect line
    //gpio_set_level( (gpio_num_t)(int) trans->user, 0 );
}

// Called after transaction is sent/received.
static void post_transport_callback(spi_transaction_t* trans)
{
    // release the chipselect line
    gpio_set_level( (gpio_num_t)(int) trans->user, 1 );

    // notify thread that one slot has become available
    // (if realtime response is necessary, use xSemaphoreGiveFromISR() instead)
    xSemaphoreGive( spiProcessSemaphore );
}

// ---- Public Functions -------------------------------------------------------

/**
 * SPI master
 * 
 */
void spi_master_init(void)
{
    // Configuration for the SPI bus
    spi_bus_config_t buscfg =
    {
        .mosi_io_num = PIN_SPI_MASTER_MOSI,
        .miso_io_num = PIN_SPI_MASTER_MISO,
        .sclk_io_num = PIN_SPI_MASTER_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MASTER_BUFFER_SIZE,
        .flags = 0
    };

    // Increase drive strength
    assert( gpio_set_drive_capability( PIN_SPI_MASTER_MOSI, GPIO_DRIVE_CAP_3 ) == ESP_OK );
    assert( gpio_set_drive_capability( PIN_SPI_MASTER_SCK,  GPIO_DRIVE_CAP_3 ) == ESP_OK );

    // Configuration for the SPI Master interface
    spi_device_interface_config_t mstrcfg = 
    {
        .command_bits =     0,
        .address_bits =     0,
        .dummy_bits =       0,
        .mode =             0,
        .duty_cycle_pos =   128,
        .cs_ena_pretrans =  0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz =   1*1000*1000,
        .input_delay_ns =   0,
        .spics_io_num =     -1,
        .flags =            0,
        .queue_size =       SPI_QUEUE_SIZE,
        .pre_cb =           pre_transport_callback,
        .post_cb =          post_transport_callback
    };

    queryDeviceNumber = 0;
    memset( transactions, 0, sizeof(transactions) );

    // Setup all Queues
    for( int qNo = 0; qNo < SPI_QUEUE_SIZE; qNo++ )
    {
        transactions[qNo].tx_buffer = &txBuffer[qNo][0];
        transactions[qNo].rx_buffer = &rxBuffer[qNo][0];
    }

    gpio_config_t io_conf = 
    {
        .pin_bit_mask = 0,
        .mode =         GPIO_MODE_OUTPUT,
        .pull_up_en =   GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type =    GPIO_INTR_DISABLE
    };

    for( int i=0; i < SPI_DEVICE_COUNT; i++ )
    {
        io_conf.pin_bit_mask = ( 1ULL << ChipselectLines[i] );
        assert( gpio_config( &io_conf ) == ESP_OK );
        gpio_set_level( ChipselectLines[i], 1 );
    }

    // Initialize SPI Master interface
    assert( spi_bus_initialize( HSPI_HOST, &buscfg, 2 ) == ESP_OK);

    // Init device (we will control CS manually)
    assert( spi_bus_add_device( HSPI_HOST, &mstrcfg, &device ) == ESP_OK );

    // Create a semaphore for IRQ / Task sync
    spiProcessSemaphore = xSemaphoreCreateCounting( SPI_QUEUE_SIZE, SPI_QUEUE_SIZE );
}

void spi_master_task(void *pvParameters)
{
    configASSERT(((uint32_t)pvParameters) == 1); // FreeRTOS check

    // Delay setup phase until all Children have had enough time to boot
    vTaskDelay( 5000 * portTICK_PERIOD_MS );

    // ---- Clear phase --------------------------------------------------------
    ESP_LOGI( "SPIM", "---- Starting Clear Phase ----" );
    {
        uint8_t setupDevice = 0;
        while( setupDevice <  SPI_DEVICE_COUNT)
        {
            ESP_LOGI( "SPIM", "Clear device %d", setupDevice );
            uint8_t step = 0;
            while( step < 1 )
            {
                spi_transaction_t* t;
                // retrieve spent transactions
                if( spi_device_get_trans_result( device, &t, 0 ) == ESP_OK )
                {
                    ESP_LOGD( "SPIM", "Cleared device %d", setupDevice );

                    // step after each packet has been sent
                    step++;
                    // free transfer slot
                    t->length = 0;
                }

                // only use first transaction during setup phase
                t = &transactions[0];
                
                if( t->length == 0 && step < 1 )
                {
                    // delay after last transaction
                    vTaskDelay( 10 * portTICK_PERIOD_MS );

                    // Get correct chipselect
                    t->user = (void*) ChipselectLines[ setupDevice ];

                    // assert Chipselect
                    gpio_set_level( (gpio_num_t)(int) t->user, 0 );

                    // delay for Chipselect setup time
                    vTaskDelay( 1 * portTICK_PERIOD_MS );

                    // send nothing (dummy)
                    uint8_t* sendbuf = (uint8_t*) t->tx_buffer;
                    memset( sendbuf, 0, SPI_MASTER_BUFFER_SIZE );

                    t->length = SPI_MASTER_BUFFER_SIZE * 8;

                    esp_err_t ret = spi_device_queue_trans( device, t, 0 );

                    if( ret != ESP_OK )
                    {
                        ESP_LOGW( "SPIM", "Clear: SPI device queue was not empty" );
                    }
                }
            }
            setupDevice++;
        }
    }

    vTaskDelay( 50 * portTICK_PERIOD_MS );

    // ---- Setup phase --------------------------------------------------------
    ESP_LOGI( "SPIM", "---- Starting Child device Setup ----" );
    #define SETUP_STEP_COUNT 5
    {
        uint8_t setupDevice = 0;
        while( setupDevice <  SPI_DEVICE_COUNT)
        {
            ESP_LOGI( "SPIM", "Setup device %d", setupDevice );
            uint8_t step = 0;
            while( step < SETUP_STEP_COUNT )
            {
                spi_transaction_t* t;
                // retrieve spent transactions
                if( spi_device_get_trans_result( device, &t, 0 ) == ESP_OK )
                {
                    uint8_t* recvbuf = (uint8_t*) t->rx_buffer;

                    if( step == 1 )
                    {
                        if( recvbuf[6] >> 6 != ROLE_CHILD )
                        {
                            ESP_LOGE( "SPIM", "Invalid ROLE response from device %d", setupDevice );
                        }
                        uint16_t build = (recvbuf[6] & 0x3F) << 8 | recvbuf[7];
                        ESP_LOGI( "SPIM", "Device BUILD_NUMBER: %d", build );
                    }
                    
                    // step after each packet has been sent
                    step++;
                    // free transfer slot
                    t->length = 0;
                }

                // only use first transaction during setup phase
                t = &transactions[0];
                
                if( t->length == 0 && step < SETUP_STEP_COUNT )
                {
                    // delay after last transaction
                    vTaskDelay( 100 * portTICK_PERIOD_MS );

                    // Get correct chipselect
                    t->user = (void*) ChipselectLines[ setupDevice ];

                    // assert Chipselect
                    gpio_set_level( (gpio_num_t)(int) t->user, 0 );

                    // delay for Chipselect setup time
                    vTaskDelay( 1 * portTICK_PERIOD_MS );

                    uint8_t* sendbuf = (uint8_t*) t->tx_buffer;
                    memset( sendbuf, 0, SPI_MASTER_BUFFER_SIZE );

                    switch( step )
                    {
                        case 0:
                            // dummy write to make sure everyone is
                            // on the same page
                            break;
                        
                        case 1:
                            // turn off BLE SCAN
                            sendbuf[0] = 4;
                            sendbuf[1] = RCMDPORT;
                            sendbuf[2] = 0x0e;
                            sendbuf[3] = false;
                            break;

                        case 2:
                            // configure channel
                            sendbuf[0] = 5;
                            sendbuf[1] = RCMDPORT;
                            sendbuf[2] = 0x40;
                            sendbuf[3] = setupDevice + 1;
                            sendbuf[4] = 1;
                            break;

                        case 3:
                            // forward Parent vendorfilter setting
                            sendbuf[0] = 4;
                            sendbuf[1] = RCMDPORT;
                            sendbuf[2] = 0x0d;
                            sendbuf[3] = cfg.vendorfilter;
                            break;

                        case 4:
                            // forward Parent Antenna setting
                            sendbuf[0] = 4;
                            sendbuf[1] = RCMDPORT;
                            sendbuf[2] = 0x0f;
                            sendbuf[3] = cfg.wifiant;
                            break;
                    }

                    t->length = SPI_MASTER_BUFFER_SIZE * 8;

                    esp_err_t ret = spi_device_queue_trans( device, t, 0 );

                    if( ret != ESP_OK )
                    {
                        ESP_LOGW( "SPIM", "Setup: SPI device queue was not empty" );
                    }
                    else
                    {
                        ESP_LOGD( "SPIM", "Setup sent step %d device %d", step, setupDevice );
                    }
                }
            }
            setupDevice++;
        }
    }

    vTaskDelay( 50 * portTICK_PERIOD_MS );

    // ---- Run phase ----------------------------------------------------------

    ESP_LOGI( "SPIM", "--- Starting SPI Master run loop ---" );

    while(1)
    {
        if( xSemaphoreTake( spiProcessSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            spi_transaction_t* t;
            
            esp_err_t ret = spi_device_get_trans_result( device, &t, 0 );
            
            if( ret == ESP_OK )
            {
                //ESP_LOGI("SPIM", "received %d bytes", t->trans_len / 8 );

                uint8_t* recvbuf = (uint8_t*) t->rx_buffer;

                // did we receive something?
                if( recvbuf[0] != 0 )
                {
                    // only processing CHILD messages
                    if( recvbuf[1] == 0x80 )
                    {
                        // check checksum
                        uint8_t msg_len = recvbuf[0];

                        uint8_t sum = 0;
                        for(int i=0; i < msg_len; i++ )
                        {
                            sum += recvbuf[i];
                        }

                        if( sum != 0 )
                        {
                            ESP_LOGW( "SPIM", "checksum error");
                        }
                        else
                        {
                            uint8_t pktCnt = ( msg_len - (SPI_HDR_LEN + 1) ) / PACKET_EVENT_PACKED_SIZE;
                            for( int pktNo=0; pktNo < pktCnt; pktNo++ )
                            {
                                PacketEvent pkt;
                                uint8_t offset = SPI_HDR_LEN + pktNo * PACKET_EVENT_PACKED_SIZE;
                                pkt.fromBuffer( &recvbuf[ offset ] );
                                mac_add( pkt.mac, pkt.rssi, pkt.channel );
                            }
                        }
                    }

                    // clear
                    recvbuf[0] = 0;
                }

                // free transfer slot
                t->length = 0;
            }

            for(int txNo = 0; txNo < SPI_QUEUE_SIZE; txNo++ )
            {
                t = &transactions[txNo];
                uint8_t* sendbuf = (uint8_t*) t->tx_buffer;

                if( t->length == 0 )
                {
                    // delay after last transaction
                    vTaskDelay( 1 * portTICK_PERIOD_MS );

                    // Get correct chipselect
                    t->user = (void*) ChipselectLines[ queryDeviceNumber ];

                    // assert Chipselect
                    gpio_set_level( (gpio_num_t)(int) t->user, 0 );

                    // delay for Chipselect setup time
                    vTaskDelay( 1 * portTICK_PERIOD_MS );

                    // clear
                    memset( sendbuf, 0, SPI_MASTER_BUFFER_SIZE );

                    t->length = SPI_MASTER_BUFFER_SIZE * 8;

                    esp_err_t ret = spi_device_queue_trans( device, t, 0 );

                    if( ret != ESP_OK )
                    {
                        ESP_LOGW( "SPIM", "SPI device queue was not empty" );
                    }
                    // else
                    // {
                    //     ESP_LOGI( "SPIM", "enqueued query to %d, line %i", queryDeviceNumber, (gpio_num_t)(int) t->user );
                    // }

                    // round-robin
                    queryDeviceNumber = ((queryDeviceNumber + 1) % SPI_DEVICE_COUNT);
                }
            }
        }
    }
}

void spi_master_send( MessageBuffer_t* message )
{
    // TODO needs to be adapted to multi-buffer-queue
    // // clear receive-buffer
    // memset( recvbuf, 0, SPI_SLAVE_BUFFER_SIZE );

    // // allocate transaction object
    // spi_slave_transaction_t* t = &transactions[0];
    
    // t->length = SPI_SLAVE_BUFFER_SIZE * 8;

    // // copy data to send-buffer
    // int totalMessageLength = message->MessageSize + 2;
    // sendbuf[ 0 ] = totalMessageLength;
    // sendbuf[ 1 ] = message->MessagePort;
    // memcpy( &sendbuf[ 2 ], message->Message, message->MessageSize );
    
    // esp_err_t ret = spi_slave_queue_trans( VSPI_HOST, t, 0 );

    // if( ret != ESP_OK )
    // {
    //     ESP_LOGW( "SPIM", "SPI Slave device queue was not empty", ret );
    // }
}
