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

#include "driver/spi_master.h"

// ---- Local defines ----------------------------------------------------------

#define SPI_MASTER_BUFFER_SIZE  64
#define SPI_QUEUE_SIZE          2
#define SPI_DEVICE_COUNT        2

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
    // PIN_SPI_MASTER_SS2,
    // PIN_SPI_MASTER_SS3,
    // PIN_SPI_MASTER_SS4,
    // PIN_SPI_MASTER_SS5,
    // PIN_SPI_MASTER_SS6,
    // PIN_SPI_MASTER_SS7,
    PIN_SPI_MASTER_SS8,
    // PIN_SPI_MASTER_SS9,
    // PIN_SPI_MASTER_SS10,
    // PIN_SPI_MASTER_SS11,
    // PIN_SPI_MASTER_SS12
};

// ---- Local Functions --------------------------------------------------------

// Called after transaction is sent/received.
static void pre_transport_callback(spi_transaction_t* trans)
{
    // assert the correct Chipselect line
    gpio_set_level( (gpio_num_t)(int) trans->user, 0 );
}

// Called after transaction is sent/received.
static void post_transport_callback(spi_transaction_t* trans)
{
    // release the chipselect line
    gpio_set_level( (gpio_num_t)(int) trans->user, 1 );

    // notify thread that one slot has become available
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

    // Enable pull-up on SPI MISO line
    assert( gpio_set_pull_mode( PIN_SPI_MASTER_MISO, GPIO_PULLUP_ONLY ) == ESP_OK);
    
    for( int i=0; i < SPI_DEVICE_COUNT; i++ )
    {
        assert( gpio_set_level( ChipselectLines[i], 1) == ESP_OK );
        assert( gpio_set_direction( ChipselectLines[i], GPIO_MODE_OUTPUT ) == ESP_OK);
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

    while(1)
    {

        if( xSemaphoreTake( spiProcessSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            spi_transaction_t* t;
            
            esp_err_t ret = spi_device_get_trans_result( device, &t, 0 );
            
            if( ret == ESP_OK )
            {
                //ESP_LOGI("SPIS", "received %d bytes", t->trans_len / 8 );

                uint8_t* recvbuf = (uint8_t*) t->rx_buffer;

                // did we receive something?
                if( recvbuf[0] != 0 )
                {
                    // only processing CHILD messages
                    if( recvbuf[1] == 0x80 )
                    {
                        // TODO check checksum
                    }

                    // TODO parse packet and pass to macsniff

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
                    // TODO command queue erstellen und abarbeiten

                    ESP_LOGI( "SPIS", "setting up query to %d", queryDeviceNumber );

                    memset( sendbuf, 0, SPI_MASTER_BUFFER_SIZE );

                    t->user = (void*) ChipselectLines[ queryDeviceNumber ];
                    t->length = SPI_MASTER_BUFFER_SIZE * 8;

                    esp_err_t ret = spi_device_queue_trans( device, t, 0 );

                    if( ret != ESP_OK )
                    {
                        ESP_LOGW( "SPIS", "SPI device queue was not empty" );
                    }
                    else
                    {
                        ESP_LOGI( "SPIS", "enqueued query to %d, line %i", queryDeviceNumber, (gpio_num_t)(int) t->user );
                    }

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
    //     ESP_LOGW( "SPIS", "SPI Slave device queue was not empty", ret );
    // }
}
