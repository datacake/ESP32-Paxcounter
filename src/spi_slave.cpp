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
#define SPI_QUEUE_SIZE          1

static uint8_t txBuffer[SPI_QUEUE_SIZE][SPI_SLAVE_BUFFER_SIZE];
static uint8_t rxBuffer[SPI_QUEUE_SIZE][SPI_SLAVE_BUFFER_SIZE];
static uint8_t cmd_ack;
static uint8_t msg_id;
static spi_slave_transaction_t transactions[SPI_QUEUE_SIZE];
static SemaphoreHandle_t spiProcessSemaphore, spiInitSemaphore, spiDeInitSemaphore;

static void post_setup_callback(spi_slave_transaction_t *trans);
static void post_transport_callback(spi_slave_transaction_t *trans);

// Configuration for the SPI bus
static spi_bus_config_t buscfg =
{
    .mosi_io_num = PIN_SPI_SLAVE_MOSI,
    .miso_io_num = PIN_SPI_SLAVE_MISO,
    .sclk_io_num = PIN_SPI_SLAVE_SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = SPI_SLAVE_BUFFER_SIZE,
    .flags = 0
};

// Configuration for the SPI slave interface
static spi_slave_interface_config_t slvcfg = 
{
    .spics_io_num =     PIN_SPI_SLAVE_SS,
    .flags =            0,
    .queue_size =       SPI_QUEUE_SIZE,
    .mode =             0,
    .post_setup_cb =    post_setup_callback,
    .post_trans_cb =    post_transport_callback
};

// ---- Local Functions --------------------------------------------------------

// Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
static void post_setup_callback(spi_slave_transaction_t *trans)
{
    
}

// Called after transaction is sent/received. We use this to set the handshake line low.
static void post_transport_callback(spi_slave_transaction_t *trans)
{
    // make sure pins are passive first thing after transmission
    gpio_set_direction( PIN_SPI_SLAVE_MOSI, GPIO_MODE_INPUT );
    gpio_set_direction( PIN_SPI_SLAVE_SCK, GPIO_MODE_INPUT );
    gpio_set_direction( PIN_SPI_SLAVE_MISO, GPIO_MODE_INPUT );

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( spiDeInitSemaphore, &xHigherPriorityTaskWoken );

    // notify thread that one slot has become available
    xSemaphoreGiveFromISR( spiProcessSemaphore, &xHigherPriorityTaskWoken );

    portYIELD_FROM_ISR();
}

// called when SS goes low
static void IRAM_ATTR ss_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( spiInitSemaphore, &xHigherPriorityTaskWoken );

    portYIELD_FROM_ISR();
}

// ---- Public Functions -------------------------------------------------------

/**
 * SPI slave with "Handshake" signal
 * 
 */
void spi_slave_init(void)
{
    // // --- register an interrupt handler for SS line ---------------------------
    gpio_install_isr_service( 0 );

    gpio_config_t io_conf = 
    {
        .pin_bit_mask = ( 1ULL << PIN_SPI_SLAVE_SS ),
        .mode =         GPIO_MODE_INPUT,
        .pull_up_en =   GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type =    GPIO_INTR_NEGEDGE
    };

    // Configure SS line as interrupt input
    assert( gpio_config( &io_conf ) == ESP_OK);
    gpio_isr_handler_add( PIN_SPI_SLAVE_SS, ss_isr_handler, NULL );

    msg_id = 0;
    cmd_ack = 0;

    memset( transactions, 0, sizeof(transactions) );

    // Setup all Queues
    for( int qNo = 0; qNo < SPI_QUEUE_SIZE; qNo++ )
    {
        transactions[qNo].tx_buffer = &txBuffer[qNo][0];
        transactions[qNo].rx_buffer = &rxBuffer[qNo][0];
    }

    // // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    // assert( gpio_set_pull_mode( PIN_SPI_SLAVE_MOSI, GPIO_PULLUP_ONLY ) == ESP_OK);
    // assert( gpio_set_pull_mode( PIN_SPI_SLAVE_SCK, GPIO_PULLUP_ONLY ) == ESP_OK);
    assert( gpio_set_pull_mode( PIN_SPI_SLAVE_SS, GPIO_PULLUP_ONLY ) == ESP_OK);

    // Create a semaphore for IRQ / Task sync
    spiProcessSemaphore = xSemaphoreCreateCounting( SPI_QUEUE_SIZE, SPI_QUEUE_SIZE );
    spiInitSemaphore = xSemaphoreCreateCounting( 1, 0 );
    spiDeInitSemaphore = xSemaphoreCreateCounting( 1, 0 );
}

void spi_slave_task(void *pvParameters)
{
    int spi_state = 0;
    configASSERT(((uint32_t)pvParameters) == 1); // FreeRTOS check

    ESP_LOGI( "SPIS", "PaxRover Build: %u, Role: %s",
        (int) BUILD_NUMBER,
        DEVICE_ROLE == ROLE_CHILD ? "CHILD" : "PARENT" );

    ESP_LOGI( "SPIS", "---- Starting SPI Slave run loop ----");

    while(1)
    {
        switch( spi_state )
        {

        case 0:
            // wait forever on process semaphore
            xSemaphoreTake( spiProcessSemaphore, portMAX_DELAY );
            
            // clear the other two in case they are set (no blocking)
            xSemaphoreTake( spiInitSemaphore, 0 );
            xSemaphoreTake( spiDeInitSemaphore, 0 );

            for(int txNo = 0; txNo < SPI_QUEUE_SIZE; txNo++ )
            {
                spi_slave_transaction_t* t = &transactions[txNo];
                uint8_t* sendbuf = (uint8_t*) t->tx_buffer;
                uint8_t* recvbuf = (uint8_t*) t->rx_buffer;

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

                // free transfer slot
                t->length = 0;

                if( t->length == 0 )
                {
                #if DEVICE_ROLE == ROLE_CHILD
                    // if we have packets in the queue then
                    // start processing them in the buffer

                    // SPI data Frame for CHILD comms
                    // byte |    0    |    1    |    2    |    3    |    4    |    5    |  6 .. (5 + N) | 6 + N |
                    //      | msg_len |  0xf0   | cmd_ack | msg_id  |   pkt_queue_len   | <PacketEvent> |  sum  |
                    //      |<--------------------------- msg_len ------------------------->|
                    //      |<------------------------- sum ----------------------->|

                    // PacketEvent encoding
                    // byte |    0    |    1    |    2    |    3    | ..
                    //      |                tv_sec
                    //
                    //   .. |    4    |    5    |    6    |    7    | ..
                    //   .. |                tv_nsec                | ..
                    //
                    //   .. |    8    |    9    |   10    |   11    | ..
                    //   .. |  mac[0] |  mac[1] |  mac[2] |  mac[3] | ..
                    //
                    //   .. |    12   |    13   |   14    |   15    |
                    //   .. |  mac[4] |  mac[5] |  rssi   | channel |

                    uint16_t pkt_queue_len;
                    portENTER_CRITICAL(&packetListMutex);
                    size_t qLen = packets.size();
                    portEXIT_CRITICAL(&packetListMutex);
                    uint8_t msg_len = 0;

                    if( qLen > 0xffff )
                    {
                        pkt_queue_len = 0xffff;
                    }
                    else
                    {
                        pkt_queue_len = qLen;
                    }

                    // Add up to 3 PacketEvents into this SPI transfer
                    int pktNo;
                    for( pktNo=0; pktNo < pkt_queue_len && pktNo < 3; pktNo++ )
                    {
                        portENTER_CRITICAL(&packetListMutex);
                        PacketEvent* pkt = packets.front();     // get the oldest packet
                        packets.pop_front();                    // remove from list
                        portEXIT_CRITICAL(&packetListMutex);

                        size_t offset = pktNo * 16;
                        msg_len += pkt->toBuffer(
                            &sendbuf[SPI_HDR_LEN + offset],
                            SPI_SLAVE_BUFFER_SIZE - SPI_HDR_LEN - offset);

                        // really important!!! we need to free the memory
                        free(pkt);
                    }

                    // subtract the packets we just packed
                    pkt_queue_len -= pktNo;

                    // add header
                    msg_len += SPI_HDR_LEN + 1; // HDR_LEN + CHECKSUM (1 byte)
                    sendbuf[0] = msg_len;
                    sendbuf[1] = 0x80; // port for CHILD messages
                    sendbuf[2] = cmd_ack;
                    sendbuf[3] = msg_id++;
                    sendbuf[4] = pkt_queue_len >> 8;
                    sendbuf[5] = pkt_queue_len;
                    sendbuf[6] = (BUILD_NUMBER >> 8) | (DEVICE_ROLE << 6);
                    sendbuf[7] = BUILD_NUMBER;

                    // calc checksum
                    uint8_t sum = 0;
                    for(int i=0; i < (msg_len - 1); i++ )
                    {
                        sum += sendbuf[i];
                    }
                    sendbuf[msg_len - 1] = 0x00 - sum;

                    t->length = SPI_SLAVE_BUFFER_SIZE * 8;
                    t->trans_len = 0;

                #elif DEVICE_ROLE == ROLE_PARENT
                    t->length = SPI_SLAVE_BUFFER_SIZE * 8;
                    t->trans_len = 0;
                #endif
                }
            }
            spi_state = 1;
            break;

        case 1:
            // wait forever
            xSemaphoreTake( spiInitSemaphore, portMAX_DELAY );

            // Initialize SPI slave interface
            assert( spi_slave_initialize( VSPI_HOST, &buscfg, &slvcfg, 1 ) == ESP_OK);

            // enqueue free buffers
            for(int txNo = 0; txNo < SPI_QUEUE_SIZE; txNo++ )
            {
                spi_slave_transaction_t* t = &transactions[txNo];

                if( t->length != 0 )
                {
                    spi_slave_queue_trans( VSPI_HOST, t, 0 );
                }
            }
            spi_state = 2;
            break;

        case 2:
            // wait forever
            xSemaphoreTake( spiDeInitSemaphore, portMAX_DELAY );

            // retrieve result
            spi_slave_transaction_t* t;
            spi_slave_get_trans_result( VSPI_HOST, &t, 0 );

            // de-init SPI
            spi_slave_free( VSPI_HOST );

            spi_state = 0;
            break;
        }
    }
}

void spi_slave_send( MessageBuffer_t* message )
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