// Basic Config
#include "globals.h"
#include "macsniff.h"

static const char TAG[] = "SND";

// put data to send in RTos Queues used for transmit over channels Lora and SPI
void SendData(uint8_t port) {

  MessageBuffer_t SendBuffer;

  SendBuffer.MessageSize = payload.getSize();
  SendBuffer.MessagePort = PAYLOAD_ENCODER <= 2
                               ? port
                               : (PAYLOAD_ENCODER == 4 ? LPP2PORT : LPP1PORT);
  memcpy(SendBuffer.Message, payload.getBuffer(), payload.getSize());

  // enqueue message in LoRa send queue
#ifdef HAS_LORA
  if (xQueueSendToBack(LoraSendQueue, (void *)&SendBuffer, (TickType_t)0) ==
      pdTRUE)
    ESP_LOGI(TAG, "%d bytes enqueued to send on LoRa", payload.getSize());
#endif

// enqueue message in SPI send queue
#ifdef HAS_SPI
  if (xQueueSendToBack(SPISendQueue, (void *)&SendBuffer, (TickType_t)0) ==
      pdTRUE)
    ESP_LOGI(TAG, "%d bytes enqueued to send on SPI", payload.getSize());
#endif

  // clear counter if not in cumulative counter mode
  if ((port == COUNTERPORT) && (cfg.countermode != 1)) {
    reset_counters(); // clear macs container and reset all counters
    reset_salt();     // get new salt for salting hashes
    ESP_LOGI(TAG, "Counter cleared");
  }
} // SendData

// cyclic called function to prepare payload to send
void sendPayload() {

  if (SendCycleTimerIRQ) {
    portENTER_CRITICAL(&timerMux);
    SendCycleTimerIRQ = 0;
    portEXIT_CRITICAL(&timerMux);

    // append counter data to payload
    payload.reset();
    #if DEVICE_ROLE == ROLE_STANDALONE
    payload.addCount(macs_wifi, cfg.blescan ? macs_ble : 0);
    #endif
    // append GPS data, if present

#ifdef HAS_GPS
    // show NMEA data in debug mode, useful for debugging GPS on board
    // connection
    ESP_LOGD(TAG, "GPS NMEA data: passed %d / failed: %d / with fix: %d",
             gps.passedChecksum(), gps.failedChecksum(),
             gps.sentencesWithFix());
    // log GPS position if we have a fix and gps data mode is enabled
    if ((cfg.gpsmode) && (gps.location.isValid())) {
      gps_read();
      payload.addGPS(gps_status);
      ESP_LOGD(TAG, "lat=%.6f | lon=%.6f | %u Sats | HDOP=%.1f | Altitude=%um",
               gps_status.latitude / (float)1e6,
               gps_status.longitude / (float)1e6, gps_status.satellites,
               gps_status.hdop / (float)100, gps_status.altitude);
    } else {
      ESP_LOGD(TAG, "No valid GPS position or GPS data mode disabled");
    }
#endif
    SendData(COUNTERPORT);
  }
} // sendpayload()

// interrupt handler used for payload send cycle timer
void IRAM_ATTR SendCycleIRQ() {
  portENTER_CRITICAL(&timerMux);
  SendCycleTimerIRQ++;
  portEXIT_CRITICAL(&timerMux);
}

// cyclic called function to eat data from RTos send queues and transmit it
void processSendBuffer() {

  MessageBuffer_t SendBuffer;

#ifdef HAS_LORA
  // Check if there is a pending TX/RX job running
  if ((LMIC.opmode & (OP_JOINING | OP_REJOIN | OP_TXDATA | OP_POLL)) != 0) {
    // LoRa Busy -> don't eat data from queue, since it cannot be sent
  } else {
    if (xQueueReceive(LoraSendQueue, &SendBuffer, (TickType_t)0) == pdTRUE) {
      // SendBuffer gets struct MessageBuffer with next payload from queue
      LMIC_setTxData2(SendBuffer.MessagePort, SendBuffer.Message,
                      SendBuffer.MessageSize, (cfg.countermode & 0x02));
      ESP_LOGI(TAG, "%d bytes sent to LoRa", SendBuffer.MessageSize);
      sprintf(display_line7, "PACKET QUEUED");
    }
  }
#endif

#if defined(HAS_SPI) && (DEVICE_ROLE == ROLE_STANDALONE)
  if (xQueueReceive(SPISendQueue, &SendBuffer, (TickType_t)0) == pdTRUE) {
#  if HAS_SPI_SLAVE
    spi_slave_send( &SendBuffer );
#  endif
    ESP_LOGI(TAG, "%d bytes sent to SPI", SendBuffer.MessageSize);

    int uptime_ms = esp_timer_get_time() / 1000;
    char tbuf[14];
    printf("------------------------------------------------------------------\n");
    printf("Total WiFi: %hu, BL: %hu, uptime: %.02hu:%.02hu:%.02hu, ram: %u bytes\n",
      macs_wifi, macs_ble,
      (uptime_ms / 3600000),
      (uptime_ms / 60000) % 60,
      (uptime_ms / 1000) % 60,
      ESP.getFreeHeap());
    printf("------------------------------------------------------------------\n");
    printf("MAC          | CH | RSSI | LAST [sec] | COUNT  \n");

    for( auto iter = macs.begin(); iter != macs.end(); iter++ )
    {
      FoundDevice* dev = &*iter;

      printf( "%012llX | %.2i | %.2i  | %10lu | %i\n",
          dev->mac,
          dev->last_channel,
          dev->last_rssi,
          (uptime_ms - dev->last_timestamp) / 1000,
          dev->seen_count);
    }
  }
#endif

} // processSendBuffer

void flushQueues() {
#ifdef HAS_LORA
  xQueueReset(LoraSendQueue);
#endif
#ifdef HAS_SPI
  xQueueReset(SPISendQueue);
#endif
}