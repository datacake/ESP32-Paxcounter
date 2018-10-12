// Basic Config
#include "globals.h"
#include "wifiscan.h"
#include "macsniff.h"

// Local logging tag
static const char TAG[] = "wifi";

// using IRAM_:ATTR here to speed up callback function
IRAM_ATTR void wifi_sniffer_packet_handler(void *buff,
                                           wifi_promiscuous_pkt_type_t type) {
  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
  const wifi_ieee80211_packet_t *ipkt =
      (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  if ((cfg.rssilimit) &&
      (ppkt->rx_ctrl.rssi < cfg.rssilimit)) // rssi is negative value
    ESP_LOGD(TAG, "WiFi RSSI %d -> ignoring (limit: %d)", ppkt->rx_ctrl.rssi,
             cfg.rssilimit);
  else // count seen MAC
    mac_add((uint8_t *)hdr->addr2, ppkt->rx_ctrl.rssi, ppkt->rx_ctrl.channel );
}

void wifi_sniffer_init(void) {
  wifi_init_config_t wifiCfg = WIFI_INIT_CONFIG_DEFAULT();
  wifiCfg.nvs_enable = 0; // we don't need any wifi settings from NVRAM
  wifi_promiscuous_filter_t filter = {
      .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT}; // we need only MGMT frames
  ESP_ERROR_CHECK(esp_wifi_init(&wifiCfg));             // configure Wifi with wifiCfg
  ESP_ERROR_CHECK(
      esp_wifi_set_country(&cfg.wifi)); // set locales for RF and channels
  ESP_ERROR_CHECK(
      esp_wifi_set_storage(WIFI_STORAGE_RAM)); // we don't need NVRAM
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  ESP_ERROR_CHECK(
      esp_wifi_set_promiscuous_filter(&filter)); // set MAC frame filter
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true)); // now switch on monitor mode
}

void wifi_country_set( const wifi_country_t* country )
{
  ESP_ERROR_CHECK(
      esp_wifi_set_country(country)); // set locales for RF and channels
}

// Wifi channel rotation task
void wifi_channel_loop(void *pvParameters) {

  configASSERT(((uint32_t)pvParameters) == 1); // FreeRTOS check

  while (1) {

    if (ChannelTimerIRQ) {
      portENTER_CRITICAL(&timerMux);
      ChannelTimerIRQ = 0;
      portEXIT_CRITICAL(&timerMux);
      // rotates variable channel 1..WIFI_CHANNEL_MAX
      uint8_t new_channel = (channel % cfg.wifi.nchan) + cfg.wifi.schan;
      if( new_channel != channel )
      {
        esp_wifi_set_channel(new_channel, WIFI_SECOND_CHAN_NONE);
        ESP_LOGD(TAG, "Wifi set channel %d", new_channel);
        channel = new_channel;
      }
    }

    vTaskDelay(1 * portTICK_PERIOD_MS); // reset watchdog

  } // end of infinite wifi channel rotation loop
}

// IRQ handler
void IRAM_ATTR ChannelSwitchIRQ() {
  portENTER_CRITICAL(&timerMux);
  ChannelTimerIRQ++;
  portEXIT_CRITICAL(&timerMux);
}