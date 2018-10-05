
// Basic Config
#include "macsniff.h"
#include "globals.h"

#ifdef VENDORFILTER
#include "vendor_array.h"
#endif

// Local logging tag
static const char TAG[] = "main";

uint16_t salt;

uint16_t reset_salt(void) {
  salt = random(65536); // get new 16bit random for salting hashes
  return salt;
}

int8_t isBeacon(uint64_t mac) {
  it = std::find(beacons.begin(), beacons.end(), mac);
  if (it != beacons.end())
    return std::distance(beacons.begin(), it);
  else
    return -1;
}

// Display a key
void printKey(const char *name, const uint8_t *key, uint8_t len, bool lsb) {
  const uint8_t *p;
  char keystring[len + 1] = "", keybyte[3];
  for (uint8_t i = 0; i < len; i++) {
    p = lsb ? key + len - i - 1 : key + i;
    sprintf(keybyte, "%02X", *p);
    strncat(keystring, keybyte, 2);
  }
  ESP_LOGI(TAG, "%s: %s", name, keystring);
}

uint64_t macConvert(uint8_t *paddr) {
  return ((uint64_t)paddr[5]) | ((uint64_t)paddr[4] << 8) |
         ((uint64_t)paddr[3] << 16) | ((uint64_t)paddr[2] << 24) |
         ((uint64_t)paddr[1] << 32) | ((uint64_t)paddr[0] << 40);
}

#if DEVICE_ROLE == ROLE_CHILD
// As a child device we will report all packets to the parent device
// add all packets to a queue here
bool mac_add(uint8_t *paddr, int8_t rssi, bool sniff_type, int channel) {

  // explicitly allocate RAM in external PSRAM
  PacketEvent *p = (PacketEvent*) heap_caps_malloc( sizeof(PacketEvent) , MALLOC_CAP_SPIRAM);

  if( p != nullptr )
  {
    memcpy( p->mac, paddr, 6 );
    p->rssi = rssi;
    p->channel = channel;
    gettimeofday( &p->timestamp, NULL );
  
    portENTER_CRITICAL(&packetListMutex);
    packets.push_back( p );
    portEXIT_CRITICAL(&packetListMutex);
  }
  else
  {
    ESP_LOGE( TAG, "could not create PacketEvent" );
  }

  uint64_t mac = macConvert( paddr );

  // Log scan result
  ESP_LOGI(TAG,
    "%s RSSI %ddBi -> MAC %012llX -> %08X addr -> %d Bytes left",
    sniff_type == MAC_SNIFF_WIFI ? "WiFi" : "BLTH",
    rssi,
    mac,
    p,
    esp_get_free_heap_size());

  return true;
}
#else
bool mac_add(uint8_t *paddr, int8_t rssi, bool sniff_type, int channel) {

  char buff[16]; // temporary buffer for printf
  bool added = false;
  int8_t beaconID;    // beacon number in test monitor mode
  uint16_t hashedmac; // temporary buffer for generated hash value
  uint32_t addr2int;  // temporary buffer for shortened MAC
#ifdef VENDORFILTER
  uint32_t vendor2int; // temporary buffer for Vendor OUI
#endif

  const uint64_t mac = macConvert( paddr );

  // only last 3 MAC Address bytes are used for MAC address anonymization
  // but since it's uint32 we take 4 bytes to avoid 1st value to be 0
  addr2int = ((uint32_t)paddr[2]) | ((uint32_t)paddr[3] << 8) |
             ((uint32_t)paddr[4] << 16) | ((uint32_t)paddr[5] << 24);

#ifdef VENDORFILTER
  vendor2int = ((uint32_t)paddr[2]) | ((uint32_t)paddr[1] << 8) |
               ((uint32_t)paddr[0] << 16);
  // use OUI vendor filter list only on Wifi, not on BLE
  if ((sniff_type == MAC_SNIFF_BLE) ||
      std::find(vendors.begin(), vendors.end(), vendor2int) != vendors.end()) {
#endif

    // salt and hash MAC, and if new unique one, store identifier in container
    // and increment counter on display
    // https://en.wikipedia.org/wiki/MAC_Address_Anonymization

    snprintf(buff, sizeof(buff), "%08X",
             addr2int + (uint32_t)salt); // convert usigned 32-bit salted MAC to
                                         // 8 digit hex string
    hashedmac = rokkit(&buff[3], 5); // hash MAC last string value, use 5 chars
                                     // to fit hash in uint16_t container

    auto iter = std::find_if( macs.begin(), macs.end(), FoundDeviceByMac( mac ));
    FoundDevice* dev = nullptr;

    // do we know this mac yet?
    if( iter == macs.end() )
    {
      // new device

      FoundDevice f;
      f.mac = mac;
      f.seen_count = 0;

      auto newmac = macs.insert( macs.end(), f); // add hashed MAC, if new unique

      dev = &(*newmac);

      added = true;
    }
    else
    {
      // known device
      dev = &*iter;
    }

    if(dev == nullptr)
    {
      ESP_LOGE( TAG, "dev==nullptr");
      return false;
    }

    int uptime_ms = esp_timer_get_time() / 1000;

    dev->last_rssi = rssi;
    dev->last_channel = channel;
    dev->last_timestamp = uptime_ms;
    dev->seen_count++;

    // Count only if MAC was not yet seen
    if (added) {

      // increment counter and one blink led
      if (sniff_type == MAC_SNIFF_WIFI) {
        macs_wifi++; // increment Wifi MACs counter
#if (HAS_LED != NOT_A_PIN) || defined(HAS_RGB_LED)
        blink_LED(COLOR_GREEN, 50);
#endif
      }
#if BLECOUNTER
      else if (sniff_type == MAC_SNIFF_BLE) {
        macs_ble++; // increment BLE Macs counter
#if (HAS_LED != NOT_A_PIN) || defined(HAS_RGB_LED)
        blink_LED(COLOR_MAGENTA, 50);
#endif
      }
#endif

      // in beacon monitor mode check if seen MAC is a known beacon
      if (cfg.monitormode) {
        beaconID = isBeacon(macConvert(paddr));
        if (beaconID >= 0) {
          ESP_LOGI(TAG, "Beacon ID#%d detected", beaconID);
#if (HAS_LED != NOT_A_PIN) || defined(HAS_RGB_LED)
          blink_LED(COLOR_WHITE, 2000);
#endif
          payload.reset();
          payload.addAlarm(rssi, beaconID);
          SendData(BEACONPORT);
        }
      };
    } // added

    // Log scan result
    ESP_LOGD(TAG,
             "%s %s RSSI %ddBi -> MAC %s -> Hash %04X -> WiFi:%d  BLTH:%d -> "
             "%d Bytes left",
             added ? "new  " : "known",
             sniff_type == MAC_SNIFF_WIFI ? "WiFi" : "BLTH", rssi, buff,
             hashedmac, macs_wifi, macs_ble, ESP.getFreeHeap());
#ifdef VENDORFILTER
  } else {
    // Very noisy
    // ESP_LOGD(TAG, "Filtered MAC %02X:%02X:%02X:%02X:%02X:%02X",
    // paddr[0],paddr[1],paddr[2],paddr[3],paddr[5],paddr[5]);
  }
#endif

  // True if MAC WiFi/BLE was new
  return added; // function returns bool if a new and unique Wifi or BLE mac was
                // counted (true) or not (false)
}

#endif
