#ifndef _GLOBALS_H
#define _GLOBALS_H

// The mother of all embedded development...
#include <Arduino.h>

// attn: increment version after modifications to configData_t truct!
#define PROGVERSION "1.4.23" // use max 10 chars here!
#define PROGNAME "PAXCNT"

#define ROLE_STANDALONE 0
#define ROLE_PARENT     1
#define ROLE_CHILD      2
#define DEVICE_ROLE ROLE_PARENT

// std::set for unified array functions
#include <list>
#include <array>
#include <algorithm>

// for use of 'wifi_countr_t' struct
#include <esp_wifi.h>

// for PacketEvent
#include "PacketEvent.h"

// Struct holding devices's runtime configuration
typedef struct {
  uint8_t lorasf;      // 7-12, lora spreadfactor
  uint8_t txpower;     // 2-15, lora tx power
  uint8_t adrmode;     // 0=disabled, 1=enabled
  uint8_t screensaver; // 0=disabled, 1=enabled
  uint8_t screenon;    // 0=disabled, 1=enabled
  uint8_t countermode; // 0=cyclic unconfirmed, 1=cumulative, 2=cyclic confirmed
  int16_t rssilimit;   // threshold for rssilimiter, negative value!
  uint8_t sendcycle;   // payload send cycle [seconds/2]
  uint8_t wifichancycle; // wifi channel switch cycle [seconds/100]
  uint8_t blescantime;   // BLE scan cycle duration [seconds]
  uint8_t blescan;       // 0=disabled, 1=enabled
  uint8_t wifiant;       // 0=internal, 1=external (for LoPy/LoPy4)
  uint8_t vendorfilter;  // 0=disabled, 1=enabled
  uint8_t rgblum;        // RGB Led luminosity (0..100%)
  uint8_t gpsmode;       // 0=disabled, 1=enabled
  uint8_t monitormode;   // 0=disabled, 1=enabled
  char version[10];      // Firmware version
  wifi_country_t wifi;   // wifi settings
} configData_t;

// Struct holding payload for data send queue
typedef struct {
  uint8_t MessageSize;
  uint8_t MessagePort;
  uint8_t Message[PAYLOAD_BUFFER_SIZE];
} MessageBuffer_t;

struct FoundDevice {
  uint64_t mac;
  int8_t last_rssi;
  uint8_t last_channel;
  time_t last_timestamp;
  uint32_t seen_count;
};

inline bool operator<(const FoundDevice& lhs, const FoundDevice& rhs)
{
  return lhs.last_timestamp > rhs.last_timestamp;
}

struct FoundDeviceByMac {
  uint64_t mac_to_find;

  FoundDeviceByMac( uint64_t _mac_to_find ) : mac_to_find( _mac_to_find ) {}

  bool operator()(const FoundDevice& dev) const {
    return dev.mac == mac_to_find;
  }
};

// global variables
#if DEVICE_ROLE == ROLE_CHILD
extern std::list<PacketEvent*> packets;
extern portMUX_TYPE packetListMutex;
#elif (DEVICE_ROLE == ROLE_STANDALONE) || (DEVICE_ROLE == ROLE_PARENT)
extern std::list<FoundDevice> macs; // temp storage for MACs
extern uint16_t macs_total, macs_wifi, macs_ble;
#endif
extern configData_t cfg;                      // current device configuration
extern char display_line6[], display_line7[]; // screen buffers
extern uint8_t channel;                       // wifi channel rotation counter
extern uint16_t batt_voltage; // display values
extern hw_timer_t *channelSwitch, *sendCycle;
extern portMUX_TYPE timerMux;
extern volatile int SendCycleTimerIRQ, HomeCycleIRQ, DisplayTimerIRQ,
    ChannelTimerIRQ, ButtonPressedIRQ;
extern QueueHandle_t LoraSendQueue, SPISendQueue;

extern std::array<uint64_t, 0xff>::iterator it;
extern std::array<uint64_t, 0xff> beacons;

#ifdef HAS_GPS
#include "gps.h"
#endif

#ifdef HAS_LED
#include "led.h"
#endif

#include "payload.h"

#ifdef HAS_LORA
#include "lorawan.h"
#endif

#if defined(HAS_SPI) && defined(HAS_SPI_SLAVE)
#include "spi_slave.h"
#endif

#if (DEVICE_ROLE == ROLE_PARENT)
#include "spi_master.h"
#endif

#ifdef HAS_DISPLAY
#include "display.h"
#endif

#ifdef HAS_BUTTON
#include "button.h"
#endif

#if BLECOUNTER
#include "blescan.h"
#endif

#ifdef HAS_BATTERY_PROBE
#include "battery.h"
#endif

#ifdef HAS_ANTENNA_SWITCH
#include "antenna.h"
#endif

void reset_counters(void);
void blink_LED(uint16_t set_color, uint16_t set_blinkduration);
uint64_t uptime();

#endif