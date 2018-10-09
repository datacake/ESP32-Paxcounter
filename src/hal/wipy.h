// Hardware related definitions for Pycom WiPy Board

#define HAS_SPI 1                   // comment out if device shall not send data via SPI
#define CFG_sx1276_radio 1
#define HAS_LED NOT_A_PIN           // WiPy has no on board LED, so we use RGB LED on WiPy
#define HAS_RGB_LED   GPIO_NUM_0    // WS2812B RGB LED on GPIO0
//#define BOARD_HAS_PSRAM             // use extra 4MB extern RAM

// Hardware pins for SPI SLAVE
#define HAS_SPI_SLAVE 1
#define PIN_SPI_SLAVE_SS    GPIO_NUM_38
#define PIN_SPI_SLAVE_MOSI  GPIO_NUM_36
#define PIN_SPI_SLAVE_MISO  GPIO_NUM_22
#define PIN_SPI_SLAVE_SCK   GPIO_NUM_32

// Hardware pins for SPI MASTER
#define HAS_SPI_MASTER 1
#define PIN_SPI_MASTER_MOSI GPIO_NUM_26
#define PIN_SPI_MASTER_MISO GPIO_NUM_35
#define PIN_SPI_MASTER_SCK  GPIO_NUM_4
#define PIN_SPI_MASTER_SS1  GPIO_NUM_27
#define PIN_SPI_MASTER_SS2  GPIO_NUM_5
#define PIN_SPI_MASTER_SS3  GPIO_NUM_1
#define PIN_SPI_MASTER_SS4  GPIO_NUM_0
#define PIN_SPI_MASTER_SS5  GPIO_NUM_33
#define PIN_SPI_MASTER_SS6  GPIO_NUM_25
#define PIN_SPI_MASTER_SS7  GPIO_NUM_14
#define PIN_SPI_MASTER_SS8  GPIO_NUM_19
#define PIN_SPI_MASTER_SS9  GPIO_NUM_2
#define PIN_SPI_MASTER_SS10 GPIO_NUM_12
#define PIN_SPI_MASTER_SS11 GPIO_NUM_15
#define PIN_SPI_MASTER_SS12 GPIO_NUM_13

// select WIFI antenna (internal = onboard / external = u.fl socket)
#define HAS_ANTENNA_SWITCH  21      // pin for switching wifi antenna
#define WIFI_ANTENNA 0              // 0 = internal, 1 = external

// !!EXPERIMENTAL - not tested yet!!
// uncomment this only if your LoPy runs on a Pytrack expansion board with GPS
// see http://www.quectel.com/UploadImage/Downlad/Quectel_L76-L_I2C_Application_Note_V1.0.pdf
//#define HAS_GPS 1
//#define GPS_QUECTEL_L76 GPIO_NUM_25, GPIO_NUM_26 // SDA (P22), SCL (P21)
//#define GPS_ADDR 0x10
//#define HAS_BUTTON GPIO_NUM_37 // (P14)
//#define BUTTON_PULLUP 1  // Button need pullup instead of default pulldown

// uncomment this only if your LoPy runs on a expansion board 3.0
// #define HAS_BATTERY_PROBE ADC1_GPIO39_CHANNEL // battery probe GPIO pin -> ADC1_CHANNEL_7
// #define BATT_FACTOR 2 // voltage divider 1MOhm/1MOhm on board
// #define HAS_BUTTON GPIO_NUM_37 // (P14)
// #define BUTTON_PULLUP 1  // Button need pullup instead of default pulldown

// uncomment this only if your LoPy runs on a expansion board 2.0
//#define HAS_BATTERY_PROBE ADC1_GPIO39_CHANNEL // battery probe GPIO pin -> ADC1_CHANNEL_7
//#define BATT_FACTOR 4 // voltage divider 115kOhm/56kOhm on board
//#define HAS_BUTTON GPIO_NUM_13 // (P10)
//#define BUTTON_PULLUP 1  // Button need pullup instead of default pulldown