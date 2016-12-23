//                            USER DEFINED SETTINGS V16
//            Set fonts to be loaded, pins used and SPI control method

// ##################################################################################
//
// Define the pins that are used to interface with the display here
//
// ##################################################################################

// We must use hardware SPI
// Typical setup for NodeMCU ESP-12 is :
//
// Display SDO/MISO to NodeMCU pin D6
// Display LED      to NodeMCU pin VIN (5V)
// Display SCK      to NodeMCU pin D5
// Display SDI/MOSI to NodeMCU pin D7
// Display DC       to NodeMCU pin D3
// Display RESET    to NodeMCU pin D4
// Display GND      to NodeMCU pin GND (0V)
// Display VCC      to NodeMCU pin VIN (5V)

// ###### EDIT THE PIN NUMBERS IN THE 3 LINES FOLLOWING TO SUIT YOUR SETUP ######

#define TFT_CS   D8  // Chip select control pin
#define TFT_DC   D3  // Data Command control pin
#define TFT_RST  D4  // Reset pin (could connect to Arduino RESET pin)

// ##################################################################################
//
// Define the fonts that are to be used here
//
// ##################################################################################

// Comment out the #defines below with // to stop that font being loaded
// The ESP8366 had plenty of memory so commenting out fonts is not normally necessary
// If all fonts are loaded the extra FLASH space required is about 17000 bytes...
// To save FLASH space only enable the fonts you need!

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:.
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.

// ##################################################################################
//
// Other speed up options
//
// ##################################################################################

// Define the SPI clock frequency (40MHz works OK) 80MHz sometimes fails

// #define SPI_FREQUENCY 20000000
 #define SPI_FREQUENCY 40000000
// #define SPI_FREQUENCY 80000000


// Comment out the following #define if "SPI Transactions" do not need to be
// supported. Tranaction support is required if other SPI devices are connected.
// When commented out the code size will be smaller and sketches will
// run slightly faster, so leave it commented out unless you need it!
// Transaction support is needed to work with SD library but not needed with TFT_SdFat

// #define SUPPORT_TRANSACTIONS

