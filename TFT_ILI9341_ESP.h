/***************************************************
  Arduino TFT graphics library targetted at ESP8266
  based boards. (ESP32 support is planned!)

  This library has been derived from the Adafruit_GFX
  library and the associated driver library. See text
  at the end of this file.

  This is a standalone library that contains the
  hardware driver, the graphics funtions and the
  proportional fonts.

  The larger fonts are Run Length Encoded to reduce
  their FLASH footprint.

 ****************************************************/

// Include header file that defines the fonts loaded and the pins to be used
#include <User_Setup.h>

// Stop fonts being loaded multiple times
#ifndef _TFT_ILI9341_ESPH_
#define _TFT_ILI9341_ESPH_

// Only load the fonts defined in User_Setup.h (to save space)
// Set flag so RLE rendering code is optionally compiled
#ifdef LOAD_GLCD
  #include <Fonts/glcdfont.c>
#endif

#ifdef LOAD_FONT2
  #include <Fonts/Font16.h>
#endif

#ifdef LOAD_FONT4
  #include <Fonts/Font32rle.h>
  #define LOAD_RLE
#endif

#ifdef LOAD_FONT6
  #include <Fonts/Font64rle.h>
  #ifndef LOAD_RLE
    #define LOAD_RLE
  #endif
#endif

#ifdef LOAD_FONT7
  #include <Fonts/Font7srle.h>
  #ifndef LOAD_RLE
    #define LOAD_RLE
  #endif
#endif

#ifdef LOAD_FONT8
  #include <Fonts/Font72rle.h>
  #ifndef LOAD_RLE
    #define LOAD_RLE
  #endif
#endif

#include <Arduino.h>
#include <Print.h>

#include <pgmspace.h>

#include <SPI.h>

#define USE_FAST_PINIO

#if defined (ESP32) || defined (D0_USED_FOR_DC)
  #define DC_C digitalWrite(TFT_DC, LOW)
  #define DC_D digitalWrite(TFT_DC, HIGH)
#else
  #define DC_C GPOC = dcpinmask
  #define DC_D GPOS = dcpinmask
#endif

#ifndef NO_PIN_USED_FOR_CS
  #if defined (ESP32) || defined (D0_USED_FOR_CS)
    #define CS_L digitalWrite(TFT_CS, LOW)
    #define CS_H digitalWrite(TFT_CS, HIGH)
  #else
    #define CS_L GPOC = cspinmask
    #define CS_H GPOS = cspinmask
  #endif
#else
	#define CS_L
    #define CS_H
#endif

// We can include all the free fonts and they will only be built into
// the sketch if they are used

#include <Fonts/GFXFF/gfxfont.h>

// New custom fonts
#include <Fonts/Custom/Orbitron_Light_24.h> // CF_OL24
#include <Fonts/Custom/Orbitron_Light_32.h> // CF_OL32
#include <Fonts/Custom/Roboto_Thin_24.h>    // CF_RT24
#include <Fonts/Custom/Satisfy_24.h>        // CF_S24
#include <Fonts/Custom/Yellowtail_32.h>     // CF_Y32

// Free fonts
#include <Fonts/GFXFF/TomThumb.h>  // TT1

#include <Fonts/GFXFF/FreeMono9pt7b.h>  // FF1 or FM9
#include <Fonts/GFXFF/FreeMono12pt7b.h> // FF2 or FM12
#include <Fonts/GFXFF/FreeMono18pt7b.h> // FF3 or FM18
#include <Fonts/GFXFF/FreeMono24pt7b.h> // FF4 or FM24

#include <Fonts/GFXFF/FreeMonoOblique9pt7b.h>  // FF5 or FMO9
#include <Fonts/GFXFF/FreeMonoOblique12pt7b.h> // FF6 or FMO12
#include <Fonts/GFXFF/FreeMonoOblique18pt7b.h> // FF7 or FMO18
#include <Fonts/GFXFF/FreeMonoOblique24pt7b.h> // FF8 or FMO24

#include <Fonts/GFXFF/FreeMonoBold9pt7b.h>  // FF9  or FMB9
#include <Fonts/GFXFF/FreeMonoBold12pt7b.h> // FF10 or FMB12
#include <Fonts/GFXFF/FreeMonoBold18pt7b.h> // FF11 or FMB18
#include <Fonts/GFXFF/FreeMonoBold24pt7b.h> // FF12 or FMB24

#include <Fonts/GFXFF/FreeMonoBoldOblique9pt7b.h>  // FF13 or FMBO9
#include <Fonts/GFXFF/FreeMonoBoldOblique12pt7b.h> // FF14 or FMBO12
#include <Fonts/GFXFF/FreeMonoBoldOblique18pt7b.h> // FF15 or FMBO18
#include <Fonts/GFXFF/FreeMonoBoldOblique24pt7b.h> // FF16 or FMBO24

// Sans serif fonts
#include <Fonts/GFXFF/FreeSans9pt7b.h>  // FF17 or FSS9
#include <Fonts/GFXFF/FreeSans12pt7b.h> // FF18 or FSS12
#include <Fonts/GFXFF/FreeSans18pt7b.h> // FF19 or FSS18
#include <Fonts/GFXFF/FreeSans24pt7b.h> // FF20 or FSS24

#include <Fonts/GFXFF/FreeSansOblique9pt7b.h>  // FF21 or FSSO9
#include <Fonts/GFXFF/FreeSansOblique12pt7b.h> // FF22 or FSSO12
#include <Fonts/GFXFF/FreeSansOblique18pt7b.h> // FF23 or FSSO18
#include <Fonts/GFXFF/FreeSansOblique24pt7b.h> // FF24 or FSSO24

#include <Fonts/GFXFF/FreeSansBold9pt7b.h>  // FF25 or FSSB9
#include <Fonts/GFXFF/FreeSansBold12pt7b.h> // FF26 or FSSB12
#include <Fonts/GFXFF/FreeSansBold18pt7b.h> // FF27 or FSSB18
#include <Fonts/GFXFF/FreeSansBold24pt7b.h> // FF28 or FSSB24

#include <Fonts/GFXFF/FreeSansBoldOblique9pt7b.h>  // FF29 or FSSBO9
#include <Fonts/GFXFF/FreeSansBoldOblique12pt7b.h> // FF30 or FSSBO12
#include <Fonts/GFXFF/FreeSansBoldOblique18pt7b.h> // FF31 or FSSBO18
#include <Fonts/GFXFF/FreeSansBoldOblique24pt7b.h> // FF32 or FSSBO24

// Serif fonts
#include <Fonts/GFXFF/FreeSerif9pt7b.h>  // FF33 or FS9
#include <Fonts/GFXFF/FreeSerif12pt7b.h> // FF34 or FS12
#include <Fonts/GFXFF/FreeSerif18pt7b.h> // FF35 or FS18
#include <Fonts/GFXFF/FreeSerif24pt7b.h> // FF36 or FS24

#include <Fonts/GFXFF/FreeSerifItalic9pt7b.h>  // FF37 or FSI9
#include <Fonts/GFXFF/FreeSerifItalic12pt7b.h> // FF38 or FSI12
#include <Fonts/GFXFF/FreeSerifItalic18pt7b.h> // FF39 or FSI18
#include <Fonts/GFXFF/FreeSerifItalic24pt7b.h> // FF40 or FSI24

#include <Fonts/GFXFF/FreeSerifBold9pt7b.h>  // FF41 or FSB9
#include <Fonts/GFXFF/FreeSerifBold12pt7b.h> // FF42 or FSB12
#include <Fonts/GFXFF/FreeSerifBold18pt7b.h> // FF43 or FSB18
#include <Fonts/GFXFF/FreeSerifBold24pt7b.h> // FF44 or FSB24

#include <Fonts/GFXFF/FreeSerifBoldItalic9pt7b.h>  // FF45 or FSBI9
#include <Fonts/GFXFF/FreeSerifBoldItalic12pt7b.h> // FF46 or FSBI12
#include <Fonts/GFXFF/FreeSerifBoldItalic18pt7b.h> // FF47 or FSBI18
#include <Fonts/GFXFF/FreeSerifBoldItalic24pt7b.h> // FF48 or FSBI24

// Swap any type
template <typename T> static inline void
swap(T& a, T& b) { T t = a; a = b; b = t; }

//These enumerate the text plotting alignment (reference datum point)
#define TL_DATUM 0 // Top left (default)
#define TC_DATUM 1 // Top centre
#define TR_DATUM 2 // Top right
#define ML_DATUM 3 // Middle left
#define CL_DATUM 3 // Centre left, same as above
#define MC_DATUM 4 // Middle centre
#define CC_DATUM 4 // Centre centre, same as above
#define MR_DATUM 5 // Middle right
#define CR_DATUM 5 // Centre right, same as above
#define BL_DATUM 6 // Bottom left
#define BC_DATUM 7 // Bottom centre
#define BR_DATUM 8 // Bottom right
#define L_BASELINE  9 // Left character baseline (Line the 'A' character would sit on)
#define C_BASELINE 10 // Centre character baseline
#define R_BASELINE 11 // Right character baseline


// Change the width and height if required (defined in portrait mode)
// or use the constructor to over-ride defaults
#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_INIT_DELAY 0x80

// These are the ILI9341 control registers
#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_VSCRDEF 0x33
#define ILI9341_MADCTL  0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT  0x3A

#define ILI9341_WRDISBV  0x51
#define ILI9341_RDDISBV  0x52
#define ILI9341_WRCTRLD  0x53

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID4   0xD3
#define ILI9341_RDINDEX 0xD9
#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDIDX   0xDD // TBC

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1

#define ILI9341_MADCTL_MY  0x80
#define ILI9341_MADCTL_MX  0x40
#define ILI9341_MADCTL_MV  0x20
#define ILI9341_MADCTL_ML  0x10
#define ILI9341_MADCTL_RGB 0x00
#define ILI9341_MADCTL_BGR 0x08
#define ILI9341_MADCTL_MH  0x04

// New color definitions use for all my libraries
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFD20      /* 255, 165,   0 */
#define TFT_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define TFT_PINK        0xF81F

// Color definitions for backwards compatibility
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

// This is a structure to conveniently hold infomation on the default fonts
// Stores pointer to font character image address table, width table and height

typedef struct {
	const uint8_t *chartbl;
	const uint8_t *widthtbl;
	uint8_t height;
	uint8_t baseline;
	} fontinfo;

// Now fill the structure
const PROGMEM fontinfo fontdata [] = {
   { 0, 0, 0, 0 },

   // GLCD font (Font 1) does not have all parameters
   { 0, 0, 8, 7 },

  #ifdef LOAD_FONT2
   { (const uint8_t *)chrtbl_f16, widtbl_f16, chr_hgt_f16, baseline_f16},
  #else
   { 0, 0, 0, 0 },
  #endif

   // Font 3 current unused
   { 0, 0, 0, 0 },

  #ifdef LOAD_FONT4
   { (const uint8_t *)chrtbl_f32, widtbl_f32, chr_hgt_f32, baseline_f32},
  #else
   { 0, 0, 0, 0 },
  #endif

   // Font 5 current unused
   { 0, 0, 0, 0 },

  #ifdef LOAD_FONT6
   { (const uint8_t *)chrtbl_f64, widtbl_f64, chr_hgt_f64, baseline_f64},
  #else
   { 0, 0, 0, 0 },
  #endif

  #ifdef LOAD_FONT7
   { (const uint8_t *)chrtbl_f7s, widtbl_f7s, chr_hgt_f7s, baseline_f7s},
  #else
   { 0, 0, 0, 0 },
  #endif

  #ifdef LOAD_FONT8
   { (const uint8_t *)chrtbl_f72, widtbl_f72, chr_hgt_f72, baseline_f72}
  #else
   { 0, 0, 0, 0 }
  #endif
};



// Class functions and variables
class TFT_ILI9341_ESP : public Print {

 public:

  TFT_ILI9341_ESP(int16_t _W = ILI9341_TFTWIDTH, int16_t _H = ILI9341_TFTHEIGHT);

  void     init(void), begin(void); // Same - begin included for backwards compatibility

  void     drawPixel(uint32_t x, uint32_t y, uint32_t color);

  void     drawChar(int32_t x, int32_t y, unsigned char c, uint32_t color, uint32_t bg, uint8_t font),
           setWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1),

           pushColor(uint16_t color),
           pushColor(uint16_t color, uint16_t len),

           pushColors(uint16_t *data, uint8_t len),
           pushColors(uint8_t  *data, uint32_t len),

           fillScreen(uint32_t color),

           writeEnd(void),
           backupSPCR(void),
           restoreSPCR(void),

           drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color),
           drawFastVLine(int32_t x, int32_t y, int32_t h, uint32_t color),
           drawFastHLine(int32_t x, int32_t y, int32_t w, uint32_t color),

           drawRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color),
           fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color),
           drawRoundRect(int32_t x0, int32_t y0, int32_t w, int32_t h, int32_t radius, uint32_t color),
           fillRoundRect(int32_t x0, int32_t y0, int32_t w, int32_t h, int32_t radius, uint32_t color),

           setRotation(uint8_t r),
           invertDisplay(boolean i),

           drawCircle(int32_t x0, int32_t y0, int32_t r, uint32_t color),
           drawCircleHelper(int32_t x0, int32_t y0, int32_t r, uint8_t cornername, uint32_t color),
           fillCircle(int32_t x0, int32_t y0, int32_t r, uint32_t color),
           fillCircleHelper(int32_t x0, int32_t y0, int32_t r, uint8_t cornername, int32_t delta, uint32_t color),

           drawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color),
           fillEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color),

           drawTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color),
           fillTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color),

           drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color),

           setCursor(int16_t x, int16_t y),
           setCursor(int16_t x, int16_t y, uint8_t font),
           setTextColor(uint16_t color),
           setTextColor(uint16_t fgcolor, uint16_t bgcolor),
           setTextSize(uint8_t size),

           setTextWrap(boolean wrap),
           setTextDatum(uint8_t datum),
           setTextPadding(uint16_t x_width),

#ifdef LOAD_GFXFF
           setFreeFont(const GFXfont *f = NULL),
           setTextFont(uint8_t font),
#else
           setFreeFont(uint8_t font),
           setTextFont(uint8_t font),
#endif
           spiwrite(uint8_t),
           writecommand(uint8_t c),
           writedata(uint8_t d),
           commandList(const uint8_t *addr);

  uint8_t  readcommand8(uint8_t cmd_function, uint8_t index);
  uint16_t readcommand16(uint8_t cmd_function, uint8_t index);
  uint32_t readcommand32(uint8_t cmd_function, uint8_t index);

           // Read the colour of a pixel at x,y and return value in 565 format 
  uint16_t readPixel(int32_t x0, int32_t y0);

           // The next functions can be used as a pair to copy screen blocks (or horizontal/vertical lines) to another location
		   // Read a block of pixels to a data buffer, buffer is 16 bit and the array size must be at least w * h
  void     readRect(uint32_t x0, uint32_t y0, uint32_t w, uint32_t h, uint16_t *data);
		   // Write a block of pixels to the screen
  void     pushRect(uint32_t x0, uint32_t y0, uint32_t w, uint32_t h, uint16_t *data);

		   // This next function has been used successfully to dump the TFT screen to a PC for documentation purposes
		   // It reads a screen area and returns the RGB 8 bit colour values of each pixel
		   // Set w and h to 1 to read 1 pixel's colour. The data buffer must be at least w * h * 3 bytes
  void     readRectRGB(int32_t x0, int32_t y0, int32_t w, int32_t h, uint8_t *data);

  uint8_t  getRotation(void);

  uint16_t fontsLoaded(void),
           color565(uint8_t r, uint8_t g, uint8_t b);

  int16_t  drawChar(unsigned int uniCode, int x, int y, int font),
		   drawChar(unsigned int uniCode, int x, int y),
           drawNumber(long long_num,int poX, int poY, int font),
		   drawNumber(long long_num,int poX, int poY),
           drawFloat(float floatNumber,int decimal,int poX, int poY, int font),
           drawFloat(float floatNumber,int decimal,int poX, int poY),
		   
		   // Handle char arrays
           drawString(const char *string, int poX, int poY, int font),
           drawString(const char *string, int poX, int poY),
           drawCentreString(const char *string, int dX, int poY, int font), // Deprecated, use setTextDatum() and drawString()
           drawRightString(const char *string, int dX, int poY, int font),  // Deprecated, use setTextDatum() and drawString()

		   // Handle String type
		   drawString(const String& string, int poX, int poY, int font),
		   drawString(const String& string, int poX, int poY),
           drawCentreString(const String& string, int dX, int poY, int font), // Deprecated, use setTextDatum() and drawString()
           drawRightString(const String& string, int dX, int poY, int font);  // Deprecated, use setTextDatum() and drawString()
		   
  int16_t  height(void),
           width(void),
           textWidth(const char *string, int font),
		   textWidth(const char *string),
		   textWidth(const String& string, int font),
		   textWidth(const String& string),
           fontHeight(int16_t font);

    void   setAddrWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1);

 virtual   size_t write(uint8_t);

 private:

  uint8_t  tabcolor;
  
            SPIClass *_SPI;
inline void spi_begin() __attribute__((always_inline));
inline void spi_end() __attribute__((always_inline));




void  writeBytes_(uint8_t * data, uint8_t size);
inline void setDataBits(uint16_t bits);




  boolean  hwSPI;

  volatile uint32_t *mosiport, *clkport, *dcport, *rsport, *csport;
  int32_t  _cs, _dc, _rst, _mosi, _miso, _sclk;
  uint32_t  mosipinmask, clkpinmask, cspinmask, dcpinmask;

  uint8_t  mySPCR, savedSPCR;

  //uint8_t  fifoBuffer[64];
  uint8_t  colorBin[2];
  uint32_t lastColor = 0xFFFF;

 protected:

  int32_t  cursor_x, cursor_y, win_xe, win_ye, padX;

  uint32_t _width, _height, // Display w/h as modified by current rotation
           textcolor, textbgcolor, fontsloaded, addr_row, addr_col;

  uint8_t  glyph_ab,  // glyph height above baseline
           glyph_bb,  // glyph height below baseline
           textfont,  // Current selected font
           textsize,  // Current font size multiplier
           textdatum, // Text reference datum
           rotation;  // Display rotation (0-3)

  boolean  textwrap; // If set, 'wrap' text at right edge of display

#ifdef LOAD_GFXFF
  GFXfont
    *gfxFont;
#endif

};

#endif

/***************************************************

  ORIGINAL LIBRARY HEADER

  This is our library for the Adafruit  ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution

  Updated with new functions by Bodmer 14/4/15
 ****************************************************/
