#ifndef TFT_22_ILI9225_h
#define TFT_22_ILI9225_h


#include <string>
#include <algorithm>
#include <stdbool.h>
#include "gfxfont.h"

#include <driver/spi_master.h>

/* ILI9225 screen size */
#define ILI9225_LCD_WIDTH  176
#define ILI9225_LCD_HEIGHT 220


// autoincrement modes (register ILI9225_ENTRY_MODE, bit 5..3 )
enum autoIncMode_t 
{
    R2L_BottomUp,
    BottomUp_R2L,
    L2R_BottomUp,
    BottomUp_L2R,
    R2L_TopDown,
    TopDown_R2L,
    L2R_TopDown,
    TopDown_L2R
};

/* RGB 16-bit color table definition (RG565) */
#define COLOR_BLACK          0x0000      /*   0,   0,   0 */
#define COLOR_WHITE          0xFFFF      /* 255, 255, 255 */
#define COLOR_BLUE           0x001F      /*   0,   0, 255 */
#define COLOR_GREEN          0x07E0      /*   0, 255,   0 */
#define COLOR_RED            0xF800      /* 255,   0,   0 */
#define COLOR_NAVY           0x000F      /*   0,   0, 128 */
#define COLOR_DARKBLUE       0x0011      /*   0,   0, 139 */
#define COLOR_DARKGREEN      0x03E0      /*   0, 128,   0 */
#define COLOR_DARKCYAN       0x03EF      /*   0, 128, 128 */
#define COLOR_CYAN           0x07FF      /*   0, 255, 255 */
#define COLOR_TURQUOISE      0x471A      /*  64, 224, 208 */
#define COLOR_INDIGO         0x4810      /*  75,   0, 130 */
#define COLOR_DARKRED        0x8000      /* 128,   0,   0 */
#define COLOR_OLIVE          0x7BE0      /* 128, 128,   0 */
#define COLOR_GRAY           0x8410      /* 128, 128, 128 */
#define COLOR_GREY           0x8410      /* 128, 128, 128 */
#define COLOR_SKYBLUE        0x867D      /* 135, 206, 235 */
#define COLOR_BLUEVIOLET     0x895C      /* 138,  43, 226 */
#define COLOR_LIGHTGREEN     0x9772      /* 144, 238, 144 */
#define COLOR_DARKVIOLET     0x901A      /* 148,   0, 211 */
#define COLOR_YELLOWGREEN    0x9E66      /* 154, 205,  50 */
#define COLOR_BROWN          0xA145      /* 165,  42,  42 */
#define COLOR_DARKGRAY       0x7BEF      /* 128, 128, 128 */
#define COLOR_DARKGREY       0x7BEF      /* 128, 128, 128 */
#define COLOR_SIENNA         0xA285      /* 160,  82,  45 */
#define COLOR_LIGHTBLUE      0xAEDC      /* 172, 216, 230 */
#define COLOR_GREENYELLOW    0xAFE5      /* 173, 255,  47 */
#define COLOR_SILVER         0xC618      /* 192, 192, 192 */
#define COLOR_LIGHTGRAY      0xC618      /* 192, 192, 192 */
#define COLOR_LIGHTGREY      0xC618      /* 192, 192, 192 */
#define COLOR_LIGHTCYAN      0xE7FF      /* 224, 255, 255 */
#define COLOR_VIOLET         0xEC1D      /* 238, 130, 238 */
#define COLOR_AZUR           0xF7FF      /* 240, 255, 255 */
#define COLOR_BEIGE          0xF7BB      /* 245, 245, 220 */
#define COLOR_MAGENTA        0xF81F      /* 255,   0, 255 */
#define COLOR_TOMATO         0xFB08      /* 255,  99,  71 */
#define COLOR_GOLD           0xFEA0      /* 255, 215,   0 */
#define COLOR_ORANGE         0xFD20      /* 255, 165,   0 */
#define COLOR_SNOW           0xFFDF      /* 255, 250, 250 */
#define COLOR_YELLOW         0xFFE0      /* 255, 255,   0 */


#define MONOSPACE   1

/// Main and core class
class TFT_22_ILI9225 {

    public:

        TFT_22_ILI9225(int8_t RST, int8_t RS, int8_t CS, int8_t SDI, int8_t CLK);

        /// Initialization
        void begin(void);

        /// Clear the screen
        void clear(void); 

        /// Invert screen
        /// @param     flag true to invert, false for normal screen
        void invert(bool flag);

            /// Switch display on or off
        /// @param     flag true=on, false=off
        void setDisplay(bool flag);  

        /// Set orientation
        /// @param     orientation orientation, 0=portrait, 1=right rotated landscape, 2=reverse portrait, 3=left rotated landscape
        void setOrientation(uint8_t orientation);  

        /// Get orientation
        /// @return    orientation orientation, 0=portrait, 1=right rotated landscape, 2=reverse portrait, 3=left rotated landscape
        uint8_t getOrientation(void); 

        /// Screen size, x-axis
        /// @return   horizontal size of the screen, in pixels
        /// @note     240 means 240 pixels and thus 0..239 coordinates (decimal)
        uint16_t maxX(void);

        /// Screen size, y-axis
        /// @return   vertical size of the screen, in pixels
        /// @note     220 means 220 pixels and thus 0..219 coordinates (decimal)
        uint16_t maxY(void);

        /// Calculate 16-bit color from 8-bit Red-Green-Blue components
        /// @param    red red component, 0x00..0xff
        /// @param    green green component, 0x00..0xff
        /// @param    blue blue component, 0x00..0xff
        /// @return   16-bit color
        uint16_t setColor(uint8_t red, uint8_t green, uint8_t blue);

        /// Calculate 8-bit Red-Green-Blue components from 16-bit color
        /// @param    rgb 16-bit color
        /// @param    red red component, 0x00..0xff
        /// @param    green green component, 0x00..0xff
        /// @param    blue blue component, 0x00..0xff
        void splitColor(uint16_t rgb, uint8_t &red, uint8_t &green, uint8_t &blue);

        void drawBitmapImage(const uint16_t * image);
        void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    private:
        
        void _spi_init();
        void _spiWrite(void * data, size_t size);
        void _spiWrite(uint8_t v);
        void _spiWriteCommand(uint8_t c);
        void _spiWriteData(uint8_t d);

        void _swap(uint16_t &a, uint16_t &b);
        void _setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
        void _setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, autoIncMode_t mode);
        void _resetWindow();
        void _drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, 
                            uint16_t color, uint16_t bg, bool transparent, bool progmem, bool Xbit );
        void _orientCoordinates(uint16_t &x1, uint16_t &y1);
        void _writeRegister(uint16_t reg, uint16_t data);
        void _writeData(uint8_t HI, uint8_t LO);
        void _writeData16(uint16_t HILO);
        void _writeCommand(uint8_t HI, uint8_t LO);
        void _writeCommand16(uint16_t HILO);

        uint16_t _maxX, _maxY;
        int8_t  _rst, _rs, _cs, _sdi, _clk, _led;

        uint8_t  _orientation;

        // correspondig modes if orientation changed:
        const autoIncMode_t modeTab [3][8] = {
        //          { R2L_BottomUp, BottomUp_R2L, L2R_BottomUp, BottomUp_L2R, R2L_TopDown,  TopDown_R2L,  L2R_TopDown,  TopDown_L2R }//
        /* 90° */   { BottomUp_L2R, L2R_BottomUp, TopDown_L2R,  L2R_TopDown,  BottomUp_R2L, R2L_BottomUp, TopDown_R2L,  R2L_TopDown },   
        /*180° */   { L2R_TopDown , TopDown_L2R,  R2L_TopDown,  TopDown_R2L,  L2R_BottomUp, BottomUp_L2R, R2L_BottomUp, BottomUp_R2L}, 
        /*270° */   { TopDown_R2L , R2L_TopDown,  BottomUp_R2L, R2L_BottomUp, TopDown_L2R,  L2R_TopDown,  BottomUp_L2R, L2R_BottomUp}
        };
 

    protected:
        spi_device_handle_t _spi;
        void startWrite(void);
        void endWrite(void);
};

#endif
