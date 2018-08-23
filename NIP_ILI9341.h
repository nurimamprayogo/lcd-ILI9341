#ifndef _NIP_ILI9341_H
#define _NIP_ILI9341_H

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif
#include "Print.h"

#define BLACK 1
#define WHITE 0
#define LCDWIDTH 240
#define LCDHEIGHT 320

#define _CMD  0x00
#define _DATA 0x01
#define LCD_CONTRAST 0x15

#define ON 0x01
#define OFF 0x00

#define LCD_SOFT_RESET 0x01
#define LCD_SLEEP 0x10
#define LCD_PARTIAL 0x12
#define LCD_INVERSION 0x20
#define LCD_DISPLAY 0x28
#define LCD_SET_X 0x2A
#define LCD_SET_Y 0x2B
#define LCD_MEMORY_WRITE 0x2C
#define LCD_VERT_SCROLL_DEF 0x33
#define LCD_VERT_SCROLL_START 0x37
#define LCD_IDLE_MODE 0x38
#define LCD_MEMORY_WRITE_COU 0x3C
#define LCD_WRITE_BRIGHTNESS 0x51

#define TFT_BLACK       0x0000      /*   0,   0,   0 */ //0b1111  1 111  11 11  111
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
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFC9F

typedef struct { // Data stored PER GLYPH
	uint16_t bitmapOffset;     // Pointer into GFXfont->bitmap
	uint8_t  width, height;    // Bitmap dimensions in pixels
	uint8_t  xAdvance;         // Distance to advance cursor (x axis)
	int8_t   xOffset, yOffset; // Dist from cursor pos to UL corner
} GFXglyph;

typedef struct { // Data stored for FONT AS A WHOLE:
	uint8_t  *bitmap;      // Glyph bitmaps, concatenated
	GFXglyph *glyph;       // Glyph array
	uint8_t   first, last; // ASCII extents
	uint8_t   yAdvance;    // Newline distance (y axis)
} GFXfont;

class NIP_ILI9341 : public Print {
 public:
	NIP_ILI9341(char CS, char RST, char RS, char WR , char RD, char BL, char D0, char D1, char D2,char D3, char D4, char D5, char D6, char D7);
	NIP_ILI9341(){NIP_ILI9341(A3,A4,A2,A1,A0,10,8,9,2,3,4,5,6,7);}
	void begin(){begin(LCDWIDTH,LCDHEIGHT);}
	void begin(int w, int h);
	void reset();
	void setContrast(unsigned char value);
	void setWindows(int16_t x, int16_t y, int16_t x1, int16_t y1);
	void setCursor(int16_t x, int16_t y){setWindows(x, y, x, y);cursorx=x;cursory=y;}
	void clearDisplay();
	void rotate(uint8_t r); 
	void setInverse(bool invert);
	void setBackLed(unsigned char _bl);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void drawBitmap(int16_t x, int16_t y,const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);   
	void pushColors_any(uint8_t * block, int16_t n, bool first, uint8_t flags);
	
	void printCenter(int16_t x, int16_t y, int16_t w, const char[]);
	void printRight(int16_t x, int16_t y, int16_t w, const char[]);
	
	void setTextSize(uint8_t s);
	void setTextColor(int16_t color){textcolor=textbgcolor=color;}
	void setTextColor(int16_t color, int16_t bg){textcolor=color;textbgcolor=bg;}
	void setFont(const GFXfont *f) {
			if(f) {           
				if(!gfxFont) {
				cursory += 6;
				}
			} 
			else if(gfxFont) {
				cursory -= 6;
				}
			gfxFont = (GFXfont *)f;
		}
	
	void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint16_t color);
	void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
		drawHLine(x, y, w, color);
		drawHLine(x, y+h-1, w, color);
		drawVLine(x, y, h, color);
		drawVLine(x+w-1, y, h, color);
	}
	void fillRect2(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t color1,const uint16_t *bitmap);
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void drawVLine(int16_t x, int16_t y, int16_t h, uint16_t color) { fillRect(x, y, 1, h, color); }
	void drawHLine(int16_t x, int16_t y, int16_t w, uint16_t color) { fillRect(x, y, w, 1, color); }
	void fillScreen(uint16_t color)                                     { fillRect(0, 0, _width, _height, color); }
	void vertScroll(int16_t top, int16_t scrollines, int16_t offset);
	void drawChar(int16_t x, int16_t y, unsigned char c,uint16_t color, uint16_t bg, uint8_t size);
	
	void writeCmdPara16(uint16_t cmd, int16_t leng, uint8_t * data, uint16_t color,uint16_t color1,uint16_t color2);
	void writeCmdData(uint16_t cmd, uint16_t dat);
	void writeCmdPara(uint16_t cmd, int8_t leng, uint8_t * data);
	void writeCmdPara(uint16_t cmd, uint8_t data);
	int out0,out1,out2;
#if ARDUINO < 100
        virtual void write(uint8_t chr);
#else        
        virtual size_t write(uint8_t chr);
#endif		
 protected:
 
 GFXfont
    *gfxFont;
	
 private:
    
	//void send(unsigned char type, unsigned char data);
	void lcd_on();
	unsigned char column,invert=0;
    unsigned char line,textsize=1;
	unsigned int _width,_height,cursorx=0,cursory=0,textcolor=TFT_BLACK,textbgcolor=TFT_BLACK;
	bool action=1;
};

#endif