#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif
#include <stdlib.h>
#include <avr/pgmspace.h>

#include "NIP_ILI9341.h"

#include "font/normal.cpp"

char pin_CS,pin_RST,pin_RS,pin_WR,pin_RD,pin_BL,pin_D0,pin_D1,pin_D2,pin_D3,pin_D4,pin_D5,pin_D6,pin_D7;
static uint8_t xUpdateMin, xUpdateMax, yUpdateMin, yUpdateMax;
unsigned char pin_Data[8];
unsigned char pin_Control[5];
int LCD_W,LCD_H;

//_lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;

#define wrt(port,pin,logic)  _SFR_IO8(port)=(_SFR_IO8(port)|(1<<pin))&(~((logic^1)<<pin))

void OUT(unsigned char pin,bool logic)  {
  unsigned char port;
  if(pin<8)       {port=0x0B;pin=pin;}
  else if(pin<14) {port=0x05;pin=pin-8;}
  else            {port=0x08;pin=pin-14;}
  
  _SFR_IO8(port)=(_SFR_IO8(port)|(1<<pin))&(~((logic^1)<<pin));
}

#define HIGH 		1
#define LOW			0
#define CS(log) 	digitalWrite(pin_CS,log)
#define WR_STROBE 	PORTC &= ~(1<<1);PORTC |= (1<<1);
#define BMASK         0x03              //more intuitive style for mixed Ports
#define DMASK         0xFC              //does exactly the same as previous
#define write_8(x)    { PORTB = (PORTB & ~BMASK) | ((x) & BMASK); PORTD = (PORTD & ~DMASK) | ((x) & DMASK); }
#define write16(x)  write8((x)>>8); write8(x) 
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

#ifndef pgm_read_byte
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
 #define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
 #define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#endif

// Pointers are a peculiar case...typically 16-bit on AVR boards,
// 32 bits elsewhere.  Try to accommodate both...

#if !defined(__INT_MAX__) || (__INT_MAX__ > 0xFFFF)
 #define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))
#else
 #define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))
#endif

NIP_ILI9341::NIP_ILI9341(char CS, char RST, char RS, char WR , char RD, char BL, char D0, char D1, char D2,char D3, char D4, char D5, char D6, char D7){
	pin_CS=CS;
	pin_RST=RST;
	pin_RS=RS;
	pin_WR=WR;
	pin_RD=RD;
	pin_BL=BL;
	
	pin_Control[0]=pin_CS;
	pin_Control[1]=pin_RS;
	pin_Control[2]=pin_WR;
	pin_Control[3]=pin_RD;
	pin_Control[4]=pin_RST;
	
	pin_D0=D0;
	pin_D1=D1;
	pin_D2=D2;
	pin_D3=D3;
	pin_D4=D4;
	pin_D5=D5;
	pin_D6=D6;
	pin_D7=D7;
	
	pin_Data[0]=pin_D0;
	pin_Data[1]=pin_D1;
	pin_Data[2]=pin_D2;
	pin_Data[3]=pin_D3;
	pin_Data[4]=pin_D4;
	pin_Data[5]=pin_D5;
	pin_Data[6]=pin_D6;
	pin_Data[7]=pin_D7;
}

void NIP_ILI9341::begin(int w,int h)
{
	_width=w;_height=h;
	LCD_W=w;LCD_H=h;
    reset();
	
	writeCmdPara(0x01,0,0);
	delay(150);
	writeCmdPara(0x28,0,0);
	writeCmdPara(0x3A,0x55);
	
	unsigned char _data[15];
	_data[0]=0x01;_data[1]=0x01;_data[2]=0x00;
	writeCmdPara(0xF6,3,_data);
	_data[0]=0x00;_data[1]=0x81;_data[2]=0x30;
	writeCmdPara(0xCF,3,_data);
	_data[0]=0x64;_data[1]=0x03;_data[2]=0x12;_data[3]=0x81;
	writeCmdPara(0xED,4,_data);
	_data[0]=0x85;_data[1]=0x10;_data[2]=0x78;
	writeCmdPara(0xE8,3,_data);
	_data[0]=0x39;_data[1]=0x2C;_data[2]=0x00;_data[3]=0x34;_data[4]=0x02;
	writeCmdPara(0xCB,5,_data);
	writeCmdPara(0xF7,0x20);
	_data[0]=0x00;_data[1]=0x00;
	writeCmdPara(0xEA,2,_data);
	writeCmdPara(0xB0,0x00);
	_data[0]=0x00;_data[1]=0x1B;
	writeCmdPara(0xB0,2,_data);
	
	writeCmdPara(0xB4,0x00);
	writeCmdPara(0xC0,0x21);
	writeCmdPara(0xC1,0x11);
	_data[0]=0x3F;_data[1]=0x3C;
	writeCmdPara(0xC5,2,_data);
	writeCmdPara(0xC7,0xB5);
	writeCmdPara(0x36,0x48);
	writeCmdPara(0xF2,0x00);
	writeCmdPara(0x26,0x01);
	_data[0]=0x0F;_data[1]=0x26;_data[2]=0x24;_data[3]=0x0B;_data[4]=0x0E;_data[5]=0x09;_data[6]=0x54;_data[7]=0xA8;
	_data[8]=0x46;_data[9]=0x0C;_data[10]=0x17;_data[11]=0x09;_data[12]=0x0F;_data[13]=0x07;_data[14]=0x00;
	writeCmdPara(0xE0,14,_data);
	_data[0]=0x00;_data[1]=0x19;_data[2]=0x1B;_data[3]=0x04;_data[4]=0x10;_data[5]=0x07;_data[6]=0x2A;_data[7]=0x47;
	_data[8]=0x39;_data[9]=0x03;_data[10]=0x06;_data[11]=0x06;_data[12]=0x30;_data[13]=0x38;_data[14]=0x0F;
	writeCmdPara(0xE1,14,_data);
	//writeCmdPara(0X13,0,0);
	
	writeCmdPara(0x11,0,0);
	delay(150);
	rotate(0);
	fillScreen(BLACK);
	writeCmdPara(0x29,0,0);
	
	//rotate(4);
	setInverse(0);
}

void NIP_ILI9341::reset(void)
{
	for(int x=0;x<5;x++){pinMode(pin_Control[x],OUTPUT);digitalWrite(pin_Control[x],HIGH);} 
	for(int x=0;x<8;x++){pinMode(pin_Data[x],OUTPUT);digitalWrite(pin_Data[x],LOW);} 
	analogWrite(pin_BL,200);
	delay(100);
	digitalWrite(pin_RST, LOW);
    delay(100);
    digitalWrite(pin_RST, HIGH);
    delay(100);
	
	writeCmdData(0xB0, 0x0000);   //R61520 needs this to read ID
}

void NIP_ILI9341::rotate(uint8_t r)
{
    uint8_t val;
	//val=r & 7;
    _width = (r & 1) ? LCD_H : LCD_W;
    _height = (r & 1) ? LCD_W : LCD_H;
    switch (r) {
    case 0:                    //PORTRAIT:mirror
        val = 0x48;             //MY=0, MX=1, MV=0, ML=0, BGR=1
        break;
    case 1:                    //LANDSCAPE: 90 degrees mirror
        val = 0x28;             //MY=0, MX=0, MV=1, ML=0, BGR=1
        break;
    case 2:                    //PORTRAIT_REV: 180 degrees mirror
        val = 0x98;             //MY=1, MX=0, MV=0, ML=1, BGR=1
        break;
    case 3:                    //LANDSCAPE_REV: 270 degrees mirror
        val = 0xF8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
        break;	
	case 4:                    //PORTRAIT: 08, 68, b8, c8
        val = 0x08;             //MY=0, MX=0, MV=0, ML=0, BGR=1
        break;
    case 5:                    //LANDSCAPE: 90 degrees 
        val = 0xB8;             //MY=1, MX=0, MV=1, ML=1, BGR=1
        break;
    case 6:                    //PORTRAIT_REV: 180 degrees 
        val = 0xD8;             //MY=1, MX=1, MV=0, ML=1, BGR=1
        break;
    case 7:                    //LANDSCAPE_REV: 270 degrees 
        val = 0x68;             //MY=0, MX=1, MV=1, ML=0, BGR=1
        break;
	}
	out0=val;
	writeCmdPara(0x36, val); 
	setWindows(0, 0, _width - 1, _height - 1);
    vertScroll(0, LCD_H, 0);
}

void NIP_ILI9341::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa = LCD_H - top - scrollines;  // bottom fixed area
    int16_t vsp;
    int16_t sea = top;
    if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
	vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    sea = top + scrollines - 1;
    
	uint8_t d[6];
	
	d[0] = top >> 8;        //TFA
    d[1] = top;
    d[2] = scrollines >> 8; //VSA
    d[3] = scrollines;
    d[4] = bfa >> 8;        //BFA
    d[5] = bfa;
    writeCmdPara(0x33, 6, d);
    d[0] = vsp >> 8;        //VSP
    d[1] = vsp;
    writeCmdPara(0x37, 2, d);
	if (offset == 0) {
		writeCmdPara(0x13, 0, NULL);    //NORMAL i.e. disable scroll
	return;
    }
        // 0x6809, 0x9320, 0x9325, 0x9335, 0xB505 can only scroll whole screen
        //writeCmdData(0x61, (1 << 1) | _lcd_rev);        //!NDL, VLE, REV
        //writeCmdData(0x6A, vsp);        //VL#
}



void write8(uint16_t data)
{
    //digitalWrite(pin_D0, data & 1);
    //digitalWrite(pin_D1, (data & 2) >> 1);
    //digitalWrite(pin_D2, (data & 4) >> 2);
    //digitalWrite(pin_D3, (data & 8) >> 3);
    //digitalWrite(pin_D4, (data & 16) >> 4);
    //digitalWrite(pin_D5, (data & 32) >> 5);
    //digitalWrite(pin_D6, (data & 64) >> 6);
    //digitalWrite(pin_D7, (data & 128) >> 7);
	write_8(data);
	WR_STROBE;
}


void send(unsigned char type, uint16_t data)
{
    digitalWrite(pin_RS, type);
    //digitalWrite(pin_RD, HIGH);
    //digitalWrite(pin_WR, HIGH);
    write8(data >> 8);
    write8(data);
}

void NIP_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	setWindows(x, y, x+w-1, y+h-1);
	uint8_t hi = color >> 8, lo = color & 0xFF;
	int end;
	CS(LOW);
    send(_CMD,0x2C);
	digitalWrite(pin_RS, _DATA);
    while (h-- > 0) {
        end = w;
        do {
            write8(hi);
            write8(lo);
        } while (--end != 0);
	}
	CS(HIGH);
    //writeCmdData(0x2C, color);
}

void NIP_ILI9341::fillRect2(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t color1,const uint16_t *bitmap)
{
	setWindows(x, y, x+w-1, y+h-1);
	uint8_t hi,lo,hi1,lo1;
	int end;
	CS(LOW);
    send(_CMD,0x2C);
	digitalWrite(pin_RS, _DATA);
	uint16_t data=0;
    while (h-- > 0) {
        end = w;
        do {
			//if((bitmap[w*(h/8)]>>h%(h/8)) & 1 ==1 ){
			//	write8(hi);
			//	write8(lo);
			//}
			//else {
				//pgm_read_byte(&normal[c - ' '][i]);
				data=(*bitmap++);
			hi = data>>8 ; lo = data&0xFF;
			hi1 = color1 >> 8; lo1 = color1 & 0xFF;	
				write8(hi);
				write8(lo);
			//}
			Serial.println(data,HEX);
			
			
        } while (--end != 0);
	}
	CS(HIGH);
    //writeCmdData(0x2C, color);
}

void NIP_ILI9341::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // MCUFRIEND just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= _width || y >= _height)
        return;
    setWindows(x, y, x, y);
    writeCmdData(0x2C, color);
}

void NIP_ILI9341::setWindows(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
	unsigned char _data[4];
	_data[0]=x>>8;_data[1]=x & 0xFF;_data[2]=x1>>8;_data[3]=x1 & 0xFF;
	writeCmdPara(0x2A,4,_data);
	
	_data[0]=y>>8;_data[1]=y & 0xFF;_data[2]=y1>>8;_data[3]=y1 & 0xFF;
	writeCmdPara(0x2B,4,_data);
}

void NIP_ILI9341::pushColors_any(uint8_t * block, int16_t n, bool first, uint8_t flags)
{
    uint16_t color;
    uint8_t h, l;
	bool isconst = flags & 1;
	bool isbigend = (flags & 2) != 0;
    CS(LOW);
    if (first) {
        send(_CMD,0x2C);
    }
    while (n-- > 0) {
        if (isconst) {
            h = pgm_read_byte(block++);
            l = pgm_read_byte(block++);
        } else {
		    h = (*block++);
            l = (*block++);
		}
        color = (isbigend) ? (h << 8 | l) :  (l << 8 | h);
        write16(color);
    }
    CS(HIGH);
}

void NIP_ILI9341::setInverse(bool invert)
{
	//CS(LOW);
    //send(_CMD,0x20 + invert);
    //CS(HIGH);
    //WriteCmdData(0x61, _lcd_rev);
	writeCmdPara(0x20 + invert,0,0);
}

void NIP_ILI9341::writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            drawPixel(y0, x0, color);
        } else {
            drawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

void NIP_ILI9341::writeCmdPara16(uint16_t cmd, int16_t leng, uint8_t * data, uint16_t color,uint16_t color1,uint16_t color2)
{
    CS(LOW);
    send(_CMD,cmd);
	digitalWrite(pin_RS, _DATA);
    while (leng-- > 0) {
        uint16_t _data8 = *data++;
		for(char x=3;x>=0;x--){
			//Serial.println(((_data8>>x)&1)*0xFF,HEX);
			uint8_t dat=(_data8>>(x*2))&3;
			if(dat==1)
				{write16(color);}
			else if(dat==2)
				{write16(color1);}
			else if(dat==3)
				{write16(0);}
			else 
				{write16(color2);}
		}
    }
    CS(HIGH);
}

void NIP_ILI9341::writeCmdPara(uint16_t cmd, int8_t leng, uint8_t * data)
{
    CS(LOW);
    send(_CMD,cmd);
	digitalWrite(pin_RS, _DATA);
    while (leng-- > 0) {
        uint8_t _data8 = *data++;
        write8(_data8);
    }
    CS(HIGH);
}

void NIP_ILI9341::writeCmdPara(uint16_t cmd, uint8_t data)
{
    CS(LOW);
    send(_CMD,cmd);
	digitalWrite(pin_RS, _DATA);
    write8(data);
    CS(HIGH);
}

void NIP_ILI9341::writeCmdData(uint16_t cmd, uint16_t dat)
{
    CS(LOW);
    send(_CMD,cmd);
    send(_DATA,dat);
    CS(HIGH);
}

void NIP_ILI9341::setTextSize(uint8_t s)
{
	if(s>0)textsize=s;
}

void NIP_ILI9341::drawBitmap(int16_t x, int16_t y,const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color) {

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;
		
	CS(LOW);
    send(_CMD,0x2C);
	digitalWrite(pin_RS, _DATA);
	for(int16_t j=0; j<h; j++, y++) {
        for(int16_t i=0; i<w; i++) {
            if(i & 7) byte <<= 1;
            else      byte   = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
            if(byte & 0x80) 
			{write16(color);}
			else 
			{write16(0);}
		Serial.println(byte ,HEX);
		delay(100);
        }
    }
	
    CS(HIGH);	
		
    //for(int16_t j=0; j<h; j++, y++) {
    //    for(int16_t i=0; i<w; i++) {
    //        if(i & 7) byte <<= 1;
    //        else      byte   = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
    //        if(byte & 0x80) drawPixel(x+i, y, color);
    //    }
    //}
}

void NIP_ILI9341::drawChar(int16_t x, int16_t y, unsigned char c,uint16_t color, uint16_t bg, uint8_t size) 
{
	if(!gfxFont){
        if((x >= _width)            || // Clip right
           (y >= _height)           || // Clip bottom
           ((x + 6 * size - 1) < 0) || // Clip left
           ((y + 8 * size - 1) < 0))   // Clip top
            return;
        if((c >= 176)) c++; // Handle 'classic' charset behavior
        for(int8_t i=0; i<5; i++ ) { // Char bitmap = 5 columns
            uint8_t line = pgm_read_byte(&normal[c - ' '][i]);
            for(int8_t j=0; j<8; j++, line >>= 1) {
				if(action==1){
					if(line & 1) {
						if(size == 1)
							drawPixel(x+i, y+j, color);
						else
							fillRect(x+i*size, y+j*size, size, size, color);
					} 
					else if(bg != color) {
						if(size == 1)
							drawPixel(x+i, y+j, bg);
						else
							fillRect(x+i*size, y+j*size, size, size, bg);
					}
				}
            }
        }
        //if(bg != color) { // If opaque, draw vertical line for last column
        //    if(size == 1) drawVLine(x+5, y, 8, bg);
        //    else          fillRect(x+5*size, y, size, 8*size, bg);
        //}
	}
	else {
        c -= (uint8_t)pgm_read_byte(&gfxFont->first);
        GFXglyph *glyph  = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
        uint8_t  *bitmap = (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);

        uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
        uint8_t  w  = pgm_read_byte(&glyph->width),
                 h  = pgm_read_byte(&glyph->height);
        int8_t   xo = pgm_read_byte(&glyph->xOffset),
                 yo = pgm_read_byte(&glyph->yOffset);
        uint8_t  xx, yy, bits = 0, bit = 0;
        int16_t  xo16 = 0, yo16 = 0;

        if(size > 1) {
            xo16 = xo;
            yo16 = yo;
        }
        for(yy=0; yy<h; yy++) {
            for(xx=0; xx<w; xx++) {
                if(!(bit++ & 7)) {
                    bits = pgm_read_byte(&bitmap[bo++]);
                }
				if(action==1){
					if(bits & 0x80) {
						if(size == 1) {
							drawPixel(x+xo+xx, y+yo+yy, color);
						} else {
							fillRect(x+(xo16+xx)*size, y+(yo16+yy)*size, size, size, color);
						}
					}
					else if(bg != color) {
						if(size == 1)
							drawPixel(x+xo+xx, y+yo+yy, bg);
						else
							fillRect(x+(xo16+xx)*size, y+(yo16+yy)*size, size, size, bg);
					}
				}
                bits <<= 1;
            }
        }
    }
}

void NIP_ILI9341::printCenter(int16_t x, int16_t y, int16_t w, const char str[])
{
	cursorx=x;
	cursory=y;
	action=0;
	print(str);
	if((cursorx-x)>w){
		cursorx=x;cursory=y;
		action=1;
		print(str);
	}
	else {
		int16_t sisa=(w-(cursorx-x))/2;
		cursorx=x+sisa;cursory=y;
		action=1;
		print(str);
	}
}

void NIP_ILI9341::printRight(int16_t x, int16_t y, int16_t w, const char str[])
{
	cursorx=x;
	cursory=y;
	action=0;
	print(str);
	if((cursorx-x)>w){
		cursorx=x;cursory=y;
		action=1;
		print(str);
	}
	else {
		int16_t sisa=(w-(cursorx-x));
		cursorx=x+sisa;cursory=y;
		action=1;
		print(str);
	}
}

#if ARDUINO < 100
void NIP_ILI9341::write(uint8_t chr)
#else
size_t NIP_ILI9341::write(uint8_t chr)
#endif
{	
	if(!gfxFont) {
		if(chr == '\n') {                        // Newline?
				cursorx  = 0;                     // Reset x to zero,
				cursory += textsize * 8;          // advance y one line
			} 
		else if(chr != '\r') {                 // Ignore carriage returns
			if(1 && ((cursorx + textsize * 6) > _width)) { // Off right?
				cursorx  = 0;                 // Reset x to zero,
				cursory += textsize * 8;      // advance y one line
			}
			drawChar(cursorx, cursory, chr, textcolor, textbgcolor, textsize);
			cursorx += textsize * 6;
		}
	}
	else { // Custom font
	
        if(chr == '\n') {
            cursorx  = 0;
            cursory += (int16_t)textsize *
                        (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        } else if(chr != '\r') {
            uint8_t first = pgm_read_byte(&gfxFont->first);
            //if((chr >= first) && (chr <= (uint8_t)pgm_read_byte(&gfxFont->last))) {
                GFXglyph *glyph = &(((GFXglyph *) pgm_read_pointer(&gfxFont->glyph))[chr - first]);
                uint8_t   w     = pgm_read_byte(&glyph->width),
                          h     = pgm_read_byte(&glyph->height);
                if((w > 0) && (h > 0)) { // Is there an associated bitmap?
                    int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset); // sic
                    if(1 && ((cursorx + textsize * (xo + w)) > _width)) {
                        cursorx  = 0;
                        cursory += (int16_t)textsize *
                          (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
                    }
                    drawChar(cursorx, cursory, chr, textcolor, textbgcolor, textsize);
                }
                cursorx += (uint8_t)pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize;
            //}
        }

    }
}