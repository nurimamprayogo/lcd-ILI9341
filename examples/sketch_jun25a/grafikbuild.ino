#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
void writeLineRAM(int16_t x0, int16_t y0, int16_t x1, int16_t y1,int8_t color) {
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
    //for(int x=0;x<100;x++){
    //  grafik[x*(96/8)+(89/8)]=(grafik[x*(96/8)+(89/8)]|(1<<19-((96/8)+(89%8))))&(~((1^0)<<(19-((96/8)+(89%8)))));
    //  grafik[x*(96/8)+(90/8)]=(grafik[x*(96/8)+(90/8)]|(1<<19-((96/8)+(90%8))))&(~((1^0)<<(19-((96/8)+(90%8)))));
    //}
    for (; x0<=x1; x0++) {
        if (steep) {
          grafik[x0*(96/4)+(y0/4)]=(grafik[x0*(96/4)+(y0/4)]/*|(3<<(27-((96/4)+(y0%4)))*2)*/)&(~((3^color)<<((27-((96/4)+(y0%4)))*2)));//updown
          // Serial.println(((27-((96/4)+(y0%4)))*2),BIN);
            //drawPixel(y0, x0, color);
        } else {
          grafik[y0*(96/4)+(x0/4)]=(grafik[y0*(96/4)+(x0/4)]/*|(3<<(27-((96/4)+(x0%4)))*2)*/)&(~((3^color)<<((27-((96/4)+(x0%4)))*2)));//datar
            //drawPixel(x0, y0, color);
           //  Serial.println(((27-((96/4)+(x0%4)))*2),BIN);
        }
      // delay(100);
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
     
    }
}


