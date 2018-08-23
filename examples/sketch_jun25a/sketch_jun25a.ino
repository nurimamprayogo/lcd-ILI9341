#include <NIP_ILI9341.h>

NIP_ILI9341 lcd;

//#include <font/Org_01.h>
char nilai[3];

int8_t grafik[50*((96/4)+1)];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
lcd.begin();
lcd.rotate(1);

//lcd.setFont(&Org_01);
lcd.fillScreen(TFT_BLACK);
lcd.setTextSize(1);
lcd.setTextColor(TFT_WHITE);

lcd.fillRect(0,0,320,4,TFT_RED);
lcd.fillRect(0,4,4,236,TFT_RED);
lcd.fillRect(316,4,4,236,TFT_RED);
lcd.fillRect(4,236,312,4,TFT_RED);
lcd.fillRect(4,165,316,4,TFT_RED);
lcd.fillRect(158,165,4,76,TFT_RED);

lcd.writeLine(210,25,210,75,TFT_LIGHTGREY);
lcd.writeLine(210,75,300,75,TFT_LIGHTGREY);
lcd.setCursor(208,15);
lcd.print("t");
lcd.setCursor(304,73);
lcd.print("s");
for(int x=1;x<26;x++){
  //if(x%10==0)lcd.drawHLine(206,(77-((x+1)*2)),4,TFT_RED);
  if(x%5==0)lcd.drawHLine(207,(77-((x+1)*2)),3,TFT_YELLOW);
  else lcd.drawHLine(209,(77-((x+1)*2)),1,TFT_WHITE);
}
for(int x=1;x<19;x++){
  if(x%5==0)lcd.drawVLine((205+((x+1)*5)),76,4,TFT_YELLOW);
  else lcd.drawVLine((205+((x+1)*5)),76,2,TFT_WHITE);
}

lcd.fillRect(225,16,5,5,TFT_RED);
lcd.setCursor(231,15);
lcd.print("IRON");

lcd.fillRect(270,16,5,5,TFT_BLUE);
lcd.setCursor(276,15);
lcd.print("AIR");

lcd.writeLine(100,59,199,59,TFT_WHITE);
lcd.drawRect(99,10,101,99,TFT_WHITE);
lcd.drawRect(10,10,90,149,TFT_WHITE);

lcd.drawRect(10,10,90,15,TFT_WHITE);
lcd.fillRect(11,11,88,13,TFT_BLUE);
lcd.printCenter(10,14,90,"IRON TEMP");

lcd.drawRect(10,59,90,15,TFT_WHITE);
lcd.fillRect(11,60,88,13,TFT_BLUE);
lcd.printCenter(10,63,90,"AIR TEMP");

lcd.drawRect(10,108,90,15,TFT_WHITE);
lcd.fillRect(11,109,88,13,TFT_BLUE);
lcd.printCenter(10,112,90,"FAN SPEED");

lcd.drawRect(10,175,142,15,TFT_WHITE);
lcd.fillRect(11,176,140,13,TFT_BLUE);
lcd.printCenter(10,179,142,"VOLTAGE   CURRENT");

lcd.drawRect(168,175,142,15,TFT_WHITE);
lcd.fillRect(169,176,140,13,TFT_BLUE);
lcd.printCenter(168,179,142,"VOLTAGE   CURRENT");

lcd.setTextSize(3);
sprintf(nilai,"%3d",analogRead(A5)/4);
lcd.printCenter(10,31,90,nilai);

sprintf(nilai,"%3d",analogRead(A5)/4);
lcd.printCenter(10,80,90,nilai);

sprintf(nilai,"%3d",analogRead(A5)/4);
lcd.printCenter(10,129,90,nilai);

lcd.setTextSize(5);
sprintf(nilai,"%3d",analogRead(A5)/4);
lcd.printCenter(100,17,100,nilai);

sprintf(nilai,"%3d",analogRead(A5)/4);
lcd.printCenter(100,66,100,nilai);

//lcd.fillRect2(211, 25, 64, 64, TFT_RED, TFT_GREEN,marilyn_64x64);
lcd.setWindows(192, 25, 320, 89);
Serial.println("dasdadsadsadsadasda");

//unsigned int _data[16]={0x98E5, 0x98E5, 0x98E5, 0x98E5, 0x98E5, 0x98E4, 0x90E4, 0x9905, 0x9905, 0x9104, 0x90E4, 0x9905, 0x9925, 0x9925, 0x9925, 0x9945};
//_data[0]=0x00;_data[1]=0x19;_data[2]=0x1B;_data[3]=0x04;_data[4]=0x10;_data[5]=0x07;_data[6]=0x2A;_data[7]=0x47;
//  _data[8]=0x39;_data[9]=0x03;_data[10]=0x06;_data[11]=0x06;_data[12]=0x30;_data[13]=0x38;_data[14]=0x0F;
for(int x=0;x< 100*((90/8)+1);x++)grafik[x]=0x00;

//writeLineRAM(50,0,60,10);
//lcd.setWindows(211, 25, 306, 125);
//lcd.drawBitmap(200,25,grafik,90,100,TFT_BLUE);
//lcd.writeCmdPara16(0x2C, 1199, grafik);
//lcd.writeLine(211,76,221,86,TFT_RED)

lcd.setTextColor(TFT_WHITE,TFT_BLACK);

}

int8_t iron_lvl[91][2];

void loop() {
  // put your main code here, to run repeatedly:
  
iron_lvl[90][1]=50-map(analogRead(A5),0,1023,0,50);
iron_lvl[90][0]=map(analogRead(A5),0,1023,0,50);

long milis=millis();

//lcd.setTextSize(5);
//sprintf(nilai,"%3d",iron_lvl[89][0]);
//lcd.printCenter(100,17,100,nilai);

//sprintf(nilai,"%3d",iron_lvl[89][1]);
//lcd.printCenter(100,66,100,nilai);


for(int x=0;x< 50*((96/4)+1);x++)grafik[x]=0xFF;
  for(int y=1;y<90;y++){
    int x=y;
    lcd.drawPixel(210+x,74-(iron_lvl[x][0]),TFT_BLACK);
    lcd.drawPixel(210+x,74-(iron_lvl[x][1]),TFT_BLACK);
    iron_lvl[x][0]=iron_lvl[x+1][0];
    iron_lvl[x][1]=iron_lvl[x+1][1];
    lcd.drawPixel(210+x,74-(iron_lvl[x][0]),TFT_BLUE);
    lcd.drawPixel(210+x,74-(iron_lvl[x][1]),TFT_RED);


    //lcd.writeLine(210+x,74-(iron_lvl[x][0]),211+x,74-(iron_lvl[x+1][0]),TFT_BLACK);
    //lcd.writeLine(210+x,74-(iron_lvl[x][1]),211+x,74-(iron_lvl[x+1][1]),TFT_BLACK);

    //if(x>1){
    //lcd.writeLine(209+x,74-(iron_lvl[x][0]),210+x,74-(iron_lvl[x+1][0]),TFT_RED);
    //lcd.writeLine(209+x,74-(iron_lvl[x][1]),210+x,74-(iron_lvl[x+1][1]),TFT_BLUE);
    //}
    //iron_lvl[x][0]=iron_lvl[x+1][0];
    //iron_lvl[x][1]=iron_lvl[x+1][1];

    //writeLineRAM(x,50-(iron_lvl[x][0]),1+x,50-(iron_lvl[x+1][0]),0);
    //writeLineRAM(x,50-(iron_lvl[x][1]),1+x,50-(iron_lvl[x+1][1]),0);
    //if(x>1){
    
    //writeLineRAM(x-1,50-(iron_lvl[x][0]),x,50-(iron_lvl[x+1][0]),1);
    //writeLineRAM(x-1,50-(iron_lvl[x][1]),x,50-(iron_lvl[x+1][1]),2);
    
    //}
    //iron_lvl[x][0]=iron_lvl[x+1][0];
    //iron_lvl[x][1]=iron_lvl[x+1][1];
  } 

  //for(int x=0;x< 50*((96/4)+1);x++)grafik[x]=0b11011000;
  //lcd.setWindows(211, 25, 306, 74);
  //lcd.writeCmdPara16(0x2C, 1200, grafik,TFT_RED, TFT_BLUE, TFT_YELLOW);
  

long fps=1000/(millis()-milis);
lcd.setTextSize(1);
lcd.setCursor(200,145);
lcd.print("fps= ");
lcd.print(fps);
lcd.print(" ");

 //delay(1000);
  //lcd.setWindows(211, 25, 275, 89);
  //lcd.pushColors((const uint8_t*)marilyn_64x64, 64 * 64, 1, false);
  //delay(500);
  
}


