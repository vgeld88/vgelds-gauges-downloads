// RoundDash ESP32-WROOM-32 + GC9A01: first-screen verification firmware.
// SPI wiring: MOSI23, CLK18, CS15, DC2, RST4. Screen is 3.3 V.
// This is a DEMO screen test; not yet OBD/Bluetooth telemetry.
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite frame = TFT_eSprite(&tft);
bool ready = false;
uint32_t lastDraw = 0;
const int C = 120;
float gaugeAngle(float p) { return (135.0f + p*270.0f)*0.01745329252f; }
int gx(int r,float a){ return C + lroundf(r*cosf(a)); }
int gy(int r,float a){ return C + lroundf(r*sinf(a)); }
void drawGauge(int rpm){
  frame.fillSprite(TFT_BLACK);
  frame.fillCircle(C,C,118,0x7BEF);
  frame.fillCircle(C,C,114,0x2104);
  frame.fillCircle(C,C,109,TFT_BLACK);
  for(int i=0;i<=135;i++){
    float p=float(i)/135, a=gaugeAngle(p);
    uint16_t color=p>=0.84f?TFT_RED:(p>=0.67f?0xFCA0:0x049F);
    frame.drawLine(gx(103,a),gy(103,a),gx(106,a),gy(106,a),color);
  }
  for(int i=0;i<=40;i++){
    float p=float(i)/40,a=gaugeAngle(p);
    frame.drawLine(gx(i%5==0?84:93,a),gy(i%5==0?84:93,a),gx(100,a),gy(100,a),i>=33?TFT_RED:0xBDF7);
  }
  frame.setTextDatum(MC_DATUM);
  frame.setTextColor(TFT_WHITE,TFT_BLACK);
  frame.setTextFont(2);
  for(int i=0;i<=8;i++){float a=gaugeAngle(i/8.0f);frame.drawString(String(i),gx(73,a),gy(73,a));}
  float a=gaugeAngle(constrain(rpm/8000.0f,0.0f,1.0f));
  frame.drawLine(gx(-10,a),gy(-10,a),gx(80,a),gy(80,a),TFT_RED);
  frame.fillCircle(C,C,6,TFT_LIGHTGREY);
  frame.fillCircle(C,C,3,TFT_RED);
  frame.setTextColor(TFT_WHITE,TFT_BLACK);
  frame.drawString("ROUND DASH",120,82,2);
  frame.fillRoundRect(67,149,106,36,6,0x0861);
  frame.setTextColor(TFT_WHITE,0x0861);
  frame.drawString(String(rpm),120,167,4);
  frame.setTextColor(0xFCA0,TFT_BLACK);
  frame.drawString("RPM DEMO",120,200,2);
  frame.setTextColor(0x8410,TFT_BLACK);
  frame.drawString("SCREEN TEST",120,219,2);
  frame.pushSprite(0,0);
}
void setup(){
 Serial.begin(115200);
 tft.init();
 tft.setRotation(0);
 tft.fillScreen(TFT_BLACK);
 frame.setColorDepth(8);
 ready = frame.createSprite(240,240)!=nullptr;
 if(!ready) tft.drawCentreString("SPRITE ERROR",120,120,2);
}
void loop(){
 if(!ready) return;
 uint32_t now=millis();
 if(now-lastDraw < 50) return;
 lastDraw=now;
 int rpm=(int)(3800+2600*sinf(now*0.0009f));
 drawGauge(rpm);
}
