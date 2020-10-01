#include "src/IIRFilter.h"
#include "src/FIRFilter.h"
#include "filterCoef.h"
IIRFilter notch60(b_n60, a_n60);
IIRFilter lp(b_lp, a_lp);
IIRFilter hp(b_hp, a_hp);

#define MVPOINTS 6

//#include <Filters.h>
//#include <Filters/Notch.hpp>
//#include <Filters/Butterworth.hpp>
//// Sampling frequency
//const double fs = 512; // Hz
//// Notch frequency (-∞ dB)
//const double fcNotch = 60; // Hz
//// Normalized notch frequency
//const double fnNotch = 2 * fcNotch / fs;
//// Very simple Finite Impulse Response notch filter
//auto notch60N = simpleNotchFIR(fnNotch);     // fundamental
//auto notch60Harmonic = simpleNotchFIR(2 * fnNotch); // second harmonic
//// Cut-off frequency (-3 dB)
//const double fcLp = 25; // Hz
//// Normalized cut-off frequency
//const double fnLp = 2 * fcLp / fs;
//// Sixth-order Butterworth filter
//auto LpFilter = butter<6>(fnLp);


byte myReadBuffer[19] = {};
int16_t parsedADCData = 1;

byte dataType = 0;
byte HR = 0;
bool isDataValid = false;
byte leadOff = true;
byte newVal = 0;

#include "FastMap.h"
FastMap mapper;

//----OLED(SPI)-----
#include <SPI.h>
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 5, /* dc=*/ 22, /* reset=*/ 21);
//VSPI: MISO(Ignore for OLED) = 19, MOSI(D1) = 23, SCLK(D0) = 18

double ecg = 1.0;
double ecgMVTemp[3] = {1.0, 1.0, 1.0};
double mappedVal = 0.0;
double lastMinValDouble = 1.0;
double lastMaxValDouble = 1.0;
int scalYCounter = 0;
uint8_t ecgDrawLast = 1;


void setup() {
  pinMode(12, OUTPUT);
  Serial.begin(115200);
  Serial2.begin(57600); // ESP32_TX2/RX2 = G17/G16
  digitalWrite(12, LOW);
  delay(100);
  digitalWrite(12, HIGH);
  u8g2.begin();
}
uint32_t drawInterval = 0;
void loop() {
  if (Serial2.available() > 0) {
    Serial2.readBytes(myReadBuffer, 3);
    if (( myReadBuffer[0] == 0xAA ) && (myReadBuffer[1] == 0xAA)) { //find data header
      byte pLength = myReadBuffer[2];
      if (pLength == 0x04) dataType = 0; //dataType is adc val.
      if (pLength == 0x12) dataType = 1; //dataType is quality & HR.
    }
    for (byte i = 0; i < 19 ; i++) myReadBuffer[i] = 0; //clear my serial buffer.
    (dataType == 0) ? Serial2.readBytes(myReadBuffer, 5) : Serial2.readBytes(myReadBuffer, 19);
    int sum = 0;
    for (byte i = 0 ; i < 18 ; i++)sum += myReadBuffer[i];
    byte checksum_calc = ~(sum & 0xff); // inverse of lowest 8 bit of data_sum.
    (checksum_calc == myReadBuffer[18]) ? isDataValid = true : isDataValid = false;
  }
  //Parse Data
  if ((isDataValid) && (dataType == 0)) {
    // data ECG ADC Val
    parsedADCData = (myReadBuffer[2] << 8) + myReadBuffer[3];
    if (parsedADCData >= 32768)parsedADCData -= 65536;
    //    Serial.print(parsedADCData);
    //    Serial.write('\t');
    ecg = (notch60.filter(lp.filter(hp.filter(parsedADCData))));
    //        Serial.println(ecg);

    ecgMVTemp[newVal % MVPOINTS] = ecg;
    double sumMV = 0.0;
    for (byte i = 0 ; i < MVPOINTS ; i++) sumMV += ecgMVTemp[i];
    double ecgMV = sumMV / MVPOINTS;
    mapper.init(lastMinValDouble, lastMaxValDouble, 1, 44);
    mappedVal = mapper.constrainedMap(ecgMV);
    if (ecgMV > lastMaxValDouble) lastMaxValDouble = ecgMV;
    if (ecgMV < lastMinValDouble) lastMinValDouble = ecgMV;
    newVal++;
  } else if ((isDataValid) && (dataType == 1)) {
    //data is quality & HR.
    (myReadBuffer[1] == 200) ? leadOff = false : leadOff = true;
    if (!leadOff) {
      HR = myReadBuffer[3];
      //      Serial.println(HR);
    }
  }
  //  Serial.println(notch60.filter((parsedADCData)));

  isDataValid = false;
  for (byte i = 0; i < 19 ; i++) myReadBuffer[i] = 0; //clear my serial buffer.

  //OLED Draw
  if (newVal > 6) {
    byte ecgDraw = round(mappedVal);
    Serial.println(ecgDraw);
    u8g2.setDrawColor(1);
    u8g2.drawLine(scalYCounter, 63 - ecgDrawLast + 1, scalYCounter + 1, 63 - ecgDraw + 1);
    ecgDrawLast = ecgDraw;
    u8g2.setDrawColor(0);// Black
    u8g2.drawFrame(0, 20, 2, 44);  //x, y, Width, Height
    u8g2.drawFrame(scalYCounter + 2, 20, 2, 44);
    u8g2.sendBuffer();
    if (scalYCounter < 128) {
      scalYCounter++;
    } else {
      scalYCounter = 0;
      lastMaxValDouble *= 0.3;
      lastMinValDouble *= 0.3;
    }
    newVal = 0;
  }
}
