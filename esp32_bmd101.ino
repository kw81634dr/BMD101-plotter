#include "src/IIRFilter.h"
#include "src/FIRFilter.h"
#define ECG_sampleRate 512  //Hz

// 0.5 Hz butterworth high-pass, 2 order
const double a_hp[] = {1, -1.99132254835917, 0.991360035149071};
const double b_hp[] = {0.995670645877060, -1.99134129175412, 0.995670645877060};

// 50 Hz Butter low-pass, 3 order
const double a_lp[] = {1, -1.78847140715168, 1.21164264222362,  -0.286869532448224};
const double b_lp[] = {0.0170377128279652, 0.0511131384838956,  0.0511131384838956,  0.0170377128279652};

////Notch50, 48~52,3 order
const double a_n50[] = {1,  -4.82670443192572, 10.6685616767275,  -13.5064582140685, 10.3250327496380,  -4.52086729602225, 0.906481521898628};
const double b_n50[] = {0.952093231830352,  -4.67190889390532, 10.4979447423026,  -13.5102121542077, 10.4979447423026,  -4.67190889390532, 0.952093231830352};
//
////Notch60, 58~62,1 order
const double a_n60[] = {1,  -4.37428878700082, 9.28081903459846,  -11.5658089363129, 8.98198159089241,  -4.09711831321292, 0.906481521898628};
const double b_n60[] = {0.952093231830301,  -4.23400251180179, 9.13254784186472,  -11.5692110129236, 9.13254784186472,  -4.23400251180179, 0.952093231830301};

IIRFilter notch60(b_n60, a_n60);
IIRFilter lp(b_lp, a_lp);
IIRFilter hp(b_hp, a_hp);

//
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
int16_t parsedADCData = 0;
byte dataType = 0;
byte HR = 0;
bool isDataValid = false;
byte leadOff = true;

void setup() {
  pinMode(18, OUTPUT);
  Serial.begin(115200);
  Serial2.begin(57600); // ESP32_TX2/RX2 = G17/G16
  digitalWrite(18, LOW);
  delay(100);
  digitalWrite(18, HIGH);
}

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
    Serial.print(parsedADCData);
    Serial.write('\t');
    Serial.println(notch60.filter(lp.filter(parsedADCData)));
    //    Serial.println(LpFilter(notch60N(parsedADCData)));
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
}


