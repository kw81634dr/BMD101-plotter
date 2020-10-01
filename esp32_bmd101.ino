
byte myReadBuffer[19] = {};
int16_t parsedADCData = 0;
byte dataType = 0;
byte HR = 0;
bool isDataValid = false;
byte leadOff = true;

void setup() {
  Serial.begin(115200);
  Serial2.begin(57600); // ESP32_TX2/RX2 = G17/G16
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
    // Serial.println(parsedADCData);
  } else if ((isDataValid) && (dataType == 1)) {
    //data is quality & HR.
    (myReadBuffer[1] == 200) ? leadOff = false : leadOff = true;
    if (!leadOff) {
      HR = myReadBuffer[3];
      Serial.println(HR);
    }
  }
  isDataValid = false;
  for (byte i = 0; i < 19 ; i++) myReadBuffer[i] = 0; //clear my serial buffer.
}
