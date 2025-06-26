
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ESPNowW.h>
#include "ESP8266WiFi.h"
#include <ADXL345_WE.h>
#include <Adafruit_PN532.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_NeoPixel.h>

#define SCL 5//14
#define SDA 4//2

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ 13
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

#define SENSEI

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);
ADXL345_WE accSens = ADXL345_WE(0x53);
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
Adafruit_BMP280 bmp; // I2C
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, 0, NEO_GRB + NEO_KHZ800);


char Int1Flag = 0;
char Int2Flag = 0;

void IRAM_ATTR ISR1() {
    Int1Flag = 1;
    
    //detachInterrupt(25);
}
void IRAM_ATTR ISR2() {
    Int2Flag = 1;
    //detachInterrupt(25);
}

#ifdef SENSEI
uint8_t MasterAddress[] = {0xBC, 0xFF, 0x4D, 0x85, 0x7A, 0x00};
uint8_t receiverAddress[] = {0xD8, 0xBF, 0xC0, 0x17, 0xA6, 0x9A};
typedef struct messageToBeSent {
    char text[64];
    int intVal;
    float floatVal;
} messageToBeSent;
typedef struct receivedMessage {
    char text[64];
    long runTime;
} receivedMessage;
messageToBeSent myMessageToBeSent; 
receivedMessage myReceivedMessage; 
void messageSent(uint8_t *macAddr, uint8_t status) {
    Serial.print("Send status: ");
    if(status == 0){
        Serial.println("Success");
    }
    else{
        Serial.println("Error");
    }
}
void messageReceived(uint8_t* macAddr, uint8_t* incomingData, uint8_t len){
    memcpy(&myReceivedMessage, incomingData, sizeof(myReceivedMessage));
    Serial.printf("Incoming Message from: %02X:%02X:%02X:%02X:%02X:%02X \n\r", 
            macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);    
    Serial.print("Message: ");
    Serial.println(myReceivedMessage.text);
    Serial.print("RunTime [s]: ");
    Serial.println(myReceivedMessage.runTime);
    Serial.println();
}
void setup(){

    //pinMode(14, INPUT); //Interrupt-Pin ADXL345
    attachInterrupt(digitalPinToInterrupt(13), ISR1, RISING);

    //pinMode(13, INPUT); //Interrupt-Pin PN532
    attachInterrupt(digitalPinToInterrupt(12), ISR2, FALLING);

    pinMode(0, OUTPUT);
    digitalWrite(0, LOW);
    

    Serial.begin(115200);
    Wire.begin(SDA, SCL); // D5 -> GPIO14, D6 -> GPIO12
    u8g2.begin();
    delay(1000); // uncomment if your serial monitor is empty

    u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0,10,"Hello!");
      u8g2.sendBuffer();

    Serial.println("Setup Acc");
  if(!accSens.init()){
    Serial.println("ADXL345 not connected!");
    //FailureFlash(2);
  }
 accSens.setDataRate(ADXL345_DATA_RATE_200);
  accSens.setRange(ADXL345_RANGE_8G);

  accSens.setGeneralTapParameters(ADXL345_XYZ, 2.0, 30, 100);
  accSens.setAdditionalDoubleTapParameters(0, 200);
  accSens.setInterrupt(ADXL345_SINGLE_TAP, INT_PIN_2);
  accSens.setInterrupt(ADXL345_DOUBLE_TAP, INT_PIN_2);

  accSens.setActivityParameters(ADXL345_DC_MODE, ADXL345_XYZ, 1.3);
  accSens.setInterrupt(ADXL345_ACTIVITY, INT_PIN_2);
  accSens.setInterruptPolarity(0);
  Serial.println("Setup Acc done...");

// Pressure
    if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor"));
   
    }else{

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    Serial.println("Setup Pressure done...");
    }

  // NFC
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
    }else{
      // Got ok data, print it out!
      Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
      Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
      Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
      nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
    }

    
    // WiFi
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() == 0) {
        Serial.println("ESPNow Init success");
    }
    else {
        Serial.println("ESPNow Init fail");
        return;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    uint8_t result = esp_now_add_peer(receiverAddress, ESP_NOW_ROLE_COMBO, 0, NULL, 0);
    if(result != 0){
        Serial.println("Failed to add peer");
    }
    
    esp_now_register_send_cb(messageSent);  
    esp_now_register_recv_cb(messageReceived); 

    strip.begin();
  strip.setBrightness(128);
  strip.show(); 
   
}
 
void loop(){
    strip.clear();
    byte AccInt1 = 0;
    byte AccInt2 = 0;
    if(Int1Flag == 1)
    {
    AccInt1 = accSens.readAndClearInterrupts();
    
    if(accSens.checkInterrupt(AccInt1, ADXL345_SINGLE_TAP))
    {
      Serial.println("TAP!");
      Serial.println();
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0,10,"TAP!");
      u8g2.sendBuffer();
      delay(1000);
      strip.setPixelColor(0,0, 0, 128);
    }
    if(accSens.checkInterrupt(AccInt1, ADXL345_DOUBLE_TAP))
    {
      Serial.println("DOUBLE TAP!");
      Serial.println();
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0,10,"DOUBLE TAP!");
      u8g2.sendBuffer();
      delay(1000);
      strip.setPixelColor(0,0, 128, 0);
    }
    Int1Flag =0;
    }
    
    u8g2.clearBuffer();

    u8g2.sendBuffer();
   
  //   if (bmp.takeForcedMeasurement()) {
  //   // can now print out the new measurements
  //   Serial.print(F("Temperature = "));
  //   Serial.print(bmp.readTemperature());
  //   Serial.println(" *C");

  //   Serial.print(F("Pressure = "));
  //   Serial.print(bmp.readPressure());
  //   Serial.println(" Pa");

  //   Serial.print(F("Approx altitude = "));
  //   Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  //   Serial.println(" m");

  //   Serial.println();
    
  // } else {
  //   Serial.println("Forced measurement failed!");
  // }
  


  boolean success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };	// Buffer to store the returned UID
  uint8_t uidLength;				// Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  if(Int2Flag == 1){
    // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
    // 'uid' will be populated with the UID, and uidLength will indicate
    // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);

    if (success) {
      Serial.println("Found a card!");
      Serial.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
      Serial.print("UID Value: ");
      for (uint8_t i=0; i < uidLength; i++)
      {
        Serial.print(" 0x");Serial.print(uid[i], HEX);
      }
      Serial.println("");
    // Wait 1 second before continuing
      for (uint8_t i=0; i < 8; i++)
      {
      strip.setPixelColor(i,128, 0, 0);
      }
    }
    else
    {
      // PN532 probably timed out waiting for a card
      Serial.println("Timed out waiting for a card");
    }
    Int2Flag = 0;
    //nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);


    
  }
  /*if ((millis() % 100) == 1) {
    xyzFloat g;
    accSens.getGValues(&g);
    Serial.print("g-x   = ");
    Serial.print(g.x);
    Serial.print("  |  g-y   = ");
    Serial.print(g.y);
    Serial.print("  |  g-z   = ");
    Serial.println(g.z);
    Serial.println(accSens.getActTapStatusAsString());
    delay(1);
    char textMsg[] = "Hi, here's my data for you: ";
    
    memcpy(&myMessageToBeSent.text, textMsg, sizeof(textMsg));
    myMessageToBeSent.intVal = 4242;
    myMessageToBeSent.floatVal = 42.42;
    esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
  }*/
  //delay(2000);

  
  u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0,10,"Hello!");
      u8g2.sendBuffer();

    
  strip.setBrightness(128);
  strip.show();
  delay(100);

}
#endif
#ifndef SENSEI
uint8_t receiverAddress[] = {0xBC, 0xFF, 0x4D, 0x85, 0x7A, 0x00};
typedef struct messageToBeSent{
    char text[64];
    long runTime;
} messageToBeSent;
typedef struct receivedMessage {
    char text[64];
    int intVal;
    float floatVal;
} receivedMessage;
messageToBeSent myMessageToBeSent; 
receivedMessage myReceivedMessage; 
void messageReceived(uint8_t* macAddr, uint8_t* incomingData, uint8_t len){
    memcpy(&myReceivedMessage, incomingData, sizeof(myReceivedMessage));
    Serial.printf("Incoming Message from: %02X:%02X:%02X:%02X:%02X:%02X \n\r", 
            macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
    Serial.print("Message: ");
    Serial.println(myReceivedMessage.text);
    Serial.print("Integer Value: ");
    Serial.println(myReceivedMessage.intVal);
    Serial.print("Float Value: ");
    Serial.println(myReceivedMessage.floatVal);
    Serial.println();
    Serial.println("Sending answer...");
    Serial.println(); 
    char textMsg[] = "Thanks for the data!";
    memcpy(&myMessageToBeSent.text, textMsg, sizeof(textMsg));
    myMessageToBeSent.runTime = millis()/1000;
    esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));    
}    
void setup(){
    Serial.begin(115200);
    Wire.begin(SDA, SCL); // D5 -> GPIO14, D6 -> GPIO12
    u8g2.begin();
    // delay(1000); // uncomment if your serial monitor is empty
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() == 0) {
        Serial.println("ESPNow Init success");
    }
    else {
        Serial.println("ESPNow Init fail");
        return;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    uint8_t result = esp_now_add_peer(receiverAddress, ESP_NOW_ROLE_COMBO, 0, NULL, 0);
    if(result != 0){
        Serial.println("Failed to add peer");
    }
    
    esp_now_register_recv_cb(messageReceived);
}
 
void loop(){

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0,10,myReceivedMessage.text);
  u8g2.sendBuffer();
  delay(1000);
}
#endif
