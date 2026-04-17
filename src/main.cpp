
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ESPNowW.h>
#include "ESP8266WiFi.h"
#include <ADXL345_WE.h>
#include <Adafruit_PN532.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_NeoPixel.h>
#include "reaction_Esp.h"
#include "app/AppEvents.h"
#include "app/Protocol.h"
#include "app/NodeConfig.h"
#include "stm.h"
#include "states/InitState.h"


#define SCL 5//14
#define SDA 4//2

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ 13
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

#define CHANNEL 1

#define SENSEI

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);
ADXL345_WE accSens = ADXL345_WE(0x53);
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
Adafruit_BMP280 bmp; // I2C
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, 0, NEO_GRB + NEO_KHZ800);

static struct Ctx Data;
static app_event_queue_t EventQueue;
uint8_t changes = 0;

/**
 * @brief CFSM state machine instance
 */
static cfsm_Ctx stateMachine;

volatile char Int1Flag = 0;
volatile char Int2Flag = 0;

void IRAM_ATTR ISR1() {
    Int1Flag = 1;
    
    //detachInterrupt(25);
}
void IRAM_ATTR ISR2() {
    Int2Flag = 1;
    //detachInterrupt(25);
}

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t myAdress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};




message_t MessageToBeSent[3]; 
message_t ReceivedMessage; 

uint8_t idByMacAdress(uint8_t* macAddr){
  return lightIdByMacAddress(macAddr);
}

void messageSent(uint8_t *macAddr, uint8_t status) {
    if(status == 0){
        Serial.printf("Send Message to: %02X:%02X:%02X:%02X:%02X:%02X \n\r", 
            macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]); 
            Serial.println("Success"); 
            changes = 1; 
    }
    else{
      Serial.printf("Send Message to: %02X:%02X:%02X:%02X:%02X:%02X \n\r", 
            macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);  
        Serial.println("Error");
         Serial.println(status);
    }
   
}

void messageReceived(uint8_t* macAddr, uint8_t* incomingData, uint8_t len){
    (void)macAddr;
    if (!isExpectedMessageSize(len)) {
      Serial.print("Unexpected message size: ");
      Serial.println(len);
      return;
    }

    memcpy(&ReceivedMessage, incomingData, sizeof(ReceivedMessage));
    if (!isValidMessageForGroup(&ReceivedMessage, Data.groupId)) {
      Serial.println("ignored, different group");
      return;
    }

    app_event_t event = {};
    event.eventId = EVENT_NETWORK_MESSAGE;
    event.stateId = ReceivedMessage.stateId;
    event.groupId = ReceivedMessage.groupId;
    event.sourceLightId = ReceivedMessage.sourceLightId;
    event.targetLightId = ReceivedMessage.targetLightId;
    event.message = ReceivedMessage;
    if (!appEventQueuePush(&EventQueue, &event)) {
      Serial.println("ignored, event queue full");
    }
}

void setup(){
    
    Data.lightId = 2;
    
    attachInterrupt(digitalPinToInterrupt(PIN_ADXL_INTERRUPT), ISR1, RISING);

    attachInterrupt(digitalPinToInterrupt(PIN_PN532_INTERRUPT), ISR2, FALLING);

    pinMode(0, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(PIN_GROUP_SELECT, INPUT);//AB
    pinMode(PIN_TOUCH, INPUT);//Touch
    pinMode(PIN_MASTER_SELECT, INPUT);//Master Jumper

    digitalWrite(0, LOW);
    //digitalWrite(2, HIGH);

    Serial.begin(115200);
    Wire.begin(SDA, SCL); // D5 -> GPIO14, D6 -> GPIO12
    u8g2.begin();
    delay(1000); // uncomment if your serial monitor is empty

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0,10,"Hello!");
    u8g2.sendBuffer();

    Data.role = roleFromMasterSwitch(digitalRead(PIN_MASTER_SELECT));
    Data.isMaster = Data.role == ROLE_MASTER;
    if(Data.isMaster == 1){
       Serial.println("MASTER");
    }

    Data.groupId = groupFromSwitch(digitalRead(PIN_GROUP_SELECT));
    Data.requestedStateId = STATE_INIT;
    Data.stateId = STATE_INIT;
    Data.activeLightId = NO_ACTIVE_LIGHT;
    Data.registeredLights[Data.lightId] = 1;
    appEventQueueInit(&EventQueue);
    Data.light[Data.lightId].lightId = Data.lightId;
    Data.light[Data.lightId].groupId = Data.groupId;
    Serial.print("GROUP ");
    Serial.println(Data.groupId == GROUP_A ? "A" : "B");
   

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
    Serial.print("Setup Pressure done...");
    if (bmp.takeForcedMeasurement()) {
      Data.initialPressure = bmp.readPressure();
      Serial.println(Data.initialPressure);
    }
  }

  // NFC
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.println("Didn't find PN53x board");
    //while (1); // halt
    }else{
      // Got ok data, print it out!
      Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
      Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
      Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
      nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
    }

    // WiFi
    WiFi.mode(WIFI_STA);
    wifi_promiscuous_enable(true);
    wifi_set_channel(CHANNEL);
    wifi_promiscuous_enable(false); 
    delay(1000);

    if (esp_now_init() == 0) {
      Serial.println("ESPNow Init success");
      String mac = WiFi.macAddress();
      Serial.println(mac);
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0,10,mac.c_str());
      u8g2.sendBuffer();
    }
    else {
        Serial.println("ESPNow Init fail");
        return;
    }
    
    if(Data.isMaster == 1){
      uint8_t result;
      esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
      for(uint8_t i = 0; i < 3; i++){
        result = esp_now_add_peer(SlaveAddress[i], ESP_NOW_ROLE_SLAVE, CHANNEL, NULL, 0);
        if(result != 0){
            Serial.print("Failed to add peer");
            Serial.println(i + 1);
        }
      }
    }else{
      esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
      uint8_t result = esp_now_add_peer(MasterAddress, ESP_NOW_ROLE_CONTROLLER, CHANNEL, NULL, 0);
      if(result != 0){
          Serial.println("Failed to add peer");
      }
    }
    
  esp_now_register_send_cb(messageSent);  
  esp_now_register_recv_cb(messageReceived); 

  strip.begin();
  strip.setBrightness(255);
  strip.show(); 

  delay(3000);  
  

  cfsm_init(&stateMachine, &Data);
  cfsm_transition(&stateMachine, InitState_enter);

  Serial.println();
}
 
void loop(){
    strip.clear();
    app_event_t event = {};
    while (appEventQueuePop(&EventQueue, &event)) {
        if (event.eventId == EVENT_NETWORK_MESSAGE && event.message.messageType == MSG_STATE_SET) {
            Data.requestedStateId = event.message.stateId;
        }
        if (event.eventId == EVENT_NETWORK_MESSAGE) {
            Data.light[0] = event.message;
            changes = 1;
        }
        if (Data.role == ROLE_MASTER && event.message.messageType == MSG_REGISTER) {
            uint8_t lightId = event.message.sourceLightId;
            if (lightId < MAX_LIGHTS) {
                Data.registeredLights[lightId] = 1;
                message_t ack = makeRegisterAckMessage(lightId, Data.groupId);
                uint8_t peerAddress[6] = {};
                if (copyPeerAddressForLight(lightId, peerAddress)) {
                    esp_now_send(peerAddress, (uint8_t*)&ack, sizeof(ack));
                }
            }
        }
        cfsm_event(&stateMachine, event.eventId);
    }
    cfsm_process(&stateMachine);

    uint8_t DisplayFlag = 0;
    byte AccInt1 = 0;
    //byte AccInt2 = 0;
    //uint8_t light_id = Data.lightId;
    Data.groupId = groupFromSwitch(digitalRead(PIN_GROUP_SELECT));
    Data.light[Data.lightId].lightId = Data.lightId;
    Data.light[Data.lightId].groupId = Data.groupId;

    u8g2.clearBuffer();
          
    //u8g2.sendBuffer();
    
    if(digitalRead(PIN_TOUCH)==1){
      Data.light[Data.lightId].touched = 1;
      Serial.println("Touch!");
    }else{
      Data.light[Data.lightId].touched = 0;
    }

    Data.light[Data.lightId].modeAB = Data.groupId;


    if(Int1Flag == 1)
    {
      DisplayFlag = 10;
      AccInt1 = accSens.readAndClearInterrupts();
      
      if(accSens.checkInterrupt(AccInt1, ADXL345_SINGLE_TAP))
      {
        Serial.println("TAP!");
        Data.light[Data.lightId].tap = 1;
        
        if(Data.isMaster == 1){
          Data.light[0].tap = 1;
          //esp_now_send(receiverAddress, (uint8_t *) &MessageToBeSent, sizeof(MessageToBeSent));
          //Data.light[0].color_r = 255;
          //Data.light[2].color_g = 255;
          //Data.light[3].color_b = 255;
        
        }else{
          memcpy(&MessageToBeSent[0], &Data.light[Data.lightId], sizeof(MessageToBeSent[0]));
          MessageToBeSent[0].tap = 1;
          MessageToBeSent[0].lightId = Data.lightId;
          MessageToBeSent[0].groupId = Data.groupId;
          if(esp_now_send(MasterAddress, (uint8_t *) &MessageToBeSent[0], sizeof(MessageToBeSent[0])) == 0){
            Serial.println("Send!");
          }
        }

        //MessageToBeSent[Data.lightId].tap = 0;

        changes = 1;
        
      }
      /*if(accSens.checkInterrupt(AccInt1, ADXL345_DOUBLE_TAP))
      {
        
      }*/
      Int1Flag =0;
    }
   
    /*if (bmp.takeForcedMeasurement()) {
      if((bmp.readPressure() + PRESSURE_OFFSET)> Data.initialPressure){
        //Data.squeezed[Data.lightId] = 1;
        for (uint8_t i=0; i < 8; i++)
        {
          strip.setPixelColor(i,0, 128, 0);
        }
      }else{
        for (uint8_t i=0; i < 8; i++)
        {
          strip.setPixelColor(i,0, 0, 0);
        }
      }
    }*/
  

  /*if(Data.isMaster == 1){
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
  }*/
  //strip.show();

  if(changes == 1){

    
    if(Data.isMaster == 1){

      if(Data.light[0].tap == 1){
        Data.light[0].color_r = 255;
        Data.light[2].color_r = 255;
        Data.light[3].color_r = 255;
        Data.light[0].color_g = 0;
        Data.light[2].color_g = 0;
        Data.light[3].color_g = 0;
        Data.light[0].color_b = 0;
        Data.light[2].color_b = 0;
        Data.light[3].color_b = 0;
        Serial.println("Tap0");
      }else{
        
      }
      if(Data.light[2].tap == 1){
        Data.light[0].color_r = 0;
        Data.light[2].color_r = 0;
        Data.light[3].color_r = 0;
        Data.light[0].color_g = 255;
        Data.light[2].color_g = 255;
        Data.light[3].color_g = 255;
        Data.light[0].color_b = 0;
        Data.light[2].color_b = 0;
        Data.light[3].color_b = 0;
        Serial.println("Tap2");
      }else{
        
      }
      if(Data.light[3].tap == 1){
        Data.light[0].color_r = 0;
        Data.light[2].color_r = 0;
        Data.light[3].color_r = 0;
        Data.light[0].color_g = 0;
        Data.light[2].color_g = 0;
        Data.light[3].color_g = 0;
        Data.light[0].color_b = 255;
        Data.light[2].color_b = 255;
        Data.light[3].color_b = 255;
        Serial.println("Tap3");
      }else{
        
      }

      for(uint8_t i=0; i<3; i++){
        uint8_t targetLightId = i + 1;
        memcpy(&MessageToBeSent[i], &Data.light[targetLightId], sizeof(MessageToBeSent[i]));
        MessageToBeSent[i].lightId = targetLightId;
        MessageToBeSent[i].groupId = Data.groupId;
        esp_now_send(SlaveAddress[i], (uint8_t *) &MessageToBeSent[i], sizeof(MessageToBeSent[i]));
      }
      //memcpy(&MessageToBeSent[2], &Data.light[3], sizeof(message_t));
      //esp_now_send(SlaveAddress[2], (uint8_t *) &MessageToBeSent[2], sizeof(MessageToBeSent));
      //Data.light[0].color_r = 0;
      //Data.light[2].color_g = 0;
      //Data.light[3].color_b = 0;
      
    }
    for (uint8_t i=0; i < 8; i++)
    {
      strip.setPixelColor(i,Data.light[Data.lightId].color_r, Data.light[Data.lightId].color_g, Data.light[Data.lightId].color_b);
    }
  strip.show();
  changes = 0;
  }

  
  //u8g2.clearBuffer();
  if(DisplayFlag > 0){
    
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0,10,"TAP!");
    u8g2.sendBuffer();

    DisplayFlag--;    
  }else{
    Data.light[3].tap = 0;
    Data.light[2].tap = 0;
    Data.light[0].tap = 0;
    
  }
  u8g2.sendBuffer();
  delay(100);
  
}
//#endif

