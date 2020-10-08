/* Investigating Lora For use in a Cattle Tracking and Monitoring System
 * This is the first prototype for the Sensor Node
 * Intended Functionality is to broadcast a message to all receiving gateways which will determine it's location
 * Hardware Required: Adafruit Feather 32u4 wih LoRa Module, MCP7940M RTC
 * Author: Mark Njoroge
 * Date: October 2020
 */

/****************** Include Libraries ***********************/
#include <SPI.h>
#include <RH_RF95.h>
#include <MCP7940.h>

/***************** Defining Constants **********************/
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RF95_FREQ 915.0    // Radio Frequency, match to Tx


/***************** Function Definitions **************************/
void setup_radio();
void setup_rtc();


/******************* Instantiate classes **************************/
RH_RF95 rf95(RFM95_CS, RFM95_INT);
MCP7940_Class MCP7940;


/******************** Global Variables ******************************/
unsigned long micro_seconds;



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  setup_radio();
  setup_rtc();

}

void loop() {
  // put your main code here, to run repeatedly:


}


void setup_radio(){
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST,HIGH);

  //manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  Serial.println(F("Initialising Radio"));
  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1);
  }
  Serial.println(F("LoRa radio init OK!"));

  //setting the frequency
 if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1);
  } 

  rf95.setTxPower(23, false);
}


void setup_rtc(){
  while (!MCP7940.begin()) {
    Serial.println(F("Unable to find MCP7940M."));
    while(1);
  }
  while (!MCP7940.deviceStatus()) {// Turn oscillator on if necessary  //
    Serial.println(F("Oscillator is off, turning it on."));
    bool deviceStatus = MCP7940.deviceStart();  // Start oscillator and return state//
    if (!deviceStatus) {
      Serial.println(F("Oscillator did not start, trying again."));
      delay(1000);
    } // of if-then oscillator didn't start
  } // of while the oscillator is off
  MCP7940.adjust();                           
  
}
