/*Investigating LoRa For use in a Cattle Tracking and Monitoring System
  This is the first prototype for the gateway code
  Intended functionality is to receive time stamps from different nodes and to store them to an SD card for post processing
  Hardware Required: Arduino Uno, LoRa hat for arduino, MCP7940M RTC, Pushbutton, Arduio SD Card Module
  Author: Mark Njoroge
  Date: October 2020
*/

/*************** Include Libraries *******************/
#include <SPI.h>
#include <stdio.h>
#include <RH_RF95.h>
#include <MCP7940.h>
#include <SD.h>

/*************** Defining Constants ******************/
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define RF95_FREQ 915.0   // Radio Frequency, match to Rx
#define SYNCH_BUTTON 3   // Buton to start Synchronisation

/************ Function definitions *******************/
void button_pressed();
void setup_radio();
void setup_rtc();
void setup_sdcard();

//Instantiate classes
RH_RF95 rf95(RFM95_CS, RFM95_INT);
MCP7940_Class MCP7940;
//File myFile;

/*************** Global Variables *******************/
unsigned long current_microseconds;
unsigned long prev_microseconds;
unsigned long max_microseconds;
unsigned long last_interruptTime = 0;
int debounce_delay = 500;
int LED_state = 0;
bool synchronising;

/**************** First Start up *********************/
void setup() {  //set up code, runs once
  //setting up inputs and outputs
  pinMode(SYNCH_BUTTON, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  //while(!Serial) {
   // delay(1);
 // }
  Serial.println(F("Cattle Tracker - This is the Gateway, Welcome"));

  // Setting up the modules
  setup_radio();
  setup_rtc();
  //setup_sdcard();

  Serial.println(F("First start: Sending beacon to Nodes"));
  uint8_t data[] = "1";
  rf95.send(data,sizeof(data));
  rf95.waitPacketSent();


  
  
  
  //attaching interrupts
  attachInterrupt(digitalPinToInterrupt(SYNCH_BUTTON), button_pressed, RISING);

}

/**************** Main Code **********************/
void loop() {
  current_microseconds = micros();
  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  if (synchronising) {
    sprintf(data,"%lu",current_microseconds);
    rf95.send(data,sizeof(data));
    rf95.waitPacketSent();

    synchronising = false;
  }

}
/********************* Function *********************/
void button_pressed(){
  long interruptTime = millis(); //this doesnt work, need a new way to debounce

  if ((last_interruptTime - interruptTime) > debounce_delay) {
   //write interrupt logic here
   //plan is to start the synchronisation content
   Serial.print(F("Button Pressed at:   "));
   Serial.println(current_microseconds);
   synchronising = true;
  }
  last_interruptTime = interruptTime;
}


void setup_radio(){
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST,HIGH);
  
  // manual reset
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

void setup_rtc() {
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


void setup_sdcard() {
  if (!SD.begin(10)) {
    Serial.println(F("SD Card Initialization failed!"));
     while (1);
  }  
}
