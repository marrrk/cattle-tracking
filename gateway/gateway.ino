/*Investigating LoRa For use in a Cattle Tracking and Monitoring System
  This is the first prototype for the gateway code
  Intended functionality is to receive time stamps from different nodes and to store them to an SD card for post processing
  Author: Mark Njoroge
  Date: October 2020
*/

/*************** Include Libraries *******************/
#include <SPI.h>
#include <RH_RF95.h>
#include <MCP7940.h>
#include <SD.h>

/*************** Defining Constants ******************/
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define RF95_FREQ 915.0   // Radio Frequency, match to Rx
#define BEACON_BUTTON 3   // Buton to start Synchronisation

/************ Function definitions *******************/
void button_pressed();
void setup_radio();
void setup_rtc();
void setup_sdcard();

//Instantiate classes
RH_RF95 rf95(RFM95_CS, RFM95_INT);
MCP7940_Class MCP7940;
File myFile;

/*************** Global Variables *******************/
unsigned long micro_seconds;
unsigned long last_interruptTime = 0;
int debounce_delay = 500;
int LED_state = 0;
bool first_start = true;
//bool synchronising = true;

/**************** Setting up *********************/
void setup() {  //set up code, runs once
  //setting up inputs and outputs
  pinMode(BEACON_BUTTON, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST,HIGH);

  Serial.begin(9600);
  Serial.println(F("Cattle Tracker - This is the Gateway, Welcome"));

  // Setting up LoRa Radio
  setup_radio();
  
  //attaching interrupts
  attachInterrupt(digitalPinToInterrupt(BEACON_BUTTON), button_pressed, RISING);
}

/**************** Main Code **********************/
void loop() {
  if (first_start) {
    //send alert messages to nodes
  }
}

void button_pressed(){
  long interruptTime = millis();

  if ((last_interruptTime - interruptTime) > debounce_delay) {
   //write interrupt logic here
   //plan is to start the synchronisation content
   //synchronising = true;
  }
  last_interruptTime = interruptTime;
}


void setup_radio(){
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
  
  
}
