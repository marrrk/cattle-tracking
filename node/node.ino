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
void determineMessageType(char indicator);
bool synchronized();

/******************* Instantiate classes **************************/
RH_RF95 rf95(RFM95_CS, RFM95_INT);
MCP7940_Class MCP7940;


/******************** Global Variables ******************************/
unsigned long prev_micros;
unsigned long current_micros;
unsigned long time_stamp;
uint8_t count = 0;
bool synchronise;


void setup() {
  Serial.begin(9600);
  //while(!Serial) {
    //delay(1);
  //}
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  setup_radio();
  setup_rtc();

  prev_micros = micros();  
  //attachInterrupt(digitalPinToInterrupt(RFM95_INT), determineMessageType, CHANGE);
}

void loop() {
  //sleep for a while, ideally
  current_micros = micros();
  time_stamp = current_micros - prev_micros;
  if (time_stamp>1E6) {
    prev_micros = current_micros; 
  }
  if (rf95.available()) {   //message available OR synchronizing
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      String message = (char*)buf;
      determineMessageType(message[0]);
      //Serial.println(message[0]);
      //RH_RF95::printBuffer("Received: ", buf, len);

      Serial.print(F("Got: "));
      Serial.println(message);

      //received message, now reply accordingly. if synchronizing or not 
      if (synchronise) {
        //synchronizing logic, send the time back and wait
        Serial.println(F("Syncrhonizing"));
      }
      else { Serial.println(F("Synchronizing complete")); }
      
      rf95.sleep();
    } 
    else { 
        Serial.println(F("Receive Failed"));  
      }
      
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN,HIGH);
  } else {              //no message received, send the signal containing information from node
    //delay(5000); //wait five seconds
    //Serial.print("Device time:   ");
    //Serial.println(time_stamp);
  }
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

  Serial.println(F("RTC Setup OK!"));
}


void determineMessageType(char indicator) {
  if (indicator == '1') {
    // the gateway has just turned on, do nothing
    //Serial.println("This is a test");
  }
  else if (indicator == '2') {
    synchronise = true;     //gateway has started the synchronisation procedure
  }
  else if (indicator == '3') {
    synchronise = false;    //the timings have synchronised
  }
}
