/*Investigating LoRa For use in a Cattle Tracking and Monitoring System
  This is the first prototype for the gateway code
  Intended functionality is to receive time stamps from different nodes and to store them to an SD card for post processing
  Hardware Required: Arduino Uno, LoRa hat for arduino, MCP7940M RTC, Pushbutton, Arduio SD Card Module
  Author: Mark Njoroge
  Date: October 2020
*/

/*************** Include Libraries *******************/
#include <SPI.h>
//#include <stdio.h>
#include <RH_RF95.h>
//#include <MCP7940.h>
//#include <SD.h>

/*************** Defining Constants ******************/
#define RFM95_CS 8       //chip select for SPI line on radio
#define RFM95_RST 4
#define RFM95_INT 7
#define RF95_FREQ 915.0   // Radio Frequency, match to Rx
#define SYNCH_BUTTON 3   // Buton to start Synchronisation
//#define SDCARD_CS 4       // chip select for SPI line on SDCARD
#define INPUT_CAPTURE 13          // Port B, pin 0, input interrupt pin 

/************ Function definitions *******************/
void button_pressed();
void setup_radio();
void setup_rtc();
void setup_sdcard();

//Instantiate classes
RH_RF95 rf95(RFM95_CS, RFM95_INT);
//MCP7940_Class MCP7940;
//File myFile;

/*************** Global Variables *******************/
unsigned long cycles = 0;
unsigned long last_interruptTime = 0;
unsigned long time_stamp;


uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
int debounce_delay = 500;
int LED_state = 0;
bool synchronising = true;


/**************** First Start up *********************/
void setup() {  //set up code, runs once
  //setting up inputs and outputs
  pinMode(SYNCH_BUTTON, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INPUT_CAPTURE, OUTPUT);
  
  Serial.begin(9600);
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;
  TCNT3 = 0;
  //OCR1A = 50000;
  TCCR3B |= (1 << ICNC3);       //enable noise cancelling
  TCCR1B |= (1 << ICES3);       //interrupt occurs on rising edge
  TCCR1B |= (1 << CS30);        //no prescaler
  //TCCR1B |= (1 << WGM12);

  TIMSK1 |= (1 << ICIE3);       //enable input capure interrupt 
  TIMSK1 |= (1 << TOIE3);       // enable overflow interrupt
  //TIMSK1 |= (1 << OCIE1A);

  interrupts();
  
  Serial.println(F("Cattle Tracker - This is the Gateway, Welcome"));

  // Setting up the modules
  setup_radio();
  //setup_rtc();
  //setup_sdcard();
  
  Serial.println(F("First start: Sending beacon to Nodes"));
  uint8_t beacon[] = "1"; 
  rf95.send(beacon,sizeof(beacon));
  rf95.waitPacketSent();
  
  //attaching interrupts
  attachInterrupt(digitalPinToInterrupt(SYNCH_BUTTON), button_pressed, RISING);

}

/********************** Interrupt Service Routines ***************************/
ISR(TIMER3_OVF_vect) {        //timer one overflow interrupt service routine
  cycles++;                   //increments the number of cycles done
}

ISR(TIMER3_CAPT_vect) {
  //perform the distance calculation maybe ?
  unsigned long receive_time = (ICR3 + (cycles * 0xFFFF));   //probs wrong uno
  char *eptr;
  if (((char*)buf)[0] == '2') { //received a synch type message
     //based on synch technique: store drift amount/store synch time.
     
  }
  else if (((char*)buf)[0] == '4') {//received a time_stamp
    //calculate distance, and display distance 
    unsigned long message_time = strtoul((char*)buf+1, &eptr, 10);
    Serial.print("Time stamp received:   ");
    Serial.println(message_time);
    Serial.print("Time of arrival:    ");
    Serial.println(receive_time);
    long difference = receive_time - message_time;
    
    double distance = (difference * 125E-9) * 2.9925E8;
    Serial.print("Message Received. Distance calculated is: ");
    Serial.println(distance);
  }
  PORTC = 0;
}


/****************************** Main Code ************************************/
void loop() {
  char data[RH_RF95_MAX_MESSAGE_LEN];       //change to uint8_t when removing print statements
  char *eptr;
  
  
  if (synchronising) {    //gonna have to do synchronizing things here
    char time_to_send[100];
    cycles = 0;
    unsigned long transmit_time = ((cycles * 0xFFFF) + TCNT3);
   
    strcpy(data,"2"); 
    sprintf(time_to_send, "%lu", transmit_time);
    strncat(data, time_to_send,sizeof(time_to_send));
  
    //Serial.println(data);
    
    rf95.send(data,sizeof(data));
    rf95.waitPacketSent();

    synchronising = false;
  } 
  else if(rf95.available()) {
    uint8_t len = sizeof(buf);
    if(rf95.recv(buf, &len)) {
      //PORTB |= (1 << PB0);
      PORTC |= (1 << PC7);
    }
  }

}
/********************* Function *********************/
void button_pressed(){
  long interruptTime = TCNT3; //debouncing
  
  if ((last_interruptTime - interruptTime) > 0xFFFF) {
   Serial.println("Button Pressed");
   //write interrupt logic here
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
/*
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


/*void setup_sdcard() {
  if (!SD.begin(SDCARD_CS)) {
    Serial.println(F("SD Card Initialization failed!"));
     while (1);
  }  
} */
