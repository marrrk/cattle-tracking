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
#define RFM95_CS 10       //chip select for SPI line on radio
#define RFM95_RST 9
#define RFM95_INT 2
#define RF95_FREQ 915.0   // Radio Frequency, match to Rx
#define SYNCH_BUTTON 3   // Buton to start Synchronisation
#define SDCARD_CS 4       // chip select for SPI line on SDCARD
#define INPUT_CAPTURE 8          // Port B, pin 0, input interrupt pin 

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
uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
String message;
int debounce_delay = 500;
int LED_state = 0;
bool synchronising;


/**************** First Start up *********************/
void setup() {  //set up code, runs once
  //setting up inputs and outputs
  pinMode(SYNCH_BUTTON, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INPUT_CAPTURE, OUTPUT);
  
  Serial.begin(9600);
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  //OCR1A = 50000;
  TCCR1B |= (1 << ICNC1);       //enable noise cancelling
  TCCR1B |= (1 << ICES1);       //interrupt occurs on rising edge
  TCCR1B |= (1 << CS10);        //no prescaler
  //TCCR1B |= (1 << WGM12);

  TIMSK1 |= (1 << ICIE1);       //enable input capure interrupt 
  TIMSK1 |= (1 << TOIE1);       // enable overflow interrupt
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
   /*
  delay(1000);
  //Setting up the timers and interrupts
  //noInterrupts();   //disable interrupts so nothing happens while writing to registers 
  //cli();
  //reset everything to zero to start fresh
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  TCCR1B |= (1 << ICNC1);
  TCCR1B |= (1 << ICES1);       //interrupt occurs on rising edge
  TCCR1B |= (1 << CS10);        //no prescaler


  TIMSK1 |= (1 << ICIE1);       //enable input capure interrupt 
  TIMSK1 |= (1 << TOIE1);       // enable overflow interrupt
  //interrupts();
  //sei();

  */
  //attaching interrupts
  attachInterrupt(digitalPinToInterrupt(SYNCH_BUTTON), button_pressed, RISING);

}

/********************** Interrupt Service Routines ***************************/
ISR(TIMER1_OVF_vect) {        //timer one overflow interrupt service routine
  //Serial.println("Overflow Detected");
  cycles++;                   //increments the number of cycles done
  //Serial.print("Cycles:   ");
  //Serial.println(cycles);
}

ISR(TIMER1_CAPT_vect) {
  //perform the distance calculation maybe ?
  //write to sd card
  time_stamp = ICR1 + cycles;   //probs wrong uno
  //message = (char*)buf;
  Serial.println("Input Capture Triggered");
  Serial.println(ICR1);
  Serial.println(cycles);
  PORTB = 0;
}


/****************************** Main Code ************************************/
void loop() {
  
  if (synchronising) {    //gonna have to do synchronizing things here.
    uint8_t message_cycles[100];
    uint8_t message_time[50];
    unsigned long transmit_time = TCNT1;
    strcpy(data,"2"); 
    sprintf(message_cycles, "%lu", cycles);
    sprintf(message_time,"%lu",transmit_time);
    
    strncat(data, message_cycles,sizeof(message_cycles));
    rf95.send(data,sizeof(data));
    rf95.waitPacketSent();

    synchronising = false;
  } 
  else if(rf95.available()) {
    uint8_t len = sizeof(buf);
    if(rf95.recv(buf, &len)) {
      PORTB |= (1 << PB0);
      
    }
  }

}
/********************* Function *********************/
void button_pressed(){
  long interruptTime = millis(); //this doesnt work, need a new way to debounce

  if ((last_interruptTime - interruptTime) > debounce_delay) {
   //write interrupt logic here
   //plan is to start the synchronisation content
   Serial.print(F("Button Pressed at:   "));
   Serial.println(TCNT1);
   //PORTB |= (1 << PB0);
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
