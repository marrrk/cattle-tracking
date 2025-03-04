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
#define SYNCH_BUTTON 2   // Buton to start Synchronisation
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
int count = 0;
long delta;
long propagation_delay;
unsigned long receive_time;
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
int debounce_delay = 500;
int LED_state = 0;
bool synchronising = false;
bool start_synch = false;

//ModemConfigChoice configuration;
const int MEAN = 12553;
const int STD = 310;
const char ID[3] = "G00";

/**************** First Start up *********************/
void setup() {  //set up code, runs once
  Serial.begin(9600);
  
  //setting up inputs and outputs
  pinMode(SYNCH_BUTTON, INPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INPUT_CAPTURE, OUTPUT);
  setup_radio();
  
  
  noInterrupts();
  //Timer/counter for tracking
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;
  TCNT3 = 0;
  //OCR1A = 50000;
  TCCR3B |= (1 << ICNC3);       //enable noise cancelling
  TCCR3B |= (1 << ICES3);       //interrupt occurs on rising edge
  TCCR3B |= (1 << CS30);        //no prescaler
  //TCCR1B |= (1 << WGM12);

  TIMSK3 |= (1 << ICIE3);       //enable input capure interrupt 
  TIMSK3 |= (1 << TOIE3);       // enable overflow interrupt
  //TIMSK1 |= (1 << OCIE1A);

  //Timer to trigger every 8 seconds. To trigger synchronization procedure
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;

  OCR1A = 62500;      //OCR Value, makes a frequency of 0.125Hz. Value = 8MHZ/1024/0.125

  TCCR1B |= (1 << WGM12);     //CTC Mode, MAX = OCR
  TCCR1B |= (1 << CS12) | (1 << CS10);    //Prescaler = 1024

  TIMSK1 |= (1 << OCIE1A);     //Enables Output Compare Interrupt A

  interrupts();
  
  Serial.println(F("Cattle Tracker - This is the Gateway, Welcome"));

  // Setting up the modules
  //setup_radio();
  //setup_rtc();
  //setup_sdcard();
  
  Serial.println(F("First start: Sending beacon to Nodes"));
  TCNT3 = 0;
  cycles = 0;
  uint8_t beacon[4] = "G00";
  strncat(beacon,"1",sizeof("1")); 
  rf95.send(beacon,sizeof(beacon));
  rf95.waitPacketSent();

  start_synch=true;
  //attaching interrupts
  //attachInterrupt(digitalPinToInterrupt(SYNCH_BUTTON), button_pressed, RISING);

}

/********************** Interrupt Service Routines ***************************/
ISR(TIMER3_OVF_vect) {        //timer one overflow interrupt service routine
    cycles++;                   //increments the number of cycles done
}

ISR(TIMER3_CAPT_vect) {
  //perform the distance calculation maybe ?
  receive_time = ((ICR3 + (cycles * 0xFFFF)));// * 1.00335); //0883746);   // time of received message, accounting for clock drift
  //Serial.println((char*)buf);
  char *eptr;
  if (((char*)buf)[3] == '3') { //received a synch type message
    //TCNT3 = strtoul((char*)buf+1, &eptr, 10);
    //cycles = 0;
    Serial.println("Synchronising");
    synchronising = true;     //node has started the synchronisation procedure     
     
  }
  else if (((char*)buf)[3] == '4') {//received a time_stamp
    //calculate distance, and display distance
    //Serial.print("Message Received:   "); Serial.println((char*)buf);
    char *received[2];
    char *ptr = NULL;
    byte index = 0;

    ptr = strtok((char*)buf, "-");

    while(ptr != NULL) { //loop to get the rest of the data
      received[index] = ptr;
      index++;
      ptr = strtok(NULL, "-");
    }
    
    unsigned long message_time = strtoul(received[0]+4, &eptr, 10);      //reading in the time_stamp received
    //unsigned long message_time = strtoul((char*)buf+4, &eptr, 10);
    Serial.print("Time stamp received:   "); Serial.println(message_time);
    Serial.print("Time of arrival:    ");  Serial.println(receive_time);
    Serial.print("Battery Received:   "); Serial.println(received[1]);
    long difference = receive_time - message_time; // - propagation_delay - delta;          //calculating time difference
    Serial.print("Difference clock times:  ");   Serial.println(difference);

    difference = difference - MEAN;
    Serial.print(F("Difference in clock times minus communication errors:   ")); Serial.println(difference);
    
    difference = abs(difference / STD);
    Serial.print(F("Number of Standard Deviations within:  "));  Serial.println(difference);
    
    double distance = (difference * 125E-9) * 2.9925E8;     //calculating distance
    Serial.print("Message Received. Distance calculated is: ");   Serial.println(distance);

    Serial.print("RSSI: ");   Serial.println(rf95.lastRssi(), DEC);
  }
  PORTC = 0;
}

ISR(TIMER1_COMPA_vect) {
  count++;

  if (count == 3) {
    start_synch = true;
    count = 0;
  }
}

//want it to send a message to tell the node to start synchronizing

/****************************** Main Code ************************************/
void loop() {
  char data[RH_RF95_MAX_MESSAGE_LEN];       //change to uint8_t when removing print statements
  char *eptr;

  if (start_synch) {
    strcpy(data,ID);
    strncat(data,"2",sizeof("2"));
    Serial.println(F("Initialising Syncrhonization Protocol"));
    //Serial.println(data);
    //Send the Synch Beacon
    rf95.send(data,sizeof(data));
    rf95.waitPacketSent();

    start_synch = false;
  }
  else if (synchronising) {    //gonna have to do synchronizing things here
    char ack_message_time[20];
    char pulse_receive_time[20];
    String pulse_message_time;
   

    /********** Include ID ***********/
    strcpy(data,ID);
    
    /******* Set type of message *********/
    strncat(data,"3",sizeof("3"));
    //Serial.print("message identifier:  ");
    //Serial.println(data);
    strncat(data,"-",1); 
    
    /*** add in the received time_stamp from gateway to the message, T1  ***/
    //pulse_message_time = (char*)buf;
    strncat(data, (char*)buf+4, sizeof(buf));
    strncat(data,"-",1);
    //strncat(data, "00000000000000000000", (21-strlen(buf)));
    //test to see what the story says
    //Serial.print("T1:    ");
    //Serial.println((char*)buf+1);
    //Serial.println(sizeof(buf));
    //Serial.println(strlen(buf));

    //Serial.print("message with T1:   ");
    //Serial.println(data);
    
    /*** add in the time the message was received, add in T2   ***/
    sprintf(pulse_receive_time, "%lu", receive_time);
    //Serial.println(pulse_receive_time);
   
    //strncat(pulse_receive_time, "00000000000000000000", (20-strlen(pulse_receive_time)));
    strncat(data, pulse_receive_time, sizeof(pulse_receive_time));
    strncat(data,"-",1);
    // test to see what the story says
    //Serial.print("T2:    ");
    //Serial.println(pulse_receive_time);
    //Serial.println(sizeof(pulse_receive_time));
    //Serial.println(strlen(pulse_receive_time));

    //Serial.println("message with T2:   ");
    //Serial.println(data);
    
    /******* finally add in T3 **********/
    unsigned long ack_time = ((cycles * 0xFFFF) + TCNT3);// * 1.00335; // 1.003340883746;
    sprintf(ack_message_time, "%lu", ack_time);
    strncat(data, ack_message_time, sizeof(ack_message_time));
    //strncat(data, "-", 1);
    Serial.println(data);

    //transmit data
    rf95.send(data,sizeof(data));
    rf95.waitPacketSent();

    
    //testing how to print the things
    /*char *strings[5];
    char *ptr = NULL;
    byte index = 0;

    ptr = strtok(data, "-");

    while(ptr != NULL) {
      strings[index] = ptr;
      index++;
      ptr = strtok(NULL, "-");
    }

    for (int n = 0; n< index; n++) {
      Serial.println(strings[n]);
    } */

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

void setup_radio(){
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST,HIGH);

  while(!Serial) {
    delay(1);
  }
  
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

  rf95.setTxPower(1, false);
 /* if (!rf95.setModemConfig(2)) {
    Serial.println(F("Setting Configuration Failed"));
    while(1);
  }
  else { Serial.println("Setting Configuration Success"); }
  */

}
