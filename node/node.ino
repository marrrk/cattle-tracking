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
#define INPUT_CAPTURE 13 // ICP3 = PC7


/***************** Function Definitions **************************/
void setup_radio();
void setup_rtc();
void determineMessageType(char indicator);
bool synchronized();



/******************* Instantiate classes **************************/
RH_RF95 rf95(RFM95_CS, RFM95_INT);
MCP7940_Class MCP7940;


/******************** Global Variables ******************************/
unsigned long time_stamp;
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t count = 0;
bool synchronise;
bool send_timestamp;
unsigned long cycles;


/****************** SETTING UP ***************************/

void setup() {
  Serial.begin(9600);
  //while(!Serial) {
    //delay(1);
  //}
  pinMode(INPUT_CAPTURE, OUTPUT); //set ICP3 pin as an output to allow writing to the port
  setup_radio();
  //setup_rtc();
  
  //Setting up the timers and interrupts
  //Starting with Timer3, the one used for the system tracking clock.
  //reset everything to zero
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;
  TCNT3 = 0;

  TCCR3B |= (1 << ICES3);     // input interrupt capture on rising edge
  TCCR3B |= (1 << ICNC3);     // noise cancelling on input capture
  //TCCR3B |= (1 << CS32);      //prescaler value, want no prescaler
  TCCR3B |= (1 << CS30);      //no prescaler

  TIMSK3 |= (1 << ICIE3);      //enable input interrupt
  TIMSK3 |= (1 << TOIE3);      //enable overflow interrupt
  
  //Next, setting up timer 1 for OCR. Interrupt to occur at 2Hz, will allow for sending timestamp
  //reset everything to zero
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TCNT1 = 0;

  OCR1A = 39062;      //OCR value to count to. found by 8Mhz/1024(prescaler value)/0.2Hz

  TCCR1B |= (1 << WGM12);   //Choosing CTC Mode, where MAX value = OCR1A
  TCCR1B |= (1 << CS12) | (1 << CS10);   // sets up Prescaler to 1024. 

  TIMSK1 |= (1 << OCIE1A);    //enable output compare interrupt A
  

}

ISR(TIMER3_CAPT_vect) {       //input interrupt ISR
  char *eptr;
  //will synchronise times ?
   if (((char*)buf)[0] == '1') {
    // the gateway has just turned on, do nothing
    //Serial.println("This is a test");
  }
  else if (((char*)buf)[0] == '2') {
    TCNT1 = strtoul((char*)buf+1, &eptr, 10);
    cycles = 0;
    synchronise = true;     //gateway has started the synchronisation procedure
  }
  else if (((char*)buf)[0] == '3') {
    synchronise = false;    //the timings have synchronised
  }
  
  PORTC = 0;
}

ISR(TIMER3_OVF_vect) {        //TIMER3 overflow ISR
  cycles++;
}

ISR(TIMER1_COMPA_vect) {      //TIMER1 output compare ISR 
  //Serial.println("OCR Value reached");
  //set send time_stamp on true, handled by main loop
  send_timestamp = true;
}


void loop() {
  //sleep for a while, ideally
  char *e;
  char data[RH_RF95_MAX_MESSAGE_LEN];
  
  if (rf95.available()) {   //message available OR synchronizing
    
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      //flag ISR
      PORTC |= (1 << PC7);
      //RH_RF95::printBuffer("Received: ", buf, len);
      
      Serial.print(F("Got in string: "));
      Serial.println((char*)buf);
      
      Serial.println(((char*)buf)[0]);
      Serial.print("Got in long:   ");
      unsigned long x = strtoul((char*)buf, &e, 10);
      Serial.println(x);

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
  }
  else if (send_timestamp) {              //no message received, send the signal containing information from node
      char message_time[100];
      unsigned long transmit_time = (cycles * 0xFFFF) + TCNT3;
      //Serial.print("transmit time as long:   ");
      //Serial.println(transmit_time);
      strcpy(data,"4");
      sprintf(message_time,"%lu", transmit_time);
      //Serial.print("transmit time as string: ");
      //Serial.println(message_time);
      strncat(data,message_time, sizeof(message_time));
      //Serial.print("Sending time_stamp:   ");
      //Serial.println(data);
      rf95.send(data,sizeof(data));
      rf95.waitPacketSent();
      
      send_timestamp = false;
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

/*;
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
*/

void determineMessageType(char indicator) {
  if (indicator == '1') {
    // the gateway has just turned on, do nothing
    //Serial.println("This is a test");
  }
  else if (indicator == '2') {
    cycles = 0;
    synchronise = true;     //gateway has started the synchronisation procedure
  }
  else if (indicator == '3') {
    synchronise = false;    //the timings have synchronised
  }
}
