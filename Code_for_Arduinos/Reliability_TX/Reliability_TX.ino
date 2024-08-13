/*
 * Preliminary code for the Start Node module of the soft Robotics Project
 * Soft Robotics For the Moon!
 * Run by doctor Nathan Usevitch, Assitant Professor at Brigham Young University.
 * Code Written by Christopher Paul for ME 497r at BYU. November 2023.
 * Libraries: TMRh20/RF24, https://github.com/tmrh20/RF24/
 */

#include <SPI.h> //included by default for every arduino.
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#define LED_PIN_RED 2   // red LED, use to indicate receiving.
#define LED_PIN_GREEN 3 // green LED, use to indicate transmitting.
#define FAST_BLINK 100  // miliseconds
#define SLOW_BLINK 1000 // miliseconds
RF24 radio(7, 8);       // CE, CSN
// -------------------- VARIABLES ------------------- //
enum Parent_state
{
  RECEIVING,
  TRANSMITTING_1,
  TRANSMITTING_2,
  OFF,
  COMPLETED
} parent_state;
// data to compare the received data back against to see if it matches.
const int transmit_data2[16] = {5, 23, -1, 0, -17, 64, 3, 156, -233, 0, 0, 12345, -65000, 0, 7, 7}; //maximum length of the arrays we can transmit at once. 
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
// this node is 00001, the master node or start node, has long range atennae. This one will start off the communication.
const byte addresses[][6] = {"00001", "00002", "00003", "00004", "00005"}; 
auto self = addresses[0];
auto child = addresses[1];
// -------------------- FUNCTIONS ------------------- //
// checks for whether the delay_time has passed and sets the LED on or off.
void Parent_TX_1_function_unblocking()
{
  static bool successful;
  static unsigned long past_time2 = millis();
  static unsigned short wait_time_ms = 10;
  if(millis() - past_time2 > wait_time_ms){
    digitalWrite(LED_PIN_GREEN, HIGH);
    past_time2 = millis();
    successful = radio.write(&transmit_data2, sizeof(transmit_data2));
    if(successful){
      digitalWrite(LED_PIN_RED, LOW);
      wait_time_ms = 10;
      for(int i = 0; i < 16; i++){ //print out the sent array values.
        Serial.print(transmit_data2[i]);
        Serial.print(" ");
      }
      Serial.println();
      successful = false; // reset the successful flag.
    }
    else{
      digitalWrite(LED_PIN_RED, HIGH);
      // transmitting failed, now lets change the checking time.
      wait_time_ms = 1;
    }
    digitalWrite(LED_PIN_GREEN, LOW);
  }
  
}

void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(child);
  radio.openReadingPipe(1, self);
  radio.setPALevel(RF24_PA_MAX);     // This sets the power level at which the module will transmit.
  parent_state = TRANSMITTING_1;
  radio.stopListening();
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  digitalWrite(LED_PIN_RED, LOW);   // LED is off.
  digitalWrite(LED_PIN_GREEN, LOW); // LED is off.
}

void loop()
{
  switch (parent_state)
  {
  case TRANSMITTING_1:
    Parent_TX_1_function_unblocking();
    break;
  }
}
