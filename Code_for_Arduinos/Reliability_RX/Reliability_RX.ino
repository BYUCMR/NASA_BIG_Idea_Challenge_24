/*
 * Preliminary code for the Middle Node module of the soft Robotics Project
 * Soft Robotics For the Moon!
 * Run by doctor Nathan Usevitch, Assitant Professor at Brigham Young University.
 * Code Written by Christopher Paul for ME 497r at BYU. November 2023.
 * Libraries: TMRh20/RF24, https://github.com/tmrh20/RF24/
 */
// This code will include an embedded state machine with a state for parent and child statemachines.
#include <SPI.h> //included by default for every arduino.
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#define LED_PIN_RED 19   // red LED, use to indicate receiving.
#define LED_PIN_GREEN 18 // green LED, use to indicate transmitting.
#define FAST_BLINK 100  // miliseconds
#define SLOW_BLINK 1000 // miliseconds
RF24 radio(7, 8);       // CE, CSN

// -------------------- VARIABLES ------------------- //
enum overall_state
{
  PARENT,
  CHILD
} overall_state;
enum parent_state
{
  RECEIVING,
  TRANSMITTING_1,
  TRANSMITTING_2,
  OFF_P,
  COMPLETED_1
} parent_state;
enum child_state
{
  RECEIVING_1,
  RECEIVING_2,
  TRANSMITTING,
  OFF,
  TRANS_TO_PARENT
} child_state;

// data to compare the received data back against to see if it matches.
int global_received_data2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //maximum length of the arrays we can transmit at once.
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
// this node is 00002, receives from 00001, sends array to 00003 and 00004.
const byte addresses[][6] = {"00001", "00002", "00003", "00004","00005"}; 
auto self = addresses[1];
auto parent = addresses[0];
auto child1 = addresses[2];
auto child2 = addresses[3];
unsigned short num_children = 2; //the number of children left for this node to send data to.
// -------------------- FUNCTIONS ------------------- //

// checks for whether the delay_time has passed and sets the LED on or off.
void blink_led_unblocking(int delay_time)
{
  static unsigned long past_time = millis();
  static bool led_ON = false;
  if (millis() - past_time > delay_time)
  {
    // Serial.println("blinking");
    // the time to blink has come.
    if (led_ON)
    {
      digitalWrite(LED_PIN_RED, LOW);
      led_ON = false;
    }
    else
    {
      digitalWrite(LED_PIN_RED, HIGH);
      led_ON = true;
    }
    past_time = millis();
  }
}

void RX_1_Func(void)
{
  static unsigned long past_time2 = millis();
  static unsigned short wait_time_ms = 10;
  if (millis() - past_time2 > wait_time_ms)
  {
    if (radio.available())
    {
      digitalWrite(LED_PIN_GREEN, HIGH);
      wait_time_ms = 10;
      radio.read(&global_received_data2, sizeof(global_received_data2));
      // iterate through values and print data
      for (int i = 0; i < 16; i++)
      {
        Serial.print(global_received_data2[i]);
        Serial.print(" ");
      }
      Serial.println();
      digitalWrite(LED_PIN_GREEN, LOW);
    }
    else{
      wait_time_ms =1;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(parent);    // 00001 the address of node 1, or the start node.
  radio.openReadingPipe(1, self); // 00002 the address of node 2, or the middle node. (THIS MODULE)
  radio.setPALevel(RF24_PA_MAX);          // This sets the power level at which the module will transmit.
                                          // The level is super low now because the two modules are very close to each other.
  overall_state = CHILD;
  child_state = RECEIVING_1;
  radio.startListening();
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  digitalWrite(LED_PIN_RED, LOW);
  digitalWrite(LED_PIN_GREEN, LOW);
}

void loop()
{
  switch (overall_state)
  {
  case CHILD:
    switch (child_state)
    {
    case RECEIVING_1: // this one will be run muliple times.
      blink_led_unblocking(SLOW_BLINK);
      RX_1_Func();
      break;

      break;
    }
  }
}
