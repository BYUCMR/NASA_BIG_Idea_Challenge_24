/*
 * Preliminary code for the Middle Node module of the soft Robotics Project
 * Soft Robotics For the Moon!
 * Run by doctor Nathan Usevitch, Assitant Professor at Brigham Young University.
 * Code Written by Christopher Paul for ME 497r at BYU. November 2023.
 * Libraries: TMRh20/RF24, https://github.com/tmrh20/RF24/
 */
// This code will include an embedded state machine with a
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
const int compare_data[4][2] = {{-7, -7}, {14, 0}, {-14, 0}, {7, 7}};
int received_data[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
// this node is 00002, the master node or start node.
const byte addresses[][6] = {"00001", "00002", "00003"}; // receive signals from 00001, send signals to 00001 and 00003
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
void Parent_TX_1_function_unblocking()
{
  radio.stopListening();
  digitalWrite(LED_PIN_GREEN, HIGH);
  static int transmit_count_1 = 0;
  // static unsigned long past_time2 = millis();
  // blink_led_unblocking(FAST_BLINK);
  const int transmit_data[4][2] = {{-7, -7}, {14, 0}, {-14, 0}, {7, 7}};
  radio.write(&transmit_data, sizeof(transmit_data));
  parent_state = RECEIVING;
  Serial.println("TRANSMITTING DATA");
  transmit_count_1++;
  radio.startListening();
  digitalWrite(LED_PIN_GREEN, LOW);
}
void Parent_TX_2_COMPLETED()
{
  radio.stopListening();
  digitalWrite(LED_PIN_GREEN, HIGH);
  static int transmit_count_2 = 0;
  // make the array all 1's
  const int transmit_data[4][2] = {{1, 1}, {1, 1}, {1, 1}, {1, 1}};
  radio.write(&transmit_data, sizeof(transmit_data));
  parent_state = COMPLETED_1;
  Serial.println("TRANSMITTING COMPLETED DATA");
  transmit_count_2++;
  digitalWrite(LED_PIN_RED, HIGH);
  digitalWrite(LED_PIN_GREEN, HIGH);
}
void Parent_RX_func()
{
  Serial.println("RECEIVING");
  // the radio should already be in listening mode.
  static unsigned long past_time_r = millis();
  blink_led_unblocking(SLOW_BLINK);
  // two paths out of receiving:
  //  1. We receive the data back.
  //  2. 2000 ms have passed so we go back to transmitting.
  if (millis() - past_time_r > 2000)
  {
    parent_state = TRANSMITTING_1;
    past_time_r = millis();
  }
  else if (radio.available())
  {
    int received_data[4][2];
    radio.read(&received_data, sizeof(received_data));
    bool matches_data = true;
    // NEED TO WRITE A PRINTING FUNCTION that can also compare the arrays.
    // Serial.println(recieved_data);
    for (int x = 0; x < 4; x++)
    {
      for (int y = 0; y < 2; y++)
      {
        Serial.print(received_data[x][y]);
        Serial.print(" ");
        if (received_data[x][y] != compare_data[x][y])
        {
          matches_data = false;
        }
      }
      Serial.println();
    }
    if (matches_data)
    {
      Serial.println("MATCHES");
      parent_state = TRANSMITTING_2;
    }
    else
    {
      Serial.println("DOES NOT MATCH");
      parent_state = TRANSMITTING_1; // something seems off here.
    }
  }
}
void Child_TX_function()
{
  radio.stopListening();
  digitalWrite(LED_PIN_GREEN, HIGH);
  static int transmit_count_3 = 0;
  // static unsigned long past_time2 = millis();
  // blink_led_unblocking(FAST_BLINK);
  // const int transmit_data[4][2] = {{-7, -7},{14, 0},{-14, 0},{7, 7}};
  radio.write(&received_data, sizeof(received_data));
  child_state = RECEIVING_2;
  Serial.println("TRANSMITTING DATA");
  transmit_count_3++;
  radio.startListening();
  digitalWrite(LED_PIN_GREEN, LOW);
}
// checks for array of all ones.
void Child_RX_2()
{
  Serial.println("RECEIVING_2");
  // the radio should already be in listening mode.
  static unsigned long past_time_r = millis();
  blink_led_unblocking(SLOW_BLINK);
  // two paths out of receiving:
  //  1. We receive the data back.
  //  2. 2000 ms have passed so we go back to transmitting.
  if (millis() - past_time_r > 2000)
  {
    child_state = TRANSMITTING;
    past_time_r = millis();
  }
  else if (radio.available())
  {
    int received_data2[4][2];
    radio.read(&received_data2, sizeof(received_data2));
    bool data_correct = true;
    // NEED TO WRITE A PRINTING FUNCTION that can also compare the arrays.
    // Serial.println(recieved_data);
    for (int x = 0; x < 4; x++)
    {
      for (int y = 0; y < 2; y++)
      {
        Serial.print(received_data2[x][y]);
        Serial.print(" ");
        if (received_data2[x][y] != 1)
        {
          data_correct = false;
        }
      }
      Serial.println();
    }
    if (data_correct)
    {
      Serial.println("MATCHES");
      child_state = TRANS_TO_PARENT;
      digitalWrite(LED_PIN_RED, HIGH);
      digitalWrite(LED_PIN_GREEN, HIGH);
    }
    else
    {
      Serial.println("DOES NOT MATCH");
      child_state = TRANSMITTING; // something seems off here.
    }
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("STARTING");
  radio.begin();
  radio.openWritingPipe(addresses[0]);    // 00001 the address of the receiver. (THIS MODULE)
  radio.openReadingPipe(1, addresses[1]); // 00001 the address of the transmitter
  radio.setPALevel(RF24_PA_MIN);          // This sets the power level at which the module will transmit.
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
  case PARENT:
    switch (parent_state)
    {
    case RECEIVING: // this one will be run muliple times.
      Parent_RX_func();
      break;

    case TRANSMITTING_1:
      // Serial.println(transmit_count);
      Parent_TX_1_function_unblocking();
      break;

    case TRANSMITTING_2:
      Serial.println("TRANSMITTING_2");
      Parent_TX_2_COMPLETED();
      break;

    case OFF:
      delay(1000);
      Serial.println("OFF");
      // blink_led(500);
      parent_state = RECEIVING;
      // past_time = millis();
      radio.startListening();
      break;

    case COMPLETED_1:
      // blink_led_unblocking(5000);
      Serial.println("COMPLETED");
      break;
    }
    break;
  case CHILD:
    switch (child_state)
    {
    case RECEIVING_1: // this one will be run muliple timees.
      Serial.println("RECEIVING_1");
      blink_led_unblocking(SLOW_BLINK);
      if (radio.available())
      {
        radio.read(&received_data, sizeof(received_data));
        // iterate through values and print data
        for (int x = 0; x < 4; x++)
        {
          for (int y = 0; y < 2; y++)
          {
            Serial.print(received_data[x][y]);
            Serial.print(" ");
          }
          Serial.println();
        }
        child_state = TRANSMITTING;
        // radio.stopListening();
      }
      break;
    case RECEIVING_2: // this is the one waiting for if the sent data was correct.
      // if the data was correctly received, the tx will send back an array of all ones.
      // Serial.println("RECEIVING_2");
      Child_RX_2();

      break;

    case TRANSMITTING: // stay here for a few times at least before i implement the second bounce back.
      Child_TX_function();

      break;

    case OFF:
      Serial.println("CHILD STATE_OFF");

      break;

    case TRANS_TO_PARENT:
      // link_led_unblocking(5000);

      Serial.println("TRANSITIONING TO PARENT MODE");
      overall_state = PARENT;
      parent_state = TRANSMITTING_1;
      child_state = OFF;
      radio.stopListening();
      radio.openWritingPipe(addresses[2]); 
      break;

    break;
  }
}
