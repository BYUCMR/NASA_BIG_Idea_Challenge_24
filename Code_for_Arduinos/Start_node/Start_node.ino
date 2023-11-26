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
const int transmit_data[4][2] = {{-7, -7}, {14, 0}, {-14, 0}, {7, 7}};
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
// this node is 00001, the master node or start node.
const byte addresses[][6] = {"00001", "00002", "00003", "00004", "00005"}; 
// -------------------- FUNCTIONS ------------------- //
// checks for whether the delay_time has passed and sets the LED on or off.
void blink_led_unblocking(int delay_time)
{
  static unsigned long past_time = millis();
  static bool led_ON = false;
  if (millis() - past_time > delay_time)
  {
    Serial.println("blinking");
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
  static int transmit_count = 0;
  // static unsigned long past_time2 = millis();
  // blink_led_unblocking(FAST_BLINK);
  radio.write(&transmit_data, sizeof(transmit_data));
  parent_state = RECEIVING;
  Serial.println("TRANSMITTING DATA");
  transmit_count++;
  radio.startListening();
  digitalWrite(LED_PIN_GREEN, LOW);
}
void Parent_TX_2_COMPLETED()
{
  radio.stopListening();
  digitalWrite(LED_PIN_GREEN, HIGH);
  static int transmit_count = 0;
  // make the array all 1's
  const int complete_data_array[4][2] = {{1, 1}, {1, 1}, {1, 1}, {1, 1}};
  radio.write(&complete_data_array, sizeof(complete_data_array));
  parent_state = COMPLETED;
  Serial.println("TRANSMITTING COMPLETED DATA");
  transmit_count++;
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
    for (int x = 0; x < 4; x++)
    {
      for (int y = 0; y < 2; y++)
      {
        Serial.print(received_data[x][y]);
        Serial.print(" ");
        if (received_data[x][y] != transmit_data[x][y])
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

void setup()
{
  Serial.begin(9600);
  Serial.println("STARTING");
  radio.begin();
  radio.openWritingPipe(addresses[1]);    // 00002 the address of the receiver.
  radio.openReadingPipe(1, addresses[0]); // 00001 the address of the transmitter (THIS MODULE)
  radio.setPALevel(RF24_PA_MIN);          // This sets the power level at which the module will transmit.
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
  case RECEIVING: // this one will be run muliple times.
    Parent_RX_func();
    break;

  case TRANSMITTING_1:
    Parent_TX_1_function_unblocking();
    break;

  case TRANSMITTING_2:
    Serial.println("TRANSMITTING_2");
    Parent_TX_2_COMPLETED();
    break;

  case OFF:
    Serial.println("OFF");
    break;

  case COMPLETED:
    Serial.println("COMPLETED");
    break;
  }
}
