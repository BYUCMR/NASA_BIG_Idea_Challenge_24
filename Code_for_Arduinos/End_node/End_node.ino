
/*
 * Arduino Wireless Communication Tutorial
 *     Example 1 - Transmitter Code
 *
 * by Dejan Nedelkovski, www.HowToMechatronics.com
 *
 *
 * Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define LED_PIN_WHITE 2 // White LED, use to indicate RECEIVING_1.
#define LED_PIN_GREEN 3 // Green LED, use to indicate transmitting.
#define FAST_BLINK 100  // miliseconds
#define SLOW_BLINK 1000 // miliseconds
RF24 radio(7, 8);       // CE, CSN
enum child_state
{
  RECEIVING_1,
  RECEIVING_2,
  TRANSMITTING,
  OFF,
  COMPLETED
} child_state;
int global_received_data[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
int transmit_count = 0;
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
const byte addresses[][6] = {"00001", "00002", "00003"};
// -------------------- FUNCTIONS ------------------- //
void blink_led_unblocking(int delay_time)
{
  static unsigned long past_time = millis();
  static bool led_ON = false;
  if (millis() - past_time > delay_time)
  {
    if (led_ON)
    {
      digitalWrite(LED_PIN_WHITE, LOW);
      led_ON = false;
    }
    else
    {
      digitalWrite(LED_PIN_WHITE, HIGH);
      led_ON = true;
    }
    past_time = millis();
  }
}

void Child_TX_function()
{
  radio.stopListening();
  digitalWrite(LED_PIN_GREEN, HIGH);
  static int transmit_count = 0;
  // static unsigned long past_time2 = millis();
  // blink_led_unblocking(FAST_BLINK);
  // const int transmit_data[4][2] = {{-7, -7},{14, 0},{-14, 0},{7, 7}};
  radio.write(&global_received_data, sizeof(global_received_data));
  child_state = RECEIVING_2;
  Serial.println("TRANSMITTING DATA");
  transmit_count++;
  radio.startListening();
  digitalWrite(LED_PIN_GREEN, LOW);
}

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
      child_state = COMPLETED;
      digitalWrite(LED_PIN_WHITE, HIGH);
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
  radio.openWritingPipe(addresses[1]);    // 00002 the address of the middle node.
  radio.openReadingPipe(0, addresses[2]); // 00003 the address of the end node. (THIS MODULE)
  radio.setPALevel(RF24_PA_MIN);          // This sets the power level at which the module will transmit.
                                 // The level is super low now because the two modules are very close to each other.
  child_state = RECEIVING_1;
  radio.startListening();
  pinMode(LED_PIN_WHITE, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  digitalWrite(LED_PIN_WHITE, LOW);
  digitalWrite(LED_PIN_GREEN, LOW);
}

void loop()
{
  switch (child_state)
  {
  case RECEIVING_1: // this one will be run muliple timees.
    Serial.println("RECEIVING_1");
    blink_led_unblocking(SLOW_BLINK);
    if (radio.available())
    {
      radio.read(&global_received_data, sizeof(global_received_data));
      // iterate through values and print data
      for (int x = 0; x < 4; x++)
      {
        for (int y = 0; y < 2; y++)
        {
          Serial.print(global_received_data[x][y]);
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
    Serial.println("OFF");
    // blink_led(500);
    // transmitter_state = TRANSMITTING;
    /// radio.stopListening();

    break;

  case COMPLETED:
    // link_led_unblocking(5000);
    Serial.println("COMPLETED");
    break;

  default:
    Serial.println("ERROR: transmitter_state is in an unknown state");
    break;
  }
}
