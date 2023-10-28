/*
 * Arduino Wireless Communication Tutorial
 *       Example 1 - Receiver Code
 *
 * by Dejan Nedelkovski, www.HowToMechatronics.com
 *
 * Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";
enum radio__
  {
    LISTENING,
    OFF
  }radio__;
void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  pinMode(2, OUTPUT); // led to show that we are receiving.
  digitalWrite(2, LOW);
  radio__ = LISTENING;
}

void loop()
{
  

  switch (radio__)
  {
  case LISTENING:
    if (radio.available())
    {
      char text[32] = "";
      radio.read(&text, sizeof(text));
      // Serial.println(radio.getPayloadSize());
      Serial.println(text);
      // delay(2000);
      if (strncmp(text, "Hello World", 11) == 0)
      {
        digitalWrite(2, HIGH);
        radio__ = OFF;
      }
    }
    break;

  case OFF:
    // code for when radio is off
    break;

  default:
    // code for when radio state is unknown
    break;
  }
  
}