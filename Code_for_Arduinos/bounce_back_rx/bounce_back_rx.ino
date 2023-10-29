
/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* Adapted and modified by: Chris Paul for ME 497r at BYU.
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define LED_PIN 2
RF24 radio(7, 8); // CE, CSN
enum radio_state
  {
    RECEIVING,
    TRANSMITTING,
    OFF,
    COMPLETED
  }transmitter_state;
char received_text[32] = "";
/*Next we need to create a byte array which will 
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to 
which receiver we will talk, so in our case we will have the same address at both the receiver 
and the transmitter.*/
const byte addresses[][6] = {"00001", "00002"}; 
// -------------------- FUNCTIONS ------------------- //
void blink_led(int delay_time) {
  digitalWrite(LED_PIN, HIGH);
  delay(delay_time);
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  Serial.begin(9600);
  Serial.println("STARTING");
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002 the address of the receiver. (THIS MODULE)
  radio.openReadingPipe(0, addresses[1]); // 00001 the address of the transmitter 
  radio.setPALevel(RF24_PA_MIN); //This sets the power level at which the module will transmit. 
                                //The level is super low now because the two modules are very close to each other.
  transmitter_state = RECEIVING;
  radio.startListening();
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
    switch (transmitter_state)
    {
      case RECEIVING: //this one will be run muliple timees.
      Serial.println("RECEIVING");
      if (radio.available()){
        radio.read(&received_text, sizeof(received_text));
        Serial.println(received_text);
        transmitter_state = OFF;
      }
      break;

      case TRANSMITTING: //stay here for a few times at least before i implement the second bounce back. 
      for(int i = 0; i<10; i++){
        delay(1000);
        radio.write(&received_text, sizeof(received_text));
        blink_led(500);
      }
      transmitter_state = COMPLETED;
      break;

      case OFF:
      blink_led(500);
      transmitter_state = TRANSMITTING;
      radio.stopListening();
      break;

      case COMPLETED:
      blink_led(500);
      Serial.println("COMPLETED");
      break;

      default:
      Serial.println("ERROR: transmitter_state is in an unknown state");
      break;
    }
}
