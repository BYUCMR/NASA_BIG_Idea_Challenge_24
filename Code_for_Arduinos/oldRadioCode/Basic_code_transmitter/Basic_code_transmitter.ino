/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001"; //Next we need to create a byte array which will 
//represent the address, or the so called pipe through which the two modules will communicate.
/*We can change the value of this address to any 5 letter string and this enables to choose to 
which receiver we will talk, so in our case we will have the same address at both the receiver 
and the transmitter.*/
void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN); //This sets the power level at which the module will transmit. 
                                //The level is super low now because the two modules are very close to each other.
  radio.stopListening();
}

void loop() {
  const char text[] = "Hello World";
  radio.write(&text, sizeof(text));
  delay(1000);
}