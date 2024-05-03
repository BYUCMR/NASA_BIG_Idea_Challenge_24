
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
#define LED_PIN_RED 2
#define FAST_BLINK 100 //miliseconds
#define SLOW_BLINK 1000 //miliseconds
RF24 radio(7, 8); // CE, CSN
enum radio_state
  {
    RECEIVING,
    TRANSMITTING,
    OFF,
    COMPLETED
  }transmitter_state;
char received_text[32] = "";
int received_data[4][2] = {{0, 0},{0, 0},{0, 0},{0, 0}};
int transmit_count = 0;
/*Next we need to create a byte array which will 
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to 
which receiver we will talk, so in our case we will have the same address at both the receiver 
and the transmitter.*/
const byte addresses[][6] = {"00001", "00002"}; 
// -------------------- FUNCTIONS ------------------- //
void blink_led(int delay_time) {
  digitalWrite(LED_PIN_RED, HIGH);
  delay(delay_time);
  digitalWrite(LED_PIN_RED, LOW);
}

void blink_led_unblocking(int delay_time){
  static unsigned long past_time = millis();
  static bool led_ON = false;
  if(millis() - past_time > delay_time ){
    if(led_ON){
      digitalWrite(LED_PIN_RED, LOW);
      led_ON = false;
    }
    else{
      digitalWrite(LED_PIN_RED, HIGH);
      led_ON = true;
    }
    past_time = millis();
  }
}

void transmitter_function_unblocking(){
  static int transmit_count = 0;
  static unsigned long past_time2 = millis();
  blink_led_unblocking(FAST_BLINK);
  if(transmit_count < 3 ){
    if(millis() - past_time2 > 1000){
      Serial.println("TRANSMITTING DATA");
      //const int transmit_data[4][2] = {{-7, -7},{14, 0},{-14, 0},{7, 7}};
      //const char transmit_text[] = "Hello World";
      radio.write(&received_data, sizeof(received_data));
      transmit_count++;
      past_time2 = millis();
    }
  }
  else{
    transmitter_state = COMPLETED;
    //Serial.println("To Off");
    transmit_count = 0;
    digitalWrite(LED_PIN_RED, LOW);
  }
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
  pinMode(LED_PIN_RED, OUTPUT);
  digitalWrite(LED_PIN_RED, LOW);
}

void loop() {
    switch (transmitter_state)
    {
      case RECEIVING: //this one will be run muliple timees.
      Serial.println("RECEIVING");
      blink_led_unblocking (SLOW_BLINK);
      if (radio.available()){
        radio.read(&received_data, sizeof(received_data));
        //iterate through values and print data
        for(int x = 0; x < 4; x++){
        for(int y = 0; y < 2; y++){
          Serial.print(received_data[x][y]);
          Serial.print(" ");
        }
        Serial.println();
      }
        transmitter_state = OFF;
        radio.stopListening();
      }
      break;

      case TRANSMITTING: //stay here for a few times at least before i implement the second bounce back.
      transmitter_function_unblocking();
      
      break;

      case OFF:
      Serial.println("OFF");
      //blink_led(500);
      transmitter_state = TRANSMITTING;
      ///radio.stopListening();
      
      break;

      case COMPLETED:
      blink_led_unblocking(5000);
      Serial.println("COMPLETED");
      break;

      default:
      Serial.println("ERROR: transmitter_state is in an unknown state");
      break;
    }
}
