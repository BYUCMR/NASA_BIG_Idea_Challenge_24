
/*
 * Preliminary code for the Middle Node module of the soft Robotics Project
 * Soft Robotics For the Moon!
 * Run by doctor Nathan Usevitch, Assitant Professor at Brigham Young University.
 * Code Written by Christopher Paul for both ME497r and for the NASA Big Idea Challenge Project.
 * Libraries: TMRh20/RF24, https://github.com/tmrh20/RF24/
 *            TimerInterrupt, https://github.com/khoih-prog/TimerInterrupt?tab=readme-ov-file#important-notes-about-isr
 */
//--------------Timer Library setup--------------------//
// The link to the repository is as follows: https://github.com/khoih-prog/TimerInterrupt?tab=readme-ov-file#important-notes-about-isr
#define USE_TIMER_1     true
#define USE_TIMER_2     false
#warning Using Timer1, Timer2
#include "TimerInterrupt.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#define LED_PIN_RED 19   // red LED, use to indicate receiving.
#define LED_PIN_GREEN 18 // green LED, use to indicate transmitting.
#define MOTOR_ERROR 6 // pin for reading error state from the half bridge motor driver. If it goes low, something is wrong.
#define MOTOR_SLEEP 4 // pin for controlling the sleep state of the half bridge motor driver. LOW is sleep, HIGH is awake.
#define FAST_BLINK 100  // miliseconds
#define SLOW_BLINK 1000 // miliseconds
RF24 radio(7, 8);       // CE, CSN
//-----------Motor Control Setup----------------//
#include "DCMotorControl.h"
DCMotorControl Motors[] = { //There is only one motor that each will be controlling. There is no need for multiple motors.
  //DCMotorControl(9, 10, 5, 2, 3), //DCMotorControl::DCMotorControl( uint8_t DirectionPinA, uint8_t DirectionPinB, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) //uses this constructor first!!
  DCMotorControl(10, 5, 3, 2) //DCMotorControl::DCMotorControl( uint8_t DirectionPin, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) //uses this constructor second to illustrate the point.
};
#define NumberOfMotors 1
#define ControlRate_ms 10
#define ControlRate_us 10000
#define TIMER_INTERVAL_MS 10L // 10ms, or 10,000us as specfified by the ControlRate_us variable in the DCMotorControl.h file.
#define DeadbandTicks 100
#define DeadbandDutyCycle 0
#define TicksPerInch ((50 * 64) / (3.14159265359 * 0.713))
#define TicksPerRevolution (50 * 64)
#define HomingSpeedTolerance 0.01
#define MinimumPWM 0
#define Kp 0.01
#define Ki 0.003
#define Kd 0.001
#define DutyCycleStall 25
#define MaxDutyCycleDelta 5
float DutyCycle = 0.0;
float CurrentRPM = 0.0;
int MotorDrive[NumberOfMotors] = {0};
bool MotorEnabled = false;
float DesiredPosition[NumberOfMotors] = {0};
int LastTicks = 0;
int CurrentTicks = 0;
int LastTimeMillis = 0;
int CurrentTimeMillis = 0;
uint8_t HomingMotor = 0;
int counter = 0;
bool motor_running = false;
enum child_state
{
  RECEIVING_1,
  RECEIVING_2,
  TRANSMITTING,
  OFF,
  COMPLETED,
  RUN_MOTOR
} child_state;
int global_received_data[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
int transmit_count = 0;
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
//This node is 00005, the end node, receives from 00004, and is the end of the line.
const byte addresses[][6] = {"00001", "00002", "00003", "00004","00005"};
auto self = addresses[4];
auto parent = addresses[3];
// -------------------- FUNCTIONS ------------------- //
void blink_led_unblocking(int delay_time)
{
  static unsigned long past_time = millis();
  static bool led_ON = false;
  if (millis() - past_time > delay_time)
  {
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
  //  2. 1000 ms have passed so we go back to transmitting.
  if (millis() - past_time_r > 1000)
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
      child_state = RUN_MOTOR;
      digitalWrite(LED_PIN_RED, LOW);
      digitalWrite(LED_PIN_GREEN, LOW);
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
  radio.openWritingPipe(parent);    
  radio.openReadingPipe(0, self); // 00003 the address of the end node. (THIS MODULE)
  radio.setPALevel(RF24_PA_MIN);          // This sets the power level at which the module will transmit.
                                 // The level is super low now because the two modules are very close to each other.
  child_state = RECEIVING_1;
  radio.startListening();
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  digitalWrite(LED_PIN_RED, LOW);
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
  case RUN_MOTOR:
    Serial.println("RUN_MOTOR");
    for (int i = 0; i < 4; i++)
    {
      digitalWrite(MOTOR1_DIR_PIN_A, HIGH);
      digitalWrite(MOTOR1_DIR_PIN_B, LOW);
      analogWrite(MOTOR1_STEP_PIN, 255);
      //Wait 3 seconds
      delay(3000);
      //STOP MOTORS
      analogWrite(MOTOR1_STEP_PIN, 0);
      delay(2000);
      //Set motor direction counter clockwise
      digitalWrite(MOTOR1_DIR_PIN_A, LOW);
      digitalWrite(MOTOR1_DIR_PIN_B, HIGH);
      analogWrite(MOTOR1_STEP_PIN, 255);
      delay(3000);
      //STOP MOTORS
      analogWrite(MOTOR1_STEP_PIN, 0);
      delay(2000);    
    }
    digitalWrite(LED_PIN_WHITE, HIGH);
    digitalWrite(LED_PIN_GREEN, HIGH);
    child_state = COMPLETED;
  
    break;
  default:
    Serial.println("ERROR: transmitter_state is in an unknown state");
    break;
  }
}
