//Timer Library setup
#include "TimerInterrupt.h"
// The link to the repository is as follows: https://github.com/khoih-prog/TimerInterrupt?tab=readme-ov-file#important-notes-about-isr
// NOTES ABOUT ISR:
// Select the timers you're using, here ITimer1
#define USE_TIMER_1     true
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false
TimerInterrupt ITimer1(1);
// Init timer ITimer1

void TimerHandler();
IntervalTimer ControlTimer;
IntervalTimer RadioResetTimer;

//Radio Communication Setup
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(15,14); // CE, CSN

typedef enum {  RC_Zero, RC_SetDesiredPositionAll, RC_ResetRadio, RC_SetCurrentPosition, RC_ReadCurrentPosition, RC_SetDesiredPosition} RadioCommands_t;
const uint64_t pipes[25] = { 0xF0F0F0F0CDLL, 0xF0F0F0F061LL, 0xF0F0F0F081LL, 0xF0F0F0F0A1LL, 0xF0F0F0F0C1LL,
                              0xF0F0F0F025LL, 0xF0F0F0F065LL, 0xF0F0F0F085LL, 0xF0F0F0F0A5LL, 0xF0F0F0F0C5LL,
                              0xF0F0F0F029LL, 0xF0F0F0F069LL, 0xF0F0F0F089LL, 0xF0F0F0F0A9LL, 0xF0F0F0F0C9LL,
                              0xF0F0F0F02BLL, 0xF0F0F0F06BLL, 0xF0F0F0F08BLL, 0xF0F0F0F0ABLL, 0xF0F0F0F0CBLL,
                              0xF0F0F0F02DLL, 0xF0F0F0F06DLL, 0xF0F0F0F08DLL, 0xF0F0F0F0ADLL, 0xF0F0F0F0CFLL,};

#define NodeID 0
#define RF_Channel 2
#define RadioResetRate_us 2000000

// Motor Control Setup
#include "DCMotorControl.h"
DCMotorControl Motors[] = {
    DCMotorControl(16, 17, 5, 2, 3), // Motor 0 DCMotorControl( uint8_t DirectionPin, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin)

};
#define NumberOfMotors (sizeof(Motors) / sizeof(Motors[0]))
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

// IntervalTimer ControlTimer;
// IntervalTimer RadioResetTimer;
void ControllerISR(void);
// Debug Setup

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting!!");
  radio.begin();
  radio.openReadingPipe(0, pipes[NodeID]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(RF_Channel);
  radio.startListening();
  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");
  for (uint8_t i = 0; i < NumberOfMotors; i++)
  {
    Motors[i].setParameters(Kp, Ki, Kd, ControlRate_us, DeadbandTicks, DeadbandDutyCycle, TicksPerInch, TicksPerRevolution, MinimumPWM);
    Motors[i].setDutyCycleStall(DutyCycleStall);
    Motors[i].setMaxDutyCycleDelta(MaxDutyCycleDelta);
  }

  for (uint8_t i = 0; i < (NumberOfMotors); i++)
  {
    MotorEnabled = true;
    Motors[i].setMotorEnable(MotorEnabled);
    Motors[i].setMode(DC_Automatic);
  }

#ifdef DEBUG
  // Serial.begin(9600);
#endif

  ControlTimer.begin( ControllerISR , ControlRate_us ); // attach the service routine here
  RadioResetTimer.begin( RadioResetISR , RadioResetRate_us ); // attach the service routine here
}
void loop()
{
  // make it so that the motor moves back and forth between 3 and -3 inches
  // but also make it so that motor[0].run() is called every 10ms
  unblocking_timer();
  if(Motors[0].getCurrentPositionInches() > 3){
    Motors[0].setDesiredPositionInches(-3);
  }
  else if(Motors[0].getCurrentPositionInches() < -3){
    Motors[0].setDesiredPositionInches(3);
  }
  else{
    Motors[0].setDesiredPositionInches(3);
  }
  

}

void unblocking_timer(){
  static unsigned long past_time = millis();
  if(millis() - past_time >10){
    past_time = millis();
    Motors[0].run();
  }
}

void RadioResponse(void)
{
  long Ticks[8] = {0};
  // RadioResetTimer.begin( RadioResetISR , RadioResetRate_us ); // attach the service routine here
  Serial.println("Message Received");
  radio.read(&Ticks, sizeof(Ticks));
  Serial.print(Ticks[0]);
  Serial.print(" ");
  Serial.print(Ticks[1]);
  Serial.print(" ");
  Serial.println(Ticks[2]);
  switch (Ticks[0])
  {
  case RC_SetDesiredPositionAll:
  {
    for (uint8_t i = 0; i < (NumberOfMotors); i++)
    {
      Motors[i].setDesiredPositionTicks(Ticks[i + 1]);
    }

    Serial.print(NodeID);
    Serial.print(" | Motor 0: ");
    Serial.print(Motors[0].getDesiredPositionTicks());
    Serial.print("\t");
    Serial.print(Motors[0].getCurrentPositionTicks());
    Serial.print("\t");
    Serial.print(Motors[0].getDutyCycle());
    Serial.print("\t");
    Serial.print(" | Motor 1: ");
    Serial.print(Motors[1].getDesiredPositionTicks());
    Serial.print("\t");
    Serial.print(Motors[1].getCurrentPositionTicks());
    Serial.print("\t");
    Serial.print(Motors[1].getDutyCycle());
    Serial.print("\t");
    Serial.print("| radio debug | ");
    Serial.print(radio.failureDetected);
    Serial.print(" ");
    Serial.print(radio.available());
    Serial.println("");

    break;
  }
  case RC_Zero:
  {
    Serial.println("Zero");
    if (0 == Ticks[1])
    {
      for (uint8_t i = 0; i < (NumberOfMotors); i++)
      {
        Motors[i].setCurrentPositionTicks(0);
        Motors[i].setDesiredPositionTicks(0);
      }
    }
    else
    {
      Motors[Ticks[1] - 1].setCurrentPositionTicks(0);
      Motors[Ticks[1] - 1].setDesiredPositionTicks(0);
    }
    break;
  }
  case RC_ResetRadio:
  {
    radio.begin();
    radio.failureDetected = 0; // Reset the detection value
    radio.openReadingPipe(0, pipes[NodeID]);
    radio.setPALevel(RF24_PA_HIGH);
    radio.startListening();
    delay(10);
    Serial.println("ResetRadio");
    break;
  }
  default:
    break;
  }
}

void ControllerISR(void)
{
  // if we are tracking a trajectory, update the setpoint.
  for (uint8_t i = 0; i < (NumberOfMotors); i++)
  {
    Motors[i].run();
  }
}

void RadioResetISR(void)
{
  if (radio.available())
  {
    RadioResponse();
  }
  radio.begin();
  radio.openReadingPipe(0, pipes[NodeID]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(RF_Channel);
  radio.startListening();
}

void TimerHandler(){
  //I just want this function to run the motor.run function at the specfified period.
  Motors[0].run();
}
