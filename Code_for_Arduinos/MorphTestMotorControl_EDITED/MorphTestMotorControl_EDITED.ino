//Timer Library setup
#define TIMER_INTERRUPT_DEBUG         2
#define _TIMERINTERRUPT_LOGLEVEL_     0
// The link to the repository is as follows: https://github.com/khoih-prog/TimerInterrupt?tab=readme-ov-file#important-notes-about-isr
// NOTES ABOUT ISR:
#define USE_TIMER_1     true

#if ( defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)  || \
        defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) ||    defined(ARDUINO_AVR_ETHERNET) || \
        defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT)   || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO)      || \
        defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) || \
        defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
        defined(ARDUINO_AVR_PROTRINKET3FTDI) )
  #define USE_TIMER_2     false
  #warning Using Timer1, Timer2
#endif
#include "TimerInterrupt.h"
// Init timer ITimer1
// init timer ITimer2


// IntervalTimer ControlTimer;
// IntervalTimer RadioResetTimer;

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
  DCMotorControl(8, 9, 5, 2, 3), //DCMotorControl::DCMotorControl( uint8_t DirectionPinA, uint8_t DirectionPinB, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) //uses this constructor first!!

};
#define NumberOfMotors 1//(sizeof(Motors) / sizeof(Motors[0]))
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

// IntervalTimer ControlTimer;
// IntervalTimer RadioResetTimer;
void ControllerISR(void);
void RadioResetISR(void);
// Debug Setup
void TimerHandler(void)
{
  static bool toggle = false;

#if (TIMER_INTERRUPT_DEBUG > 1)
  #if USE_TIMER_2
    Serial.print("ITimer2 called, millis() = ");
  #elif USE_TIMER_3
    Serial.print("ITimer3 called, millis() = ");
  #endif
  
  Serial.println(millis());
#endif

  //timer interrupt toggles outputPin
  // digitalWrite(outputPin, toggle);
  // toggle = !toggle;
}
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting!!");
  delay(1000);
  #if USE_TIMER_1

  // Timer0 is used for micros(), millis(), delay(), etc and can't be used
  // Select Timer 1-2 for UNO, 0-5 for MEGA
  // Timer 2 is 8-bit timer, only for higher frequency

  ITimer1.init();

  // Using ATmega328 used in UNO => 16MHz CPU clock ,

  if (ITimer1.attachInterruptInterval(ControlRate_ms, ControllerISR))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
  #endif
  #if USE_TIMER_2 

  ITimer2.init(); //This is the RadioResetTimer

  if (ITimer2.attachInterruptInterval(RadioResetRate_us, TimerHandler))
  {
    Serial.print(F("Starting  ITimer2 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer2. Select another freq. or timer"));
  #endif
  radio.begin();
  radio.openReadingPipe(0, pipes[NodeID]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(RF_Channel);
  radio.startListening();

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
  
  // ControlTimer.begin( ControllerISR , ControlRate_us ); // attach the service routine here
  // RadioResetTimer.begin( RadioResetISR , RadioResetRate_us ); // attach the service routine here
  Motors[0].setDesiredPositionInches(3);
  Serial.println("Setup Complete");
}
void loop()
{
  // make it so that the motor moves back and forth between 3 and -3 inches
  // but also make it so that motor[0].run() is called every 10ms
  // if(Motors[0].getCurrentPositionInches() > 3){
  //   Motors[0].setDesiredPositionInches(-3);
  //   Serial.println("Setting Desired Position to -3");
  // }
  // else if(Motors[0].getCurrentPositionInches() < -3){
  //   Motors[0].setDesiredPositionInches(3);
  //   Serial.println("Setting Desired Position to 3!");
  // }
  // Serial.print("The current motor Position is: ");
  // Serial.println(Motors[0].getCurrentPositionInches());

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
  // Serial.println("ControllerISR");
  // if we are tracking a trajectory, update the setpoint.
  for (uint8_t i = 0; i < (NumberOfMotors); i++)
  {
    Motors[i].run();
    // Serial.println("Motor Ran");
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


