#include <Arduino.h>
//Define pins:
//Motor 1 Direction Pins are connected to D16 and D17
#define MOTOR1_DIR_PIN_A 16
#define MOTOR1_DIR_PIN_B 17
//Motor 1 Step Pin is connected to D5
#define MOTOR1_STEP_PIN 5

void setup(){
    //Set all the pins as Outputs
    pinMode(MOTOR1_DIR_PIN_A, OUTPUT);
    pinMode(MOTOR1_DIR_PIN_B, OUTPUT);
    pinMode(MOTOR1_STEP_PIN, OUTPUT);
}

void loop(){
    //Set motor direction clockwise
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