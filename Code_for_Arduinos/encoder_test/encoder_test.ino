/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#define MOTOR1_DIR_PIN_A 8
#define MOTOR1_DIR_PIN_B 9
#define MOTOR1_STEP_PIN 5

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(2, 3);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
  //Set all the pins as Outputs
  pinMode(MOTOR1_DIR_PIN_A, OUTPUT);
  pinMode(MOTOR1_DIR_PIN_B, OUTPUT);
  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR1_DIR_PIN_A, HIGH);
  digitalWrite(MOTOR1_DIR_PIN_B, LOW);
  analogWrite(MOTOR1_STEP_PIN, 255);
}

long positionLeft  = -999;

void loop() {
  long newLeft, newRight;
  newLeft = knobLeft.read();
  if (newLeft != positionLeft) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.println();
    positionLeft = newLeft;
  }
  //If the left knob is turned past 100 or negative 100 then stop the motor
  if (newLeft >= 100 || newLeft <= -100) {
    analogWrite(MOTOR1_STEP_PIN, 0);
    digitalWrite(MOTOR1_DIR_PIN_A, LOW);
    digitalWrite(MOTOR1_DIR_PIN_B, LOW);
    //wrote both pins to LOW to double guarantee a stop.
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
  }
}
