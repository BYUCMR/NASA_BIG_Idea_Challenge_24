#ifndef ROLLER_RUN_H
#define ROLLER_RUN_H

#include <Arduino.h>

#define USE_TIMER_1 true
#define USE_TIMER_2 false

#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <time.h>
#include "RollerStructs.h"
#include "Roller.h"
#include "DCMotorControl.h"
#include "Roller.h"

#define LED_PIN_RED 19          // red LED, use to indicate receiving.
#define LED_PIN_GREEN 18        // green LED, use to indicate transmitting.
#define FAST_BLINK 100          // miliseconds
#define SLOW_BLINK 1000         // miliseconds

typedef struct Roller_Settings {
    uint8_t roller_num;
    Parent parent;
    Child* children;
    uint8_t num_children;
    uint32_t baud_rate;
} Roller_Settings;

/**
 * @brief Takes care of assigning unique addresses for all communication.
 * 
 * @param num_parent The parent roller number (e.g., roller 1).
 * @param num_child The child roller number (e.g., roller 2).
 * 
 * Note: Avoid using hexadecimal values A, 5, 2, and 1 in addresses.
 * They have alternating 0s and 1s in binary. (More details: https://maniacalbits.blogspot.com/2013/04/rf24-addressing-nrf24l01-radios-require.html)
 *
 * Be cautious when defining parameters:
 * - For parent and self: num_parent is the parent roller's number, and num_child is the self roller's number.
 * - For self and child: num_parent is the self roller's number, and num_child is the child roller's number.
 * @return A 40-bit address stored in a uint64_t object
 */
uint64_t create_address(uint8_t num_parent, uint8_t num_child);


/**
 * @brief The abstracted roller_setup function to be called in the setup() function of initial .cpp file
 * 
 * @param settings A struct containing configuration information (roller number, addresses, etc) 
 * for the specific roller
 */
void roller_setup(Roller_Settings settings);

/**
 * @brief The abstracted roller_run function to be called repeatedly in the loop() function of
 * initial .cpp file
 */
void roller_run();

#endif