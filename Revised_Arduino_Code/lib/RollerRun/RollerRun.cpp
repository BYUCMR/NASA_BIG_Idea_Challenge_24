#include "RollerRun.h"

#include "TimerInterrupt.h"

//#define PRINT_DETAILS

#ifdef PRINT_DETAILS
#include "printf.h"
#endif

//#define DEBUG_ISR

#define NumberOfMotors 1
#define ControlRate_ms 10
#define ControlRate_us 10000
#define TIMER_INTERVAL_MS 10L  // 10ms, or 10,000us as specfified by the ControlRate_us variable in the DCMotorControl.h file.
#define DeadbandTicks 100
#define DeadbandDutyCycle 5
#define TicksPerRevolution (6678.624)
#define TicksPerInch (24.5) * (6678.624 / 43.9822)  // = 3270.283 for 60 rpm motor.
#define HomingSpeedTolerance 0.01
#define MinimumPWM 0
#define Kp 0.01
#define Ki 0.003
#define Kd 0.001
#define DutyCycleStall 25
#define MaxDutyCycleDelta 5
#define MOTOR_ERROR 6  // pin for reading error state from the half bridge motor driver. If it goes low, something is wrong.
#define MOTOR_SLEEP 4  // pin for waking up motor. If it goes high, motor is active

DCMotorControl Motors[] = {
    DCMotorControl(10, 5, 3, 2)  // DCMotorControl::DCMotorControl( uint8_t DirectionPin, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) //uses this constructor second to illustrate the point.
};

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

RF24 radio(7, 8);  // CE, CSN
uint8_t parent_data_index = 0;
static uint8_t DATA_SIZE;
static uint8_t DATA_AND_CHECKSUM_SIZE;
static uint8_t NUM_CHILDREN;
static bool is_node_0 = false;

static bool tx_to_child_complete = true;
static bool tx_to_parent_complete = true;

static bool new_setpoint = false;

Roller* self;

/**
 * @brief Initializes key aspects of the roller; called by roller_setup
 */
void init_roller();

/**
 * @brief Initializes the nRF24L01 module; called by init_roller();
 */
void init_radio();

/**
 * @brief Initializes the motor driver; called by init_roller();
 */
void init_motor();

/**
 * @brief Initializes the LEDS; called by init_roller();
 */
void init_LEDs();

/**
 * @brief Receives data from Serial and trasnfers it to checksum
 *
 * Only called if the roller is not roller_00000 (i.e. the computer)
 */
void serial_receive();

/**
 * Receives data from the radio
 *
 * Processes the data received in the following structre:
 * - If data is from parent and is valid, prepare to send to children (if any) then set motor desired position
 * - If data is from parent and is invalid, prepare to ask the parent to resend the data
 * - If data is from child and asks for a resend and this roller has valid data, then prepare to resend data
 * - If data is from child and asks for a resend and this roller has invalid data, then do nothing (the data
 * will be automatically send once received by parent)
 */
void radio_receive();

/**
 * Transmit the data prepared for it from the radio_receive() function
 *
 * In all cases, the radio module checks if a child is currently trying to talk with it. If so, then a flag is
 * set to read that data, process it, and later return to transmitting. Transmission is attempted until completion
 * or until three attempts have been made.
 *
 * Transmits the data in the following structure:
 * - If data is to be sent to parent, send to parent. If failed three times, proceed to state RECEIVING. If
 * nothing to receive, return to radio_transmit and try again
 * - If data is to be sent to all children, iterate through each child and send. If successfully sent, mark that
 * child as having received. In each attempt, if a child has been marked as having successfully received the data,
 * this child is skipped. If, after three attempts, at least one child was not marked as successful, then proceed to
 * state RECEIVING. If nothing to receive, return to radio_transmit and try again.
 * - If data is to be sent to a single child, then the above process will only be done for that one child. All other
 * children will not be sent the data they already received.
 */
void radio_transmit();

/**
 * @brief An unblocking method to blink the red LED light
 *
 * @param delay_time The number of miliseconds for the red light to flash on for.
 */
void blink_red_led(unsigned long delay_time);

/**
 * @brief The timer1-based interrupt-service routine to control the motor.run() function
 *
 * If new data has been received, the new desired setpoint is established here.
 */
static void motor_control_ISR();

uint64_t create_address(uint8_t num_parent, uint8_t num_child) {
    static const uint8_t address_hex[] = {0x3, 0x4, 0x6, 0x7, 0x8, 0x9, 0xB, 0xC, 0xD, 0xE, 0xF};
    static const uint8_t num_hex = sizeof(address_hex) / sizeof(address_hex[0]);
    uint64_t new_address = BASE_ADDRESS;

    new_address += (uint64_t)address_hex[num_parent / num_hex] << 12;
    new_address += (uint64_t)address_hex[num_parent % num_hex] << 8;
    new_address += (uint64_t)address_hex[num_child / num_hex] << 4;
    new_address += (uint64_t)address_hex[num_child % num_hex];

    return new_address;
}

void roller_setup(Roller_Settings settings) {
    Serial.begin(settings.baud_rate);

#ifdef PRINT_DETAILS
    printf_begin();
#endif

    self = new Roller(settings.parent, settings.children, settings.num_children, settings.roller_num);

    if (self->get_roller_num() == 0) {
        is_node_0 = true;
    }

    NUM_CHILDREN = settings.num_children;

    init_roller();

    DATA_SIZE = self->get_data_size();
    DATA_AND_CHECKSUM_SIZE = self->get_data_and_checksum_size();

    // Set tx_data to 0s
    self->reset_data_from_parent();
    self->reset_data_from_child();
    self->reset_error_data();
}

void roller_run() {
    switch (self->state) {
        case RECEIVING:  // this case will be run until all serial data is received, up to 14 numbers
            blink_red_led(SLOW_BLINK);
            if (is_node_0 == true) {
                serial_receive();
            }
            radio_receive();
            break;
        case CALC_CHECKSUM:
            self->data_from_parent[DATA_SIZE] = self->calc_checksum_parent_data();
            self->state = TRANSMITTING;
            for (int i = 0; i < DATA_AND_CHECKSUM_SIZE; i++) {
            }
            break;
        case TRANSMITTING:
            radio_transmit();
            break;
    }
}

void init_roller() {
    init_radio();

    init_motor();

    // Set parent state to wait for data from serial. This should be the default option in class Roller, but just in case
    self->state = RECEIVING;

    // Set LED pins high then low to show power on
    init_LEDs();
}

void init_radio() {
    radio.begin();

    radio.powerDown();
    radio.powerUp();

    // This sets the radio frequency to 2476 MHz. This is the default value
    radio.setChannel(76);

    // Sets data transfer rate to 1 megabits per sec
    radio.setDataRate(RF24_1MBPS);

    radio.setAddressWidth(5);

    for (int i = 0; i < 6; i++) {
        radio.closeReadingPipe(i);
    }

    // Open reading pipes to all children
    for (int i = 0; i < NUM_CHILDREN; i++) {
        radio.openReadingPipe(self->get_child_reading_pipe(i), self->get_child_address(i));
    }

    if (!is_node_0) {
        radio.openReadingPipe(self->get_parent_reading_pipe(), self->get_parent_address());
    }

    // Set power level to max for long distance communication
    radio.setPALevel(RF24_PA_MAX);

    // Open first writing pipe by default (to child1)
    if (NUM_CHILDREN > 0) {
        radio.openWritingPipe(self->get_child_address(0));
    }

    // Set the timeout and number of tries for the child to sent back an auto ack
    radio.setRetries(15, 15);

    // All rollers are listening by default. Once the TRANSMITTING state is reached, it is set to stop listening
    radio.startListening();

#ifdef PRINT_DETAILS
    radio.printDetails();
#endif
}

void init_motor() {
#ifdef USE_TIMER_1
    if (!is_node_0) {
        ITimer1.init();
        if (ITimer1.attachInterruptInterval(ControlRate_ms, motor_control_ISR))
            ;
    }
    ITimer1.restartTimer();
#endif
    for (uint8_t i = 0; i < NumberOfMotors; i++) {
        Motors[i].setParameters(Kp, Ki, Kd, ControlRate_us, DeadbandTicks, DeadbandDutyCycle, TicksPerInch, TicksPerRevolution, MinimumPWM);
        Motors[i].setDutyCycleStall(DutyCycleStall);
        Motors[i].setMaxDutyCycleDelta(MaxDutyCycleDelta);
    }

    for (uint8_t i = 0; i < (NumberOfMotors); i++) {
        MotorEnabled = true;
        Motors[i].setMotorEnable(MotorEnabled);
        Motors[i].setMode(DC_Automatic);
        Motors[i].setDesiredPositionInches(0);
    }

    pinMode(MOTOR_ERROR, INPUT);
    pinMode(MOTOR_SLEEP, OUTPUT);
    digitalWrite(MOTOR_SLEEP, HIGH);  // set motor to awake.
}

void init_LEDs() {
    pinMode(LED_PIN_RED, OUTPUT);
    pinMode(LED_PIN_GREEN, OUTPUT);
    digitalWrite(LED_PIN_RED, HIGH);    // LED is ON.
    digitalWrite(LED_PIN_GREEN, HIGH);  // LED is ON
    delay(1000);
    digitalWrite(LED_PIN_RED, LOW);    // LED is OFF.
    digitalWrite(LED_PIN_GREEN, LOW);  // LED is OFF
}

void serial_receive() {
    if (Serial.available()) {
        // reset tx_data array
        self->reset_data_from_parent();
        parent_data_index = 0;

        // Turn on LEDs
        digitalWrite(LED_PIN_RED, HIGH);
        digitalWrite(LED_PIN_GREEN, HIGH);

        // Get data from Serial
        String incoming_data_str = Serial.readStringUntil('\n');  // Read until newline character (the resultant string does not include \n)
        incoming_data_str.trim();                                 // Remove any leading or trailing whitespace

        unsigned int start_index = 0;
        int end_index = incoming_data_str.indexOf(',');

        // Iterate through each substring in between ',' characters until end of string
        while (end_index > 0) {
            String number_substring = incoming_data_str.substring(start_index, end_index);  // grabs the substring at start_index inclusive and ends at end_index exclusive
            int16_t number = number_substring.toInt();

            self->data_from_parent[parent_data_index++] = number;

            start_index = end_index + 1;
            end_index = incoming_data_str.indexOf(',', start_index);

            // If more data was sent in string than allowed for, break from the loop
            if (parent_data_index == DATA_SIZE) {
                break;
            }
        }

        // If maximum data has not been reached, then collect the last number from incoming_data_str
        if (parent_data_index < DATA_SIZE && start_index < incoming_data_str.length()) {
            String number_substring = incoming_data_str.substring(start_index);  // grabs the substring at start_index inclusive till end of string
            int16_t number = number_substring.toInt();

            self->data_from_parent[parent_data_index++] = number;
        }

        // If data was correctly received, then transition to CALC_CHECKSUM
        if (parent_data_index > 0) {
            tx_to_child_complete = false;
            self->state = CALC_CHECKSUM;
            self->return_to_transmitting = false;  // This prevents the state machine from trying to submit old data
            self->reset_children_received_flags();
        }

        // Turn LEDs off
        digitalWrite(LED_PIN_RED, LOW);
        digitalWrite(LED_PIN_GREEN, LOW);
    }
}

void radio_receive() {
    static bool have_correct_data = false;
    static uint8_t* pipe_num = new uint8_t;
    static int16_t prev_checksum = 32000;  // ridiculous number for the first time through
    if (radio.available(pipe_num)) {
#ifdef PRINT_DETAILS
        radio.printDetails();
#endif
        digitalWrite(LED_PIN_GREEN, HIGH);
        if (!is_node_0 && *pipe_num == self->get_parent_reading_pipe()) {  // the parent is always on pipe 0
            noInterrupts();
            self->reset_data_from_parent();
            radio.read(self->data_from_parent, self->get_parent_data_bytes());
            interrupts();
            // if (self->data_from_parent[DATA_SIZE] != prev_checksum) {                          // Sees if it was the same data from before
            if (self->data_from_parent[DATA_SIZE] == self->calc_checksum_parent_data()) {  // Success
                Serial.println("Success read from parent");
                prev_checksum = self->data_from_parent[DATA_SIZE];
                have_correct_data = true;
                new_setpoint = true;
                Serial.println(new_setpoint);
                if (NUM_CHILDREN > 0) {
                    self->state = TRANSMITTING;
                    self->reset_children_received_flags();
                    tx_to_child_complete = false;
                }
            } else {  // Failure
                Serial.println("Failed read from parent");
                have_correct_data = false;
                tx_to_parent_complete = false;
                self->reset_error_data();
                self->error_data[0] = -1;
                self->state = TRANSMITTING;
            }
            //}
        } else if (*pipe_num != self->get_parent_reading_pipe() && NUM_CHILDREN > 0) {  // From child. The pipe_num corresponds to one more than their index in the array
            noInterrupts();
            self->reset_data_from_child();
            radio.read(self->data_from_child, self->get_child_data_bytes());
            interrupts();

            if (self->data_from_child[0] == -1) {  // Error: please resend data
                if (have_correct_data || is_node_0) { //Either has correct data or is node 0 (which always has correct data)
                    Serial.println("Child did not receive. Resending");
                    self->state = TRANSMITTING;
                    self->set_child_received_tx(*pipe_num - 1, false); //Fix this
                    tx_to_child_complete = false;
                } else {
                    // This case should never be entered. This implies that this roller had incorrect data but still sent it to a child
                }
            }
        }

        digitalWrite(LED_PIN_GREEN, LOW);
    }
    if (self->return_to_transmitting == true) {
        self->return_to_transmitting = false;
        self->state = TRANSMITTING;
    }
}

void radio_transmit() {
    if (!radio.available()) {
        if (tx_to_child_complete == false) {
            digitalWrite(LED_PIN_GREEN, HIGH);
            radio.stopListening();
            uint8_t attempt = 0;

            noInterrupts();
            while (!tx_to_child_complete && attempt < 3) {
                tx_to_child_complete = true;
                for (int i = 0; i < NUM_CHILDREN; i++) {
                    if (self->did_child_receive_tx(i) == false) {
                        radio.openWritingPipe(self->get_child_address(i));
                        if (radio.write(self->data_from_parent, self->get_parent_data_bytes()) == false) {
                            tx_to_child_complete = false;
                            Serial.print("Failed to send to child");
                            Serial.println(i);
                        } else {
                            self->set_child_received_tx(i, true);
                            Serial.print("Sent to child");
                            Serial.println(i);
                        }
                    }
                }
                attempt++;
            }

            radio.startListening();
            interrupts();

            if (!tx_to_child_complete) {
                self->return_to_transmitting = true;
            }

            digitalWrite(LED_PIN_GREEN, LOW);
        }
        if (tx_to_parent_complete == false) {
            digitalWrite(LED_PIN_GREEN, HIGH);
            radio.stopListening();
            uint8_t attempt = 0;

            noInterrupts();
            while (!tx_to_parent_complete && attempt < 3) {
                tx_to_parent_complete = true;

                radio.openWritingPipe(self->get_parent_address());
                if (radio.write(self->error_data, self->get_error_data_bytes()) == false) {
                    tx_to_parent_complete = false;
                    Serial.println("Transmission to parent incomplete");
                } else {
                    Serial.println("Transmission to parent complete");
                    tx_to_parent_complete = true;
                }
                attempt++;
            }

            radio.startListening();
            interrupts();

            if (!tx_to_parent_complete) {
                self->return_to_transmitting = true;
            }

            digitalWrite(LED_PIN_GREEN, LOW);
        }

    } else {  // if radio is available, then switch to receiving, process that data, then return to transmit
        self->return_to_transmitting = true;
    }
    self->state = RECEIVING;
}

void blink_red_led(unsigned long delay_time) {
    static unsigned long past_time = millis();
    static uint8_t led_state = LOW;
    if ((millis() - past_time) > delay_time) {
        led_state = ~led_state;
        digitalWrite(LED_PIN_RED, led_state);
        past_time = millis();
    }
}

static void motor_control_ISR(void) {
    static int new_setpoint_val = 0;
    #ifdef DEBUG_ISR
    Serial.println("ISR");
    #endif
    // if we are tracking a trajectory, update the setpoint.
    for (uint8_t i = 0; i < (NumberOfMotors); i++) {
        #ifdef DEBUG_ISR
        Serial.println("Running motors");
        Serial.println(new_setpoint);
        #endif
        if (new_setpoint == true) {
            Serial.print("New setpoint: ");
            new_setpoint_val = self->data_from_parent[self->get_roller_num() - 1];
            Serial.println(new_setpoint_val);
            Motors[i].setDesiredPositionInches(new_setpoint_val);
            new_setpoint = false;
        }
        Motors[i].run();
    }
    new_setpoint = false;
}