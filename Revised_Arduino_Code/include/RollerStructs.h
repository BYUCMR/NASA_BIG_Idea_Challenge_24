#ifndef ROLLER_STRUCTS_H
#define ROLLER_STRUCTS_H

#include <stdint.h>

/**
 * @brief Based address for all nodes. The zeros are replaced by another function in RollerRun.h
 */
#define BASE_ADDRESS 0xCECECE0000


/**
 * @brief The possible states of the roller
 * 
 * RECEIVING - Receives from serial (if roller 0) and from radio (for all rollers)
 * CALC_CHECKSUM - Transitioned to by RECEIVING when new data is received from parent. Transitions to TRANSMITTING once complete
 * TRANSMITTING - Transmits to either parent or child, depending on what data was prepared in RECEIVING
 * 
 */
enum State {
    TRANSMITTING,
    RECEIVING,
    CALC_CHECKSUM
};

/**
 * @brief The parent of the roller
 */
typedef struct Parent {
    uint64_t address;
    bool transmission_received;
    uint8_t reading_pipe_num;
} Parent;

/**
 * @brief The child of the roller
 */
typedef struct Child {
    uint64_t address;
    bool transmission_received;
    uint8_t reading_pipe_num;

    Child(uint64_t addr = 0, bool received = false, uint8_t pipe_num = 0)
        : address(addr), transmission_received(received), reading_pipe_num(pipe_num) {}
} Child;

#endif //ROLLER_STRUCTS_H