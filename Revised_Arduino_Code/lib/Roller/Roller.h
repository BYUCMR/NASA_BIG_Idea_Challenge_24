#ifndef ROLLER_H
#define ROLLER_H

#include <stdint.h>

#include "RollerStructs.h"
#include "string.h"
// Roller 0 sends data for us to 14 active rollers

/**
 * @brief The default value for data_fromt_parent, error_data, and data_from_child
 * 
 * @note The actual size of the array ends of being MAX_DATA_SIZE (or whatever parameter is passed into the constructor),
 * plus one more because of the checksum value. For the solar panel mount, there are 14 active rollers that need
 * kinematic data, hence the number 14.
 */
static const uint8_t MAX_DATA_SIZE = 14;

/**
 * @brief A class defining the necessary configurations, helper functions, and info of a roller in the NASA Lunar Robot
 */
class Roller {
   public:

    /**
     * Constructor for the Roller Class
     * @param parent An instance of the struct Parent describing the parent of this node
     * @param children An array of struct Child describing all children of this node
     * @param num_children Number of children that this node has
     * @param max_data_size The number of data points being sent (ie total active rollers) for this transmission scheme. 
     * The default value is a preprocessor directive MAX_DATA_SIZE. This value must be less than 15. Do not include space for the checksum number.
     */
    Roller(Parent parent, Child children[], uint8_t num_children, uint8_t roller_num, uint8_t max_data_size = MAX_DATA_SIZE);

    /**
     * @brief Destroy the Roller object
     */
    ~Roller();

    /**
     * @brief Get the parent address (40-bits)
     * 
     * @return uint64_t 
     */
    uint64_t get_parent_address();

    /**
     * @brief Get the parent reading pipe number
     * 
     * @return uint8_t number of reading pipe
     */
    uint8_t get_parent_reading_pipe();

    /**
     * @brief Get a child's address from the children array
     * 
     * @param index the index of the child as stored in the array children
     * @return uint64_t 40-bit address
     */
    uint64_t get_child_address(uint8_t index);

    /**
     * @brief Get the child reading pipe number
     * 
     * @param index the index of the child as stored in the array children
     * @return uint8_t number of reading pipe
     */
    uint8_t get_child_reading_pipe(uint8_t index);

    /**
     * @brief Get the number of children
     * 
     * @return uint8_t 
     */
    uint8_t get_num_children();

    /**
     * @brief Get the number of elements in data arrays (excluding checksum)
     * 
     * @return uint8_t 
     */
    uint8_t get_data_size();

    /**
     * @brief Get the number of elements in data arrays (including checksum)
     * 
     * @return uint8_t 
     */
    uint8_t get_data_and_checksum_size();

    /**
     * @brief Get the size of the data_from_parent array in bytes
     * 
     * @return uint8_t 
     */
    uint8_t get_parent_data_bytes();

    /**
     * @brief Get the size of the data_from_child array in bytes
     * 
     * @return uint8_t 
     */
    uint8_t get_child_data_bytes();

    /**
     * @brief Get the size of the error_data array in bytes
     * 
     * @return uint8_t 
     */
    uint8_t get_error_data_bytes();

    /**
     * @brief Get this roller's number designation
     * 
     * @return uint8_t 
     */
    uint8_t get_roller_num();

    /**
     * @brief Returns whether or not the parent received transmission
     * 
     * @return true - Parent did receive transmission
     * @return false - Parent did not receive transmission
     */
    bool did_parent_receive_tx();

    /**
     * @brief Returns whether of not a child received a transmission
     * 
     * @param index The index of the child as stored in the array children
     * @return true - Child did receive transmission
     * @return false - Child did not receive transmission
     */
    bool did_child_receive_tx(uint8_t index);

    /**
     * @brief Set whether or not a child received transmission
     * 
     * @param index The index of the child as stored in the array children
     * @param received the boolean value to set whether it was received or not (true = received)
     */
    void set_child_received_tx(uint8_t index, bool received);

    /**
     * @brief Calculates the checksum for the data_from_parent array
     * 
     * @return int16_t checksum value
     */
    int16_t calc_checksum_parent_data();

    /**
     * @brief Calculates the checksum for the data_from_child array
     * 
     * @return int16_t checksum value
     */
    int16_t calc_checksum_child_data();

    /**
     * @brief Calculates the checksum for a data array
     * 
     * @param data the data array to calculate checksum for
     * 
     * @note this method assumes there are _DATA_SIZE number of data points to be used in the calculation 
     * 
     * @return int16_t checksum value
     */
    int16_t calc_checksum(int16_t* data);

    /**
     * @brief Calculates the checksum for any data array and a variable length
     * 
     * @param data the int16_t data array
     * @param len the length of data to calculate checksum over
     * @return int16_t checksum value
     */
    int16_t calc_checksum_varlen(int16_t* data, uint8_t len);

    /**
     * @brief Resets all children "transmission received" struct members to false
     */
    void reset_children_received_flags();

    /**
     * @brief Clears the data_from_child array to all 0s
     */
    void reset_data_from_child();

    /**
     * @brief Clears the data_from_parent array to all 0s
     */
    void reset_data_from_parent();

    /**
     * @brief Clears the error_data array to all 0s
     */
    void reset_error_data();

    /**
     * @brief Overall state of the node
     * 
     * Enumeration values are RECEIVING, CHECKSUM, and TRANSMITTING
     */
    State state = RECEIVING;

    /**
     * @brief A flag which tells the node to return to transmit state after checking if it has receved anything
     */
    bool return_to_transmitting = false;

    /**
     * @brief Head of the data_from_parent array
     */
    int16_t* data_from_parent;

    /**
     * @brief Head of the data_from_child array
     */
    int16_t* data_from_child;

    /**
     * @brief Head of the error_data array
     */
    int16_t* error_data;

   private:

   /**
    * @brief Parent of the roller
    */
    Parent _parent;

    /**
    * @brief Array containing the children of the roller
    */
    Child* _children;

    /**
     * @brief Number designation of the roller
     */
    uint8_t _roller_num;

    /**
     * @brief Number of children of the roller
     */
    const uint8_t _NUM_CHILDREN;

    /**
     * @brief Number of elements in data arrays (excluding checksum element)
     */
    const uint8_t _DATA_SIZE;

    /**
     * @brief Number of elements in data arrays (including checksum element)
     */
    const uint8_t _DATA_AND_CHECKSUM_SIZE;

    /**
     * @brief Number of bytes in the data arrays (including checksum element)
     */
    uint8_t _data_and_checksum_bytes;
};

#endif  // ROLLER_H