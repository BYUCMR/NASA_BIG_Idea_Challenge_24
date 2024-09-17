#include "Roller.h"

Roller::Roller(Parent parent, Child children[], uint8_t num_children, uint8_t roller_num, uint8_t max_data_size) : _parent(parent), _roller_num (roller_num), _NUM_CHILDREN(num_children), _DATA_SIZE(max_data_size), _DATA_AND_CHECKSUM_SIZE(max_data_size + 1) {
    _children = (Child *)children;
    data_from_parent = new int16_t[_DATA_AND_CHECKSUM_SIZE];  // The max data size plus one for the checksum
    data_from_child = new int16_t[_DATA_AND_CHECKSUM_SIZE];   // The max data size plus one for the checksum
    error_data = new int16_t[_DATA_AND_CHECKSUM_SIZE];

    _data_and_checksum_bytes = _DATA_AND_CHECKSUM_SIZE * sizeof(data_from_parent[0]);

    memset(data_from_parent, 0, _data_and_checksum_bytes);
    memset(data_from_child, 0, _data_and_checksum_bytes);
    memset(error_data, 0, _data_and_checksum_bytes);
    state = RECEIVING;
    return_to_transmitting = false;
}

Roller::~Roller() {
    delete[] data_from_parent;
    delete[] data_from_child;
    delete[] error_data;
}

uint64_t Roller::get_parent_address() {
    return _parent.address;
}

uint8_t Roller::get_parent_reading_pipe() {
    return _parent.reading_pipe_num;
}

bool Roller::did_parent_receive_tx() {
    return _parent.transmission_received;
}

uint64_t Roller::get_child_address(uint8_t index) {
    return _children[index].address;
}

bool Roller::did_child_receive_tx(uint8_t index) {
    return _children[index].transmission_received;
}

void Roller::set_child_received_tx(uint8_t index, bool received) {
    _children[index].transmission_received = received;
}

uint8_t Roller::get_child_reading_pipe(uint8_t index) {
    return _children[index].reading_pipe_num;
}

uint8_t Roller::get_num_children() {
    return _NUM_CHILDREN;
}

uint8_t Roller::get_data_size() {
    return _DATA_SIZE;
}

uint8_t Roller::get_data_and_checksum_size() {
    return _DATA_AND_CHECKSUM_SIZE;
}

void Roller::reset_children_received_flags() {
    for (int idx = 0; idx < _NUM_CHILDREN; idx++) {
        _children[idx].transmission_received = false;
    }
}

int16_t Roller::calc_checksum_parent_data() {
    return calc_checksum_varlen(data_from_parent, get_data_size());
}

int16_t Roller::calc_checksum_child_data() {
    return calc_checksum_varlen(data_from_child, get_data_size());
}

int16_t Roller::calc_checksum(int16_t *data) {
    int16_t checksum = 0;

    for (int i = 0; i < _DATA_SIZE; i++) {
        checksum += (i % 3 + 1) * data[i];
    }

    return checksum;
}

int16_t Roller::calc_checksum_varlen(int16_t *data, uint8_t len) {
    int16_t checksum = 0;

    for (int i = 0; i < len; i++) {
        checksum += (i % 3 + 1) * data[i];
    }

    return checksum;
}

void Roller::reset_data_from_child() {
    memset(data_from_child, 0, _data_and_checksum_bytes);
}

void Roller::reset_data_from_parent() {
    memset(data_from_parent, 0, _data_and_checksum_bytes);
}

void Roller::reset_error_data() {
    memset(error_data, 0, _data_and_checksum_bytes);
}

uint8_t Roller::get_parent_data_bytes() {
    return _data_and_checksum_bytes;
}

uint8_t Roller::get_child_data_bytes() {
    return _data_and_checksum_bytes;
}

uint8_t Roller::get_error_data_bytes() {
    return _data_and_checksum_bytes;
}

uint8_t Roller::get_roller_num() {
    return _roller_num;
}