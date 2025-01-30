#include <Arduino.h>

#include "RollerStructs.h"
#include "RollerRun.h"

uint8_t roller_num = 0;
Parent parent = {0x0, false, 0};
Child child1 = {create_address(0,8), false, 1};
Child children[] = {child1};

const uint8_t NUM_CHILDREN = sizeof(children)/sizeof(children[0]);

Roller_Settings settings = {};

void setup() {
    settings.roller_num = roller_num;
    settings.parent = parent;
    settings.children = (Child*) children;
    settings.num_children = NUM_CHILDREN;
    settings.baud_rate = 115200;
    roller_setup(settings);
}

void loop() {
    roller_run();
}