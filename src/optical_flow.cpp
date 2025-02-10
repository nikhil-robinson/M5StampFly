#include <Arduino.h>
#include "common.h"
#include "optical_flow.hpp"
#include "Bitcraze_PMW3901.h"

// int16_t deltaX, deltaY;
Bitcraze_PMW3901 flow(12);

void optical_flow_init(void) {
    bool st;

    USBSerial.printf("Start OPTICAL FLOW Initialize!\n\r");

    if (!flow.begin()) {
        USBSerial.printf("OPTICAL FLOW INIT Fail!\n\r");
        while (1);
    }
}

void read_optical_flow(int16_t *dx,int16_t *dy)
{
    flow.readMotionCount(dx, dy);
}