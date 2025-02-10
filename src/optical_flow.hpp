#ifndef OPTICAL_FLOW_HPP
#define OPTICAL_FLOW_HPP

#include <Arduino.h>
#include "common.h"


void read_optical_flow(int16_t *dx,int16_t *dy);
void optical_flow_init(void);

#endif