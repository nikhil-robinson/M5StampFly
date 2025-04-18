// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// MAVLink communication
#pragma once



#include <MAVLink.h>
#include "quaternion.h"

#define SYSTEM_ID 1
#define PERIOD_SLOW 1.0
#define PERIOD_FAST 0.1
#define MAVLINK_CONTROL_SCALE 0.7f
#define MAVLINK_CONTROL_YAW_DEAD_ZONE 0.1f

extern float mavlinkControlScale ;


void processMavlink();

void sendMavlink();

void sendMessage(const void *msg);
void receiveMavlink();

void handleMavlink(const void *_msg);

// Send shell output to GCS
void mavlinkPrint(const char* str);

// Convert Forward-Left-Up to Forward-Right-Down quaternion
inline Quaternion fluToFrd(const Quaternion &q);
