#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <spi_s3.hpp>


typedef struct {
    uint8_t chipid;
    uint8_t dipihc;
} optconfig_t;

extern optconfig_t optconfig;

uint8_t powerUp(optconfig_t *optconfig);
void initRegisters(void);
void readMotionCount(int16_t *deltaX, int16_t *deltaY);
void enableFrameCaptureMode(void);
void readImage(uint8_t *image);
void opt_init();
void opt_read();
float getDeltaX();
float getDeltaY();