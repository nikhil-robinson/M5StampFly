#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <spi_s3.hpp>
#include <sensor/optical_flow/optical_flow.hpp>

namespace PMW3901 {
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
}  // namespace PMW3901

class OpticalFlowPMW3901 : public OpticalFlow {
   public:
    OpticalFlowPMW3901(){};
    ~OpticalFlowPMW3901() = default;

    void initialize() override;
    void readOpticalFlowData();
};
