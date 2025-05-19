#pragma once

#include <BasicLinearAlgebra.h>
#include <memory>
#include <config.h>

class OpticalFlow {
   public:
    OpticalFlow()  = default;
    ~OpticalFlow() = default;

    virtual void initialize();
    void update() {
        if (micros() - last_update_time_ < SENSOR::OPTICAL_FLOW_UPDATE_DU * 1000 * 1000) return;

        readOpticalFlowData();
        last_update_time_ = micros();
    }

    void dumpFlow() {
        USBSerial.printf("OpticalFlow: %d %d\n", flow_delta_x_, flow_delta_y_);
    }
    float getDeltaX() {
        return (float)flow_delta_x_;
    }
    float getDeltaY() {
        return (float)flow_delta_y_;
    }

   protected:
    int16_t flow_delta_x_;
    int16_t flow_delta_y_;

   private:
    uint32_t last_update_time_         = 0;
    virtual void readOpticalFlowData() = 0;
};
