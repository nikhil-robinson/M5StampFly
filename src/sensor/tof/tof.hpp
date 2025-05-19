#pragma once

#include <config.h>

class Tof {
   public:
    Tof()  = default;
    ~Tof() = default;

    virtual void initialize();
    void update() {
        if (micros() - last_update_time_ < SENSOR::TOF_UPDATE_DU * 1000 * 1000) return;

        readTofData();
        last_update_time_ = micros();
    }

    int16_t getTofRange() {
        return tof_range_;
    }

   protected:
    int16_t tof_range_;

   private:
    uint32_t last_update_time_ = 0;
    virtual void readTofData();
};
