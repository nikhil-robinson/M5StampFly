#ifndef FLOWDECKV1V2_H_
#define FLOWDECKV1V2_H_

#include <stdint.h>
#include <stdbool.h>


typedef struct flowMeasurement_s {
    uint32_t timestamp;
    union {
      struct {
        float dpixelx;  // Accumulated pixel count x
        float dpixely;  // Accumulated pixel count y
      };
      float dpixel[2];  // Accumulated pixel count
    };
    float stdDevX;      // Measurement standard deviation
    float stdDevY;      // Measurement standard deviation
    float dt;           // Time during which pixels were accumulated
  } flowMeasurement_t;

void flowdeck2Init();
bool flowdeck2Test();

#endif