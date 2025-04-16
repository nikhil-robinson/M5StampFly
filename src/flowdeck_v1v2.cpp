/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2017, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* flowdeck.c: Flow deck driver */
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "pmw3901.h"
// #include "system.h"
// #include "log.h"
// #include "param.h"
#include "sleepus.h"
#include "config.h"
// #include "stabilizer_types.h"
// #include "estimator.h"
#include "cf_math.h"
#include "Bitcraze_PMW3901.h"
#include "flowdeck_v1v2.hpp"
#include "esp_timer.h"
#include "optical_flow.hpp"

#define AVERAGE_HISTORY_LENGTH 4
#define OULIER_LIMIT 100
#define LP_CONSTANT 0.8f

#define FLOW_TASK_NAME          "FLOW"
#define FLOW_TASK_STACKSIZE           (3 * 1024)
#define FLOW_TASK_PRI           5

//#define USE_LP_FILTER
//#define USE_MA_SMOOTHING

#if defined(USE_MA_SMOOTHING)
static struct {
    float32_t averageX[AVERAGE_HISTORY_LENGTH];
    float32_t averageY[AVERAGE_HISTORY_LENGTH];
    size_t ptr;
} pixelAverages;
#endif

float dpixelx_previous = 0;
float dpixely_previous = 0;

static uint8_t outlierCount = 0;
static float stdFlow = 2.0f;

static bool isInit1 = false;
static bool isInit2 = false;

motionBurst_t currentMotion;

// Disables pushing the flow measurement in the EKF
static bool useFlowDisabled = false;

// Turn on adaptive standard deviation for the kalman filter
static bool useAdaptiveStd = true;

// Set standard deviation flow 
// (will not work if useAdaptiveStd is on)
static float flowStdFixed = 2.0f;

#define NCS_PIN CONFIG_SPI_PIN_CS1


static void flowdeckTask(void *param)
{
    // systemWaitStart();
    // initUsecTimer();
    uint64_t lastTime  = esp_timer_get_time();

    while (1) {

        vTaskDelay(5);
        read_optical_flow_motion(&currentMotion);

        // Flip motion information to comply with sensor mounting
        // (might need to be changed if mounted differently)
        int16_t accpx = -currentMotion.deltaY;
        int16_t accpy = -currentMotion.deltaX;

        // Outlier removal
        if (abs(accpx) < OULIER_LIMIT && abs(accpy) < OULIER_LIMIT) {
        if (useAdaptiveStd)
        {
        // The standard deviation is fitted by measurements flying over low and high texture 
        //   and looking at the shutter time
        float shutter_f = (float)currentMotion.shutter;
        stdFlow=0.0007984f *shutter_f + 0.4335f;


        // The formula with the amount of features instead
        /*float squal_f = (float)currentMotion.squal;
        stdFlow =  -0.01257f * squal_f + 4.406f; */
        if (stdFlow < 0.1f) stdFlow=0.1f;
        } else {
        stdFlow = flowStdFixed;
        }
            // Form flow measurement struct and push into the EKF
            flowMeasurement_t flowData;
            flowData.stdDevX = stdFlow;    // [pixels] should perhaps be made larger?
            flowData.stdDevY = stdFlow;    // [pixels] should perhaps be made larger?

// if task watchdog triggered,flow frequency should set lower

            flowData.dt = 0.005;

#if defined(USE_MA_SMOOTHING)
            // Use MA Smoothing
            pixelAverages.averageX[pixelAverages.ptr] = (float32_t)accpx;
            pixelAverages.averageY[pixelAverages.ptr] = (float32_t)accpy;

            float32_t meanX;
            float32_t meanY;

            xtensa_mean_f32(pixelAverages.averageX, AVERAGE_HISTORY_LENGTH, &meanX);
            xtensa_mean_f32(pixelAverages.averageY, AVERAGE_HISTORY_LENGTH, &meanY);

            pixelAverages.ptr = (pixelAverages.ptr + 1) % AVERAGE_HISTORY_LENGTH;

            flowData.dpixelx = (float)meanX;   // [pixels]
            flowData.dpixely = (float)meanY;   // [pixels]
#elif defined(USE_LP_FILTER)
            // Use LP filter measurements
            flowData.dpixelx = LP_CONSTANT * dpixelx_previous + (1.0f - LP_CONSTANT) * (float)accpx;
            flowData.dpixely = LP_CONSTANT * dpixely_previous + (1.0f - LP_CONSTANT) * (float)accpy;
            dpixelx_previous = flowData.dpixelx;
            dpixely_previous = flowData.dpixely;
#else
            // Use raw measurements
            flowData.dpixelx = (float)accpx;
            flowData.dpixely = (float)accpy;
#endif

            // Push measurements into the estimator
            if (!useFlowDisabled && currentMotion.motion == 0xB0) {
                flowData.dt = (float)(esp_timer_get_time()-lastTime)/1000000.0f;
                lastTime = esp_timer_get_time();
                // estimatorEnqueueFlow(&flowData);
            }
        } else {
            outlierCount++;
        }
    }
}


void flowdeck2Init()
{
	if (isInit1 || isInit2) {
        return;
    }

    xTaskCreate(flowdeckTask, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL, FLOW_TASK_PRI, NULL);
    isInit2 = true;
}

bool flowdeck2Test()
{
    if (!isInit2) {
        USBSerial.printf("Error while initializing the PMW3901 sensor\n");
    }

    return isInit2;//zRanger->test();
}
