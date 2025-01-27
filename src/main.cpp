/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include <FastLED.h>
#include "flight_control.hpp"
#include "esp_task_wdt.h"    // Task Watchdog Timer
#include "esp_int_wdt.h"     // Interrupt Watchdog Timer
#include "esp_system.h"      // Core Watchdog Disabling Functions

// VL53L0X_ADDRESS           0x29
// MPU6886_ADDRESS           0x68
// BMP280_ADDRESS            0x76

void setup() {
    // delay(1000 * 10);
     esp_task_wdt_deinit();

    // Disable Main System Watchdog Timer for Core 0 and Core 1
    disableCore0WDT();
    disableCore1WDT();
    init_copter();
    delay(100);
    //  xTaskCreatePinnedToCore((TaskFunction_t)loop_400Hz, "loop_400Hz", 8192, NULL, 20, NULL,1);
}

void loop() {
    loop_400Hz();
    // xTaskCreatePinnedToCore(loop_400Hz, "loop_400Hz", 8192, NULL, 20, NULL,1);
    // return;

}
