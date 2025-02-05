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
#include "sensor.hpp"
#include "Bitcraze_PMW3901.h"
#include "tof.hpp"
#include "imu.hpp"
#include "button.hpp"

// VL53L0X_ADDRESS           0x29
// MPU6886_ADDRESS           0x68
// BMP280_ADDRESS            0x76


#if 1
void setup() {
    init_copter();
    delay(100);
}

void loop() {
    loop_400Hz();
}


#else

Bitcraze_PMW3901 flow(12);


void setup() {
    // delay(1000 * 10);
    // init_copter();
    init_button();
    USBSerial.begin(115200);
    delay(1500);
    USBSerial.printf("Start StampFly!\r\n");

    sensor_init();
    while (!flow.begin()) {
        USBSerial.printf("Initialization of the flow sensor failed\r\n");
    }

    USBSerial.printf("Finish sensor init!\r\n");
    delay(100);
}

void loop() 
{
    int16_t deltaX,deltaY;
    flow.readMotionCount(&deltaX, &deltaY);
    imu_update();  // IMUの値を読む前に必ず実行
    float acc_x  = imu_get_acc_x();
    float acc_y  = imu_get_acc_y();
    float acc_z  = imu_get_acc_z();
    float gyro_x = imu_get_gyro_x();
    float gyro_y = imu_get_gyro_y();
    float gyro_z = imu_get_gyro_z();

    USBSerial.printf("X[%d] Y[%d]\r\n",deltaX,deltaY);
    USBSerial.printf("%d mm\r\n",tof_bottom_get_range());
    USBSerial.printf("ACC-X[%f] ACC-Y[%f] ACC-Z[%f]\r\n",acc_x,acc_y,acc_z);
    USBSerial.printf("GYRO-X[%f] GYRO-Y[%f] GYRO-Z[%f]\r\n",gyro_x,gyro_y,gyro_z);
    delay(100);

}

#endif