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