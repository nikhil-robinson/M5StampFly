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

#include "communication/telemetry.hpp"

#include "communication/rc.hpp"
#include "devices/led/led.hpp"

uint8_t Telem_mode     = 0;
uint8_t Telem_cnt      = 0;
const uint8_t MAXINDEX = 120;
const uint8_t MININDEX = 30;

namespace Telemetry {
float telemetry_roll_           = 0.0;
float telemetry_pitch_          = 0.0;
float telemetry_yaw_            = 0.0;
float battery_voltage_          = 0.0;
float telemetry_position_x_     = 0.0;
float telemetry_position_y_     = 0.0;
float altitude_                 = 0.0;
uint8_t telemetry_flight_state_ = 0;
float average_loop_time_        = 0.0;

void telemetry_sequence(void);
void make_telemetry_data(uint8_t* senddata);
void data2log(uint8_t* data_list, float add_data, uint8_t index);
void float2byte(float x, uint8_t* dst);
void append_data(uint8_t* data, uint8_t* newdata, uint8_t index, uint8_t len);
void data_set(uint8_t* datalist, float value, uint8_t* index);
void data_set_uint32(uint8_t* datalist, uint32_t value, uint8_t* index);
void data_set_uint16(uint8_t* datalist, uint16_t value, uint8_t* index);
void data_set_uint8(uint8_t* datalist, uint8_t value, uint8_t* index);

void telemetry(void) {
    uint8_t senddata[MAXINDEX];

    telemetry_sequence();
}

void telemetry_sequence(void) {
    uint8_t senddata[MAXINDEX];

    make_telemetry_data(senddata);

    if (RemoteControl::telemetry_send(senddata, sizeof(senddata)) == 1)
        esp_led(0x110000, 1);  // Telemetory Reciver OFF
    else
        esp_led(0x001100, 1);  // Telemetory Reciver ON
}

void setRpy(float roll, float pitch, float yaw) {
    telemetry_roll_  = roll;
    telemetry_pitch_ = pitch;
    telemetry_yaw_   = yaw;
}
void setBatteryVoltage(float voltage) {
    battery_voltage_ = voltage;
}
void setPositionX(float x) {
    telemetry_position_x_ = x;
}
void setPositionY(float y) {
    telemetry_position_y_ = y;
}
void setAltitude(float altitude) {
    altitude_ = altitude;
}
void setFlightState(uint8_t flight_state) {
    telemetry_flight_state_ = flight_state;
}
void setAverageLoopTime(float time) {
    average_loop_time_ = time;
}

void make_telemetry_data(uint8_t* senddata) {
    float d_float;
    uint8_t d_int[4];
    uint8_t index = 0;

    // Telemetry Header
    senddata[0] = 88;
    senddata[1] = 88;
    index       = 2;
    data_set(senddata, 0.0, &index);                 // 1 Time
    data_set(senddata, 0.0, &index);                 // 2 delta Time
    data_set(senddata, telemetry_roll_, &index);     // 3 Roll_angle
    data_set(senddata, telemetry_pitch_, &index);    // 4 Pitch_angle
    data_set(senddata, telemetry_yaw_, &index);      // 5 Yaw_angle
    data_set(senddata, average_loop_time_, &index);  // 6 average loop time
    data_set(senddata, 0.0, &index);                 // 7 Q
    data_set(senddata, 0.0, &index);                 // 8 R
    data_set(senddata, 0.0, &index);                 // 9 Roll_angle_reference
    data_set(senddata, 0.0, &index);                 // 10 Pitch_angle_reference
    // data_set(senddata, 0.5 * 189.0f* Pitch_angle_command, index);
    data_set(senddata, 0.0, &index);               // 11 P ref
    data_set(senddata, 0.0, &index);               // 12 Q ref
    data_set(senddata, 0.0, &index);               // 13 R ref
    data_set(senddata, 0.0, &index);               // 14 T ref
    data_set(senddata, battery_voltage_, &index);  // 15 Voltage
    data_set(senddata, 0.0, &index);               // 16 Accel_x_raw
    data_set(senddata, 0.0, &index);               // 17 Accel_y_raw
    data_set(senddata, 0.0, &index);               // 18 Accel_z_raw
    data_set(senddata, 0.0, &index);               // 19 Alt Velocity
    data_set(senddata, 0.0, &index);               // 20 Z_dot_ref
    // data_set(senddata, FrontRight_motor_duty, index);
    data_set(senddata, telemetry_position_x_, &index);  // 21
    data_set(senddata, telemetry_position_y_, &index);  // 22
    // data_set(senddata, RearLeft_motor_duty, index);
    data_set(senddata, 0.0, &index);                            // 23 Alt_ref
    data_set(senddata, 0.0, &index);                            // 24 Altitude2
    data_set(senddata, altitude_, &index);                      // 25 Sense_Alt
    data_set(senddata, 0.0, &index);                            // 26 Az
    data_set(senddata, 0.0, &index);                            // 27 Az_bias
    data_set_uint8(senddata, 0, &index);                        // 28.1 Alt_flag(1 byte)
    data_set_uint8(senddata, telemetry_flight_state_, &index);  // 28.2 fly mode(1 byte)
    data_set_uint16(senddata, 0, &index);                       // 28.3-4 tof front
}

void data_set(uint8_t* datalist, float value, uint8_t* index) {
    data2log(datalist, value, *index);
    *index = *index + 4;
}

void data_set_uint32(uint8_t* datalist, uint32_t value, uint8_t* index) {
    append_data(datalist, (uint8_t*)&value, *index, 4);
    *index = *index + 4;
}

void data_set_uint16(uint8_t* datalist, uint16_t value, uint8_t* index) {
    append_data(datalist, (uint8_t*)&value, *index, 2);
    *index = *index + 2;
}

void data_set_uint8(uint8_t* datalist, uint8_t value, uint8_t* index) {
    append_data(datalist, (uint8_t*)&value, *index, 1);
    *index = *index + 1;
}

void data2log(uint8_t* data_list, float add_data, uint8_t index) {
    uint8_t d_int[4];
    float d_float = add_data;
    float2byte(d_float, d_int);
    append_data(data_list, d_int, index, 4);
}

void float2byte(float x, uint8_t* dst) {
    uint8_t* dummy;
    dummy  = (uint8_t*)&x;
    dst[0] = dummy[0];
    dst[1] = dummy[1];
    dst[2] = dummy[2];
    dst[3] = dummy[3];
}

void append_data(uint8_t* data, uint8_t* newdata, uint8_t index, uint8_t len) {
    for (uint8_t i = index; i < index + len; i++) {
        data[i] = newdata[i - index];
    }
}

}  // namespace Telemetry
