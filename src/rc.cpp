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

#include "rc.hpp"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "flight_control.hpp"
#include "pid.hpp"
#include "optical_flow.hpp"
#include "tof.hpp"

// esp_now_peer_info_t slave;

volatile uint16_t Connect_flag = 0;

// BLE Service and Characteristic UUIDs
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"  // Custom Service UUID
#define CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-abcdef123456"  // Custom Characteristic UUID

#define FRAME_BUFFER_LEN 25

// Buffer to store received data (uint8_t array)
uint8_t receivedData[100];      // Adjust the size if needed
size_t receivedDataLength = 0;  // Track the length of received data

uint8_t sentData[100];      // Adjust the size if needed
size_t sentDataLength = 0;  // Track the length of received data

uint8_t TelemAddr[6] = {0};
volatile uint8_t MyMacAddr[6];
volatile uint8_t peer_command[4] = {0xaa, 0x55, 0x16, 0x88};
volatile uint8_t Rc_err_flag     = 0;

// RC
volatile float Stick[16];
volatile uint8_t Recv_MAC[3];

volatile uint8_t ble_send_status = 0;

// 受信コールバック
void OnDataRecv(const uint8_t *recv_data, int data_len) {
    Connect_flag = 0;

    uint8_t *d_int;
    // int16_t d_short;
    float d_float;

    // Recv_MAC[0] = recv_data[0];
    // Recv_MAC[1] = recv_data[1];
    // Recv_MAC[2] = recv_data[2];

    // if ((recv_data[0] == MyMacAddr[3]) && (recv_data[1] == MyMacAddr[4]) && (recv_data[2] == MyMacAddr[5])) {
    Rc_err_flag = 0;
    // } else {
    //     Rc_err_flag = 1;
    //     return;
    // }

    // checksum
    uint8_t check_sum = 0;
    for (uint8_t i = 0; i < 24; i++) check_sum = check_sum + recv_data[i];
    // if (check_sum!=recv_data[23])USBSerial.printf("checksum=%03d recv_sum=%03d\n\r", check_sum, recv_data[23]);
    if (check_sum != recv_data[24]) {
        // USBSerial.printf("ERROR checksum=%03d recv_sum=%03d\n\r", check_sum, recv_data[23]);
        Rc_err_flag = 1;
        return;
    }

    d_int         = (uint8_t *)&d_float;
    d_int[0]      = recv_data[3];
    d_int[1]      = recv_data[4];
    d_int[2]      = recv_data[5];
    d_int[3]      = recv_data[6];
    Stick[RUDDER] = d_float;

    d_int[0]        = recv_data[7];
    d_int[1]        = recv_data[8];
    d_int[2]        = recv_data[9];
    d_int[3]        = recv_data[10];
    Stick[THROTTLE] = d_float;

    d_int[0]       = recv_data[11];
    d_int[1]       = recv_data[12];
    d_int[2]       = recv_data[13];
    d_int[3]       = recv_data[14];
    Stick[AILERON] = d_float;

    d_int[0]        = recv_data[15];
    d_int[1]        = recv_data[16];
    d_int[2]        = recv_data[17];
    d_int[3]        = recv_data[18];
    Stick[ELEVATOR] = d_float;

    Stick[BUTTON_ARM]     = recv_data[19];  // auto_up_down_status
    Stick[BUTTON_FLIP]    = recv_data[20];
    Stick[CONTROLMODE]    = recv_data[21];  // Mode:rate or angle control
    Stick[ALTCONTROLMODE] = recv_data[22];  // 高度制御

    ahrs_reset_flag = recv_data[23];

    Stick[LOG] = 0.0;
    // if (check_sum!=recv_data[23])USBSerial.printf("checksum=%03d recv_sum=%03d\n\r", check_sum, recv_data[23]);

    USBSerial.printf("%ld %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %d %d\n\r", millis(), Stick[THROTTLE],
                     Stick[AILERON], Stick[ELEVATOR], Stick[RUDDER], Stick[BUTTON_ARM], Stick[BUTTON_FLIP],
                     Stick[CONTROLMODE], Stick[ALTCONTROLMODE], ahrs_reset_flag, Stick[LOG]);
}

volatile bool data_task_running = false;

void data_sender(void *pvParameters);
void positionHold(void *pvParameters);

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        uint8_t *value = pCharacteristic->getData();
        size_t length  = pCharacteristic->getLength();

        // Store the received data into the buffer
        // if (length > 0) {
        //     memcpy(receivedData + receivedDataLength, value, length);  // Copy data into receivedData buffer
        //     receivedDataLength += length;
        // }

        // // USBSerial.printf("GOT BLE data of size %d\n\r",receivedDataLength);

        // if (receivedDataLength == FRAME_BUFFER_LEN) {
        //     OnDataRecv(receivedData, receivedDataLength);
        //     // memset(receivedData,0,sizeof(receivedData));
        //     receivedDataLength = 0;
        // }

        if ((value[0] == 'S') && (value[1] == 'T') && !data_task_running) {
            xTaskCreatePinnedToCore(positionHold, "positionHold", 4096, NULL, 20, NULL, 0);
        }
    }

    void onRead(BLECharacteristic *pCharacteristic) {
        // Send the received data back to the client if available
        if (receivedDataLength > 0) {
            pCharacteristic->setValue(sentData, sentDataLength);  // Set the value to send back
        }
    }
};

enum MovementState { MOVE_FORWARD, STOP_FORWARD, MOVE_BACKWARD, STOP_BACKWARD };

MovementState moveState = MOVE_FORWARD;

float focal_length = 35.0; 

float calculateDistanceCM(int16_t deltaX, int16_t deltaY, int16_t height_mm) {
    float height_cm = height_mm / 10.0;  // Convert mm to cm

    float dx = (deltaX * height_cm) / focal_length;
    float dy = (deltaY * height_cm) / focal_length;
    float distance_cm = sqrt(dx * dx + dy * dy);  // Total distance in cm

    Serial.print("Height (mm): "); Serial.print(height_mm);
    Serial.print("\tDelta X (cm): "); Serial.print(dx);
    Serial.print("\tDelta Y (cm): "); Serial.print(dy);
    Serial.print("\tTotal Distance (cm): "); Serial.println(distance_cm);

    return distance_cm;
}

void positionHold(void *pvParameters) {
    const float loopInterval = 0.0025f;  // 400Hz

    USBSerial.printf("SET DEFAULT VALUE\r\n");
    Stick[THROTTLE]       = 0.0;
    Stick[AILERON]        = 0.0;
    Stick[ELEVATOR]       = 0.0;
    Stick[RUDDER]         = 0.0;
    Stick[BUTTON_ARM]     = 0.0;
    Stick[BUTTON_FLIP]    = 0.0;
    Stick[CONTROLMODE]    = 1.0;
    Stick[ALTCONTROLMODE] = 4.0;
    ahrs_reset_flag       = 0.0;
    Stick[LOG]            = 0.0;

    vTaskDelay(pdMS_TO_TICKS(200));

    USBSerial.printf("AHRS RESET\r\n");

    ahrs_reset_flag = 1.0;
    vTaskDelay(pdMS_TO_TICKS(200));
    ahrs_reset_flag = 0.0;

    USBSerial.printf("ARM\r\n");
    Stick[BUTTON_ARM] = 1.0;
    vTaskDelay(pdMS_TO_TICKS(200));
    Stick[BUTTON_ARM] = 0.0;

    USBSerial.printf("LIFT OFF\r\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Proportional control constants for X and Y axes
    const float KpX = 1.0;  // Adjust this gain as needed
    const float KpY = 1.0;  // Adjust this gain as needed

    float target_distance  = 20.0;  // Target distance in cm
    float current_distance = 0.0;   // Current traveled distance
    float elevator_command = 0.0;   // Elevator stick control (-1 to 1)

    while (true) {

        int16_t deltaX, deltaY,heigth;
        read_optical_flow(&deltaX, &deltaY);
        heigth = tof_bottom_get_range();

        current_distance += calculateDistanceCM(deltaX, deltaY,heigth);
        USBSerial.print("Distance: ");
        USBSerial.print(current_distance);
        USBSerial.print("\n");
        continue;



        // Convert motion to cm (adjust scaling factor based on sensor calibration)
        current_distance += deltaY * 0.1;  // Example scaling factor

        // Finite state machine (FSM) for movement control
        switch (moveState) {
            case MOVE_FORWARD: {
                USBSerial.println("MOVE_FORWARD");
                if (current_distance < target_distance) {
                    elevator_command = 0.1;  // Move forward
                } else {
                    elevator_command = 0.0;  // Stop
                    moveState        = STOP_FORWARD;
                }
                break;
            }

            case STOP_FORWARD: {
                USBSerial.println("STOP_FORWARD");
                vTaskDelay(pdMS_TO_TICKS(1000));  // Small pause
                moveState = MOVE_BACKWARD;
                break;
            }

            case MOVE_BACKWARD: {
                USBSerial.println("Move backward Position");
                if (current_distance > 0) {
                    elevator_command = -0.1;  // Move backward
                } else {
                    elevator_command = 0.0;  // Stop
                    moveState        = STOP_BACKWARD;
                }
                break;
            }

            case STOP_BACKWARD: {
                USBSerial.println("Reached Start Position");
                vTaskDelay(portMAX_DELAY);
            }
            break;
        }

        // Update stick inputs
        Stick[ELEVATOR] = elevator_command;

        // Debugging
        USBSerial.print("Distance: ");
        USBSerial.print(current_distance);
        USBSerial.print(" cm, Elevator Command: ");
        USBSerial.println(Stick[ELEVATOR]);

        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

void data_sender(void *pvParameters) {
    data_task_running = true;
    // vTaskDelay(pdMS_TO_TICKS(5000));
    USBSerial.printf("SET DEFAULT VALUE\r\n");
    Stick[THROTTLE]       = 0.0;
    Stick[AILERON]        = 0.0;
    Stick[ELEVATOR]       = 0.0;
    Stick[RUDDER]         = 0.0;
    Stick[BUTTON_ARM]     = 0.0;
    Stick[BUTTON_FLIP]    = 0.0;
    Stick[CONTROLMODE]    = 1.0;
    Stick[ALTCONTROLMODE] = 4.0;
    ahrs_reset_flag       = 0.0;
    Stick[LOG]            = 0.0;

    vTaskDelay(pdMS_TO_TICKS(200));

    USBSerial.printf("AHRS RESET\r\n");

    ahrs_reset_flag = 1.0;
    vTaskDelay(pdMS_TO_TICKS(200));
    ahrs_reset_flag = 0.0;

    USBSerial.printf("ARM\r\n");
    Stick[BUTTON_ARM] = 1.0;
    vTaskDelay(pdMS_TO_TICKS(200));
    Stick[BUTTON_ARM] = 0.0;

    USBSerial.printf("LIFT OFF\r\n");
    vTaskDelay(pdMS_TO_TICKS(3000));
    for (float i = 0.0; i > -0.1; i -= 0.001) {
        Stick[ELEVATOR] = i;
        USBSerial.printf("ELEVETOR %6.3f\r\n", Stick[ELEVATOR]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Stick[ELEVATOR] = -0.1;
    USBSerial.printf("ELEVETOR %6.3f\r\n", Stick[ELEVATOR]);
    vTaskDelay(pdMS_TO_TICKS(3000));
    for (float i = -0.1; i < 0.0; i += 0.001) {
        Stick[ELEVATOR] = i;
        USBSerial.printf("ELEVETOR %6.3f\r\n", Stick[ELEVATOR]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Stick[ELEVATOR] = 0.0;
    USBSerial.printf("ELEVETOR %6.3f\r\n", Stick[ELEVATOR]);
    vTaskDelay(pdMS_TO_TICKS(3000));
    for (float i = 0.0; i < 0.1; i += 0.001) {
        Stick[ELEVATOR] = i;
        USBSerial.printf("ELEVETOR %6.3f\r\n", Stick[ELEVATOR]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Stick[ELEVATOR] = 0.1;
    USBSerial.printf("ELEVETOR %6.3f\r\n", Stick[ELEVATOR]);
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (float i = 0.1; i > 0.0; i -= 0.001) {
        Stick[ELEVATOR] = i;
        USBSerial.printf("ELEVETOR %6.3f\r\n", Stick[ELEVATOR]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Stick[ELEVATOR] = 0.0;
    USBSerial.printf("ELEVETOR %6.3f\r\n", Stick[ELEVATOR]);
    vTaskDelay(pdMS_TO_TICKS(3000));

    Stick[CONTROLMODE]    = 0.0;
    Stick[ALTCONTROLMODE] = 0.0;

    USBSerial.printf("DISARM\r\n");
    Stick[BUTTON_ARM] = 1.0;
    vTaskDelay(pdMS_TO_TICKS(200));
    Stick[BUTTON_ARM] = 0.0;

    Stick[CONTROLMODE]    = 0.0;
    Stick[ALTCONTROLMODE] = 0.0;

    data_task_running = false;
    vTaskDelete(NULL);
}

void rc_init(void) {
    // Initialize Stick list
    for (uint8_t i = 0; i < 16; i++) Stick[i] = 0.0;

    BLEDevice::init("STAMP-FLY-DRONE");
    BLEServer *pServer                 = BLEDevice::createServer();
    BLEService *pService               = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
    pCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // Helps with iPhone connections
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
    USBSerial.println("ESP-BLE Ready.");
}

void send_peer_info(void) {
    uint8_t data[11];
    data[0] = CHANNEL;
    memcpy(&data[1], (uint8_t *)MyMacAddr, 6);
    // memcpy(&data[1 + 6], (uint8_t *)peer_command, 4);
}

uint8_t telemetry_send(uint8_t *data, uint16_t datalen) {
    static uint32_t cnt       = 0;
    static uint8_t error_flag = 0;
    static uint8_t state      = 0;

    esp_err_t result;

    if ((error_flag == 0) && (state == 0)) {
        // result = esp_now_send(peerInfo.peer_addr, data, datalen);
        cnt = 0;
    } else {
        cnt++;
    }

    if (ble_send_status == 0) {
        error_flag = 0;
        // state = 0;
    } else {
        error_flag = 1;
        // state = 1;
    }
    // 一度送信エラーを検知してもしばらくしたら復帰する
    if (cnt > 500) {
        error_flag = 0;
        cnt        = 0;
    }
    cnt++;
    // USBSerial.printf("%6d %d %d\r\n", cnt, error_flag, esp_now_send_status);

    return error_flag;
}

void rc_end(void) {
    // Ps3.end();
}

uint8_t rc_isconnected(void) {
    bool status = 1;
    // Connect_flag++;
    // if (Connect_flag < 40)
    //     status = 1;
    // else
    //     status = 0;
    // USBSerial.printf("%d \n\r", Connect_flag);
    return status;
}

void rc_demo() {
}
