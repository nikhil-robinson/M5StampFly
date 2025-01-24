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
#include "flight_control.hpp"
#include <WiFi.h>
#include <WiFiUdp.h>

// esp_now_peer_info_t slave;

volatile uint16_t Connect_flag = 0;

const char* ssid = "STAMP-FLY";
const char* password = "12345678";

WiFiUDP udp;
unsigned int localPort = 12345;

// Task handle for the UDP server task
TaskHandle_t udpServerTaskHandle;


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


    USBSerial.printf("%ld %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %d %d\n\r", 
                                            millis(),
                                            Stick[THROTTLE],
                                            Stick[AILERON],
                                            Stick[ELEVATOR],
                                            Stick[RUDDER],
                                            Stick[BUTTON_ARM],
                                            Stick[BUTTON_FLIP],
                                            Stick[CONTROLMODE],
                                            Stick[ALTCONTROLMODE],
                                            ahrs_reset_flag,
                                            Stick[LOG]);

}

void udpServerTask(void *pvParameters) {
  while (true) {
    // Check for incoming UDP packets
    int packetSize = udp.parsePacket();
    if (packetSize) {
      // Make sure we receive exactly 25 bytes
      if (packetSize == 25) {
        // Read the incoming 25-byte packet
        udp.read(receivedData, 25);

        // Print the received data frame
        USBSerial.println("Received 25-byte packet:");
        for (int i = 0; i < 25; i++) {
          USBSerial.print(receivedData[i], HEX);
          USBSerial.print(" ");
        }
        USBSerial.println();

        // // Optionally, send a response back to the sender
        // udp.beginPacket(udp.remoteIP(), udp.remotePort());
        // udp.write("Message received");
        // udp.endPacket();
        OnDataRecv(receivedData,packetSize);
      } else {
        USBSerial.println("Received packet size is not 25 bytes. Ignoring.");
      }
    }

    // Short delay to yield control to other tasks (optional)
    vTaskDelay(1);  // Delay for 10ms
  }
}

void rc_init(void) {
    // Initialize Stick list
    USBSerial.println("Setting up the Hotspot...");
    WiFi.softAP(ssid, password);

    USBSerial.println("WiFi AP Created");
    USBSerial.print("IP Address: ");
    USBSerial.println(WiFi.softAPIP());

  // Start the UDP server
    udp.begin(localPort);
    USBSerial.println("UDP server started on port " + String(localPort));

  // Create the task to handle UDP packet reception
  xTaskCreate(udpServerTask, "UDP Server Task", 8192, NULL, 10, &udpServerTaskHandle);
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
        cnt    = 0;
    } else
    {
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
    bool status =1;
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
