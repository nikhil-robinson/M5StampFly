// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Wi-Fi support



#include <WiFi.h>
#include <WiFiUdp.h>

#define WIFI_SSID "YUDU-DRONE"
#define WIFI_PASSWORD "12345678"
#define WIFI_UDP_IP "255.255.255.255"
#define WIFI_UDP_PORT 14550


void setupWiFi();
void sendWiFi(const uint8_t *buf, int len);

int receiveWiFi(uint8_t *buf, int len);
