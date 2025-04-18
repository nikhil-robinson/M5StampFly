// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Wi-Fi support

#if WIFI_ENABLED

#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>

#define WIFI_SSID "flix"
#define WIFI_PASSWORD "flixwifi"
#define WIFI_UDP_IP "255.255.255.255"
#define WIFI_UDP_PORT 14550


void setupWiFi();
void sendWiFi(const uint8_t *buf, int len);

int receiveWiFi(uint8_t *buf, int len);

#endif
