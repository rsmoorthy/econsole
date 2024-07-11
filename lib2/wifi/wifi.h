#ifndef WIFI_

#define WIFI_ 1

#include <Arduino.h>
#include <WiFi.h>

#include "myhw.h"

void _wifi_eventcb(WiFiEvent_t event, WiFiEventInfo_t info);
void setup_wifi(void);
void connect_wifi(void);
bool wifiIsConnected(void);
int wifiRSSI(void);
IPAddress wifiIP(void);

#endif /* WIFI_ */
