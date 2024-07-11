#include "wifi.h"

const char *WIFI_SSID = "Advaita";
const char *WIFI_Password = "NonduaLity";

void _wifi_eventcb(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.print("Received Wifi event ");
  Serial.print(event);
  if(event == WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.print(" Info: "); Serial.println(info.wifi_sta_disconnected.reason);
  }
  else
    Serial.println(" ");

  connect_wifi();
}

void setup_wifi() {
  Serial.println("Setting up Wifi...");
  WiFi.mode(WIFI_STA);
  Serial.println("... Done");
  connect_wifi();
  return;
  WiFi.onEvent(_wifi_eventcb, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.onEvent(_wifi_eventcb, WiFiEvent_t::ARDUINO_EVENT_WIFI_READY);
  WiFi.onEvent(_wifi_eventcb, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_START);
}

void connect_wifi() {
  Serial.println("Connecting to Wifi...");
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("Already connected.. Exiting");
    return;
  }

  Serial.println("Disconnect issued");
  WiFi.disconnect(true);
  Serial.println("....done");
  Serial.println("Begin issued.");
  WiFi.begin(WIFI_SSID, WIFI_Password);
  Serial.print("Connecting to WiFi..."); Serial.print(WIFI_SSID);

  int retry = 300;
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
    retry--;
    if (retry <= 0) {
      Serial.print("Timeout connecting to WiFi..."); Serial.println(WIFI_SSID);
      break;
    }
  }
}

bool wifiIsConnected() {
  return WiFi.status() == WL_CONNECTED;
}

int wifiRSSI() {
  return WiFi.RSSI();
}

IPAddress wifiIP() {
  return WiFi.localIP();
}
