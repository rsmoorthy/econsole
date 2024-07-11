
#define TXMITTER_ 1

#include <SPI.h>
// #include <Wire.h>

// #include "oled.h"
// #include "i2c_scanner.h"
// #include "bme280.h"
// #include "rtc.h"
#include <wifi.h>
#include <davis.h>

#define LED 2

uint32_t start = millis();
uint32_t start2 = millis();


void setup() {
  Serial.begin(115200);
  while(!Serial)
    delay(10);
  // Wire.begin();

  delay(5000);
  // i2cScanner();
  // setup_bme280();
  // delay(1000);
  // Serial.println("Setup OLED");
  // setup_oledDisplay();
  // Serial.println("Setup OLED ... Done");
  // delay(5000);
  // setup_rtc();
  setup_wifi();
  setup_davis();
  davis_setHighPower();
}
void loop() {
  if ((millis() - start) > 5000) {
    // print_bme280_sensor_data();
    // test_oledDisplay();
    // delay(100);
    // display_bme280_sensor_data();
    // Serial.print("Unix Time now is "); Serial.println(unixtime());
    // Serial.print("ISO Time is "); Serial.println(getISOTime());
    bool conn = wifiIsConnected();
    Serial.print("WiFi status"); Serial.print(conn);
    if(conn) {
      Serial.print(" "); Serial.println(wifiIP());
    }
    else {
      Serial.println(" ");
    }
    start = millis();
  }
  if ((millis() - start2) > 1000) {
    davis_send_frame();
    start2 = millis();
  }

  // Serial.print("Reading RSSI from Davis... "); // Serial.println(davis_rssi());
  // Serial.print(" "); Serial.println(davis_temperature());
  // davis_print_data_rxved();
  // Serial.println("...\n");
  // delay(1000);
  // davis_isr0();
}
