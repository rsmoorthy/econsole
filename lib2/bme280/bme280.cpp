#include "bme280.h"
#include "myrtc.h"
#include "wifi.h"

Adafruit_BME280 bme; // I2C

void setup_bme280() {

  bool status;

  status = bme.begin(BME280_I2C_ADDRESS);
  if(!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!!");
    while (1);
  }
}

void get_bme280_sensor_data(float *temp, float *humidity, float *pressure, float *altitude) {
  *temp = bme.readTemperature();
  *humidity = bme.readHumidity();
  *pressure = bme.readPressure() / 100.0F;
  *altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void print_bme280_sensor_data() {

  float temp, humidity, pressure, altitude;

  get_bme280_sensor_data(&temp, &humidity, &pressure, &altitude);

  Serial.print("Temp: "); Serial.print(temp); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
}

void display_bme280_sensor_data() {

  float temp, humidity, pressure, altitude;
  char buf [100];

  get_bme280_sensor_data(&temp, &humidity, &pressure, &altitude);

  clearDisplay();
  if(wifiIsConnected()) {
    IPAddress ip = wifiIP();
    snprintf(buf, 99, "IP:%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]); showText(buf, 5, 0, 1);
  }
  else {
    snprintf(buf, 99, "WiFi Conn: No"); showText(buf, 5, 0, 1);
  }
  snprintf(buf, 99, "Temp : %.2f C", temp); showText(buf, 5, 16, 1);
  snprintf(buf, 99, "Hum  : %.2f %%", humidity); showText(buf, 5, 16+10, 1);
  snprintf(buf, 99, "Pres : %.2f hPa", pressure); showText(buf, 5, 16+20, 1);
  snprintf(buf, 99, "Alt  : %.2f m", altitude); showText(buf, 5, 16+30, 1);
  snprintf(buf, 99, "%s", getISOTime()); showText(buf, 5, 16+40, 1);
  /*
  showText("Temp: " + String(temp) + " *C", 5, 5, 1);
  showText("Humidity: " + String(humidity) + " %", 5, 15, 1);
  showText("Pressure: " + String(pressure) + " hPa", 5, 25, 1);
  showText("Altitude: " + String(altitude) + " m", 5, 35, 1);
  */
}
