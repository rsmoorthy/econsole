#ifndef BME280_SENSOR_

#define BME280_SENSOR_ 1

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "oled.h"

#define BME280_I2C_ADDRESS 0x76

#define SEALEVELPRESSURE_HPA (1013.25)

void setup_bme280(void);
void get_bme280_sensor_data(float *temp, float *humidity, float *pressure, float *altitude);
void print_bme280_sensor_data(void);
void display_bme280_sensor_data(void);


#endif /* BME280_SENSOR_ */
