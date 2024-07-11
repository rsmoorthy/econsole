#ifndef DS3231_RTC_

#define DS3231_RTC_ 1

#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"


void setup_rtc(void);
DateTime now(void);
uint32_t unixtime(void);
void setTime(DateTime *tm);
const char *getISOTime(void);


#endif /* DS3231_RTC_ */




