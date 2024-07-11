#include "myrtc.h"

char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
RTC_DS3231 rtc;

void setup_rtc() {

  int retry=10;
  while(retry--) {
    if(rtc.begin()) break;
    delay(10);
  }

  if(retry == 0) return;

  if(rtc.lostPower()) {
    Serial.println("RTC lost power, setting time now");

    // Sets the time to the time when the sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

DateTime now(void) {
  return rtc.now();
}

uint32_t unixtime(void) {
  return rtc.now().unixtime();
}

char ts_string[20];

const char *getISOTime(void) {
  DateTime now = rtc.now();
  snprintf(ts_string, 29, "%d-%02d-%02dT%02d:%02d:%02d", 
          now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second() );
  return (const char *)ts_string;
}
