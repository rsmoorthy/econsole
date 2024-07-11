#ifndef PROTO_OLED_DISPLAY_

#define PROTO_OLED_DISPLAY_ 1
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>

#define LED 2

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET   -1
#define SCREEN_ADDRESS 0x3C

void setup_oledDisplay(void);
void test_oledDisplay(void);

void test_drawline(void);
void testscrolltext(void);
void showText(char *text, int x, int y, int size);
void clearDisplay(void);
void showDisplay(void);
Adafruit_SSD1306 getDisplay(void);

#endif /* PROTO_OLED_DISPLAY_ */
