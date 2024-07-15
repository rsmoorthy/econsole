#ifndef COMMON_H_ 

#define COMMON_H_

#include <Arduino.h>
#include "DavisRFM69.h"
#include "prefs.h"

void consoleSendRcv(void);
void contSend(void);
void rxvPkt(void);
void mydelay(int);

#endif
