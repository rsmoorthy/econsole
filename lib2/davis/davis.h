#ifndef DAVIS_

#define DAVIS_ 1

#include <Arduino.h>
#include "DavisRFM69.h"

#define RFM_CS   5
#define RFM_IRQ  13
#define RFM_MOSI 23
#define RFM_MISO 19
#define RFM_CLK  18

void setup_davis(void);
int davis_rssi(void);
byte davis_temperature(void);
void davis_print_data_rxved(void);
void davis_isr0(void);
void davis_send_frame(void);
void davis_setHighPower(void);

#endif /* DAVIS_ */
