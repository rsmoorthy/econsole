#include "davis.h"

DavisRFM69 radio(RFM_CS, RFM_IRQ);

uint32_t pkts_rxvd = 0;

void setup_davis(void) {

  Serial.println("Init Radio....");
  radio.init();
  radio.setChannel(10);
  Serial.println("...done");
}

void davis_setHighPower(void) {
  radio.setHighPower(true);
}

int davis_rssi(void) {
  return radio.readRSSI();
}

byte davis_temperature(void) {
  return radio.readTemperature(-1);
}

void davis_isr0(void) {
  return radio.isr0();
}

void davis_print_data_rxved(void) {
  // radio.setChannel(0);
  // radio.isr0();
  if (radio.receiveDone()) {
    int rssi = radio.rssi();
    bool syncMatched = radio.isSyncAddressMatched();
    pkts_rxvd++;
    if (rssi > -80) {
      Serial.print(millis()); Serial.print(" Davis ("); Serial.print(pkts_rxvd); Serial.print(")- rssi: ");
      Serial.print(rssi); Serial.print(" data: ");
      // Serial.print(" [ "); Serial.print(rssi < -103, DEC); Serial.print(" ] ");
      for(int i=0; i < 8; i++) {
        Serial.print(radio.data(i), HEX); Serial.print(" ");
      }
      Serial.print(" crc: "); Serial.print(radio.crc16(), HEX);
      Serial.print(" sync: "); Serial.print(syncMatched, DEC);
      Serial.print(" irqflags1: "); Serial.print(radio.irqFlags1(), HEX);
      Serial.println(" ");
    }
    // radio.setChannel(0);
    radio.receiveBegin();
  }
  else {
    // Serial.println("Davis - no data rxved");
  }
}

unsigned char send_counter = 0;
void davis_send_frame(void) {

  byte buf[] = { send_counter, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x88 };

  Serial.println("Sending radio data......");
  radio.send((const char *)buf, 6);
  Serial.print(millis()); Serial.println(" ....done");
  send_counter++;
}

