#include "common.h"

extern DavisRFM69 radio;

void consoleSendRcv()
{
  // Set up a "buffer" for characters that we'll send:

  static char sendbuffer[8]; // 62
  static int sendlength = 0;
  static uint32_t snd_counter = 0;
  static uint32_t rxvd_counter = 0;
  static uint32_t err_counter = 0;
  static uint32_t missed_counter = 0;
  static uint8_t cnter = 0;
  static uint8_t exp_cnter = 0;

  // SENDING

  // In this section, we'll gather serial characters and
  // send them to the other node if we (1) get a carriage return,
  // or (2) the buffer is full (61 characters).

  // If there is any serial input, add it to the buffer:

  if (Serial.available() > 0)
  {
    char input = Serial.read();

    if (input != '\r') // not a carriage return
    {
      sendbuffer[sendlength] = input;
      sendlength++;
    }

    // If the input is a carriage return, or the buffer is full:

    if ((input == '\r') || (sendlength == 8)) // CR or buffer full  (was 61)
    {
      // Send the packet!
      if(sendlength < 8) {
        for(byte i=sendlength; i < 8; i++) {
          sendbuffer[i] = 'A';
        }
        sendlength = 8;
      }

      sendbuffer[1] = cnter++;
      snd_counter++;

      Serial.print(millis());
      Serial.print(" -> ("); Serial.print(snd_counter); Serial.print("/"); Serial.print(err_counter); 
      Serial.print(") -> sent to ");
      Serial.print(TONODEID, DEC);
      Serial.print(", msg [");
      for (byte i = 0; i < sendlength; i++) {
        Serial.print(sendbuffer[i], HEX); Serial.print(" ");
      }
      Serial.println(" ]");

      // There are two ways to send packets. If you want
      // acknowledgements, use sendWithRetry():

      if (USEACK)
      {
        if (radio.sendWithRetry(TONODEID, sendbuffer, sendlength))
          Serial.println("ACK received!");
        else
          Serial.println("no ACK received");
      }

      // If you don't need acknowledgements, just use send():

      else // don't use ACK
      {
        radio.send(TONODEID, sendbuffer, sendlength);
      }

      sendlength = 0; // reset the packet
      // Blink(LED,10);
    }
  }

  // RECEIVING

  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:

  if (radio.receiveDone()) // Got one!
  {
    // Print out the information:
    if(exp_cnter == radio.DATA[1]) {
      rxvd_counter++;
      exp_cnter++;
    }
    else {
      if(radio.DATA[1] < exp_cnter)
        missed_counter = 256 - exp_cnter + radio.DATA[1];
      else
        missed_counter += radio.DATA[1] - exp_cnter;
      exp_cnter = radio.DATA[1] + 1;
      rxvd_counter++;
    }

    Serial.print(millis());
    Serial.print(" -> ("); Serial.print(rxvd_counter); Serial.print("/"); Serial.print(missed_counter); 
    Serial.print(") -> rcvd fm ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print(", msg [");

    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:

    for (byte i = 0; i < radio.DATALEN; i++) {
      Serial.print(radio.DATA[i], HEX); Serial.print(" ");
    }

    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.

    Serial.print("], RSSI:");
    Serial.print(radio.RSSI);
    Serial.print(",");
    Serial.println(radio.RSSI2);


    // Send an ACK if requested.
    // (You don't need this code if you're not using ACKs.)

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.println("ACK sent");
    }
  }
}

int16_t rssi;
uint32_t err_counter = 0;
unsigned char sendbuf[8] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x5A, 0xA5 };

void contSend(bool ack)
{
  static uint32_t start = 0;
  static uint32_t snt_counter = 0;
  static uint32_t rxvd_counter = 0;
  static uint32_t missed_counter = 0;
  static unsigned char cnt = 0;
  // static unsigned char buf[8] = { 'h', 'e', 'l', 'l', 'o', ' ', '1', '2' };
  // buf[0] = cnt;
  if(ack) {
    if (radio.sendWithRetry(TONODEID, sendbuf, 8)) {
      Serial.print("ACK received for "); Serial.print(cnt); 
      Serial.print(" rxvd/missed/err: "); 
        Serial.print(rxvd_counter); Serial.print("/"); Serial.print(missed_counter); Serial.print("/"); Serial.print(err_counter);
      Serial.print(" last rssi:"); Serial.println(rssi);
      rxvd_counter++;
    }
    else {
      Serial.print("ACK NOT received for "); Serial.print(cnt); 
      Serial.print(" rxvd/missed/err: "); 
        Serial.print(rxvd_counter); Serial.print("/"); Serial.print(missed_counter); Serial.print("/"); Serial.print(err_counter);
      Serial.print(" last rssi:"); Serial.println(rssi);
      missed_counter++;
    }
  }
  else {
    radio.send(TONODEID, sendbuf, 8);
    Serial.print("Sent pkt "); Serial.print(cnt); 
    Serial.print(" sent/err: "); 
      Serial.print(snt_counter); Serial.print("/"); Serial.print(err_counter);
    Serial.print(" last rssi:"); Serial.println(rssi);
    snt_counter++;
  }
  cnt++;
}

void rxvPkt()
{
  if (radio.receiveDone()) // Got one!
  {
    rssi = radio.RSSI;
    if (radio.ACKRequested())
    {
      Serial.print("rxvd from ");
      Serial.print(radio.SENDERID, DEC);
      Serial.print(" ");
      for (byte i = 0; i < radio.DATALEN; i++) {
        Serial.print(radio.DATA[i], HEX);
        Serial.print(" ");
      }
      for (byte i = 0; i < radio.DATALEN - 2; i++) {
        if(radio.DATA[i] != sendbuf[i]) {
          err_counter++;
          break;
        }
      }
      Serial.print(", RSSI:"); Serial.print(radio.RSSI);
      Serial.print(","); Serial.print(radio.RSSI2);
      Serial.println("");
      radio.sendACK();
    }
  }
}

void mydelay(int ms)
{
  uint32_t start = millis();
  while(millis() - start < ms) {}
}

