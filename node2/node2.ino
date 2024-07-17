// RFM69HCW Example Sketch
// Send serial input characters from one RFM69 node to another
// Based on RFM69 library sample code by Felix Rusu
// http://LowPowerLab.com/contact
// Modified for RFM69HCW by Mike Grusin, 4/16

// This sketch will show you the basics of using an
// RFM69HCW radio module. SparkFun's part numbers are:
// 915MHz: https://www.sparkfun.com/products/12775
// 434MHz: https://www.sparkfun.com/products/12823

// See the hook-up guide for wiring instructions:
// https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide

// Uses the RFM69 library by Felix Rusu, LowPowerLab.com
// Original library: https://www.github.com/lowpowerlab/rfm69
// SparkFun repository: https://github.com/sparkfun/RFM69HCW_Breakout

// Include the RFM69 and SPI libraries:

#include "DavisRFM69.h"
#include <SPI.h>

#include "common.h"
#include "prefs.h"

// Packet sent/received indicator LED (optional):

#define LED           9 // LED positive pin
#define GND           8 // LED ground pin

// Create a library object for our RFM69HCW module:

DavisRFM69 radio(5, 13, true);

void setup()
{
  // Open a serial port so we can send keystrokes to the module:

  delay(5000);
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("eConsole Version: 0.2");
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  

  // Set up the indicator LED (optional):

/*
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(GND,OUTPUT);
  digitalWrite(GND,LOW);
*/

  // Initialize the RFM69HCW:
  // radio.setCS(10);  //uncomment this if using Pro Micro
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  // Serial.print("RFM69 chip version: "); Serial.println(radio.chipVersion, HEX);
  radio.setChannel(0);

  // Turn on encryption if desired:

  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
}

void loop()
{
  static uint32_t st = 0;
#ifdef CONSOLE_DRIVEN_
  consoleSendRcv();
#else
  rxvPkt();
  if (millis() - st > 10000) {
    contSend(USEACK);
    st = millis();
  }
#endif
}

/*
int16_t rssi;
unsigned char cnt = 0;
uint32_t start = 0;
uint32_t rxvd_counter = 0;
uint32_t missed_counter = 0;
void contSend()
{
  static unsigned char buf[8] = { cnt, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22 };
  buf[0] = cnt;
  if (radio.sendWithRetry(TONODEID, buf, 8)) {
    Serial.print("ACK received for "); Serial.print(cnt); 
    Serial.print(" rxvd/missed: "); Serial.print(rxvd_counter); Serial.print("/"); Serial.print(missed_counter);
    Serial.print(" last rssi:"); Serial.println(rssi);
    rxvd_counter++;
  }
  else {
    Serial.print("ACK NOT received for "); Serial.print(cnt); 
    Serial.print(" rxvd/missed: "); Serial.print(rxvd_counter); Serial.print("/"); Serial.print(missed_counter);
    Serial.print(" last rssi:"); Serial.println(rssi);
    missed_counter++;
  }
  cnt++;
}

void rxvDone()
{
  if (radio.receiveDone()) // Got one!
  {
    rssi = radio.RSSI;
    if (radio.ACKRequested())
    {
      Serial.print("received from node ");
      Serial.print(radio.SENDERID, DEC);
      Serial.print(" ");
      for (byte i = 0; i < radio.DATALEN; i++) {
        Serial.print(radio.DATA[i], HEX);
        Serial.print(" ");
      }
      Serial.print(", RSSI ");
      Serial.println(radio.RSSI);
      radio.sendACK();
    }
  }
}

void loop()
{
#define PKTLOOP_ 1
#ifdef PKTLOOP_
  rxvDone();
  if (millis() - start > 2000) {
    contSend();
    start = millis();
  }
  delay(10);
  return;
#endif
  // Set up a "buffer" for characters that we'll send:

  static char sendbuffer[62];
  static int sendlength = 0;

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

    if ((input == '\r') || (sendlength == 61)) // CR or buffer full
    {
      // Send the packet!


      Serial.print("sending to node ");
      Serial.print(TONODEID, DEC);
      Serial.print(", message [");
      for (byte i = 0; i < sendlength; i++)
        Serial.print(sendbuffer[i]);
      Serial.print(" -- ");
      Serial.print(sendlength, DEC);
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

    Serial.print("received from node ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print(", message [");

    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:

    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);

    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.

    Serial.print("], RSSI ");
    Serial.println(radio.RSSI);

    // Send an ACK if requested.
    // (You don't need this code if you're not using ACKs.)

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.println("ACK sent");
    }
    // Blink(LED,10);
  }
}

void Blink(byte PIN, int DELAY_MS)
// Blink an LED for a given number of ms
{
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
*/
