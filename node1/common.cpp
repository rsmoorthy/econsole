#include "common.h"

#define PACKET_INTERVAL  2555

extern DavisRFM69 radio;

int16_t rssi;
unsigned char sendbuf[8] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x5A, 0xA5 };
char sendbuffer[8]; // 62
int sendlength = 0;
uint32_t snd_counter = 0;
uint32_t rxvd_counter = 0;
uint32_t err_counter = 0;
uint32_t missed_counter = 0;
uint8_t cnter = 0;
uint8_t exp_cnter = 0;

uint32_t cont_start_timer = 0;
uint32_t cont_end_timer = 0;
uint32_t cont_interval = 0;
uint32_t cont_last_sent = 0;
uint8_t hopCount = 0;
uint32_t chMissedPackets = 0;
uint32_t chNumResyncs = 0;
uint32_t chLastRxTime = 0;

// ISS Weather Values
float         g_windSpeed;                 // Windspeed [km/h]
uint16_t      g_windDirection;             // Directon of Wind [0-350°]
uint16_t      g_stationId;                 // Station Id
boolean       g_transmitterBatteryStatus;  // Battery Status: 0: OK, 1: Warning
float         g_goldcapChargeStatus;       // Goldcap Charge Status [V]     - msgID = 0x2
float         g_rainRate;                  // Rainrate [mm/h]               - msgID = 0x5
float         g_solarRadiation;            // Solar Radiation [?]           - msgID = 0x7
float         g_outsideTemperature;        // Outside Temperature [°C]      - msgID = 0x8
float         g_gustSpeed;                 // Gust Speed [km/h]             - msgID = 0x9
float         g_outsideHumidity;           // Outside Humidity [%rel]       - msgID = 0xa
uint16_t      g_rainClicks;                // Rainclicks received [0-127]   - msgID = 0xe
uint16_t      g_rainClicksLast;            // Last Rainclicks received
uint16_t      g_rainClicksDay;             // Rainclicks since last reset
unsigned long g_rainClicksSum;             // Rainclicks overall


void consoleSendRcv()
{
  // Set up a "buffer" for characters that we'll send:


  // SENDING

  // In this section, we'll gather serial characters and
  // send them to the other node if we (1) get a carriage return,
  // or (2) the buffer is full (61 characters).

  // If there is any serial input, add it to the buffer:

  if (Serial.available() > 0)
  {
    char input = Serial.read();

    if(input == 'c') { // start continuous sending
      cont_end_timer = millis() + 8 * 60 * 60 * 1000;
      cont_interval = PACKET_INTERVAL;
      cont_last_sent = 0;
      Serial.println("cont sending started");
    }
    else if(input == ' ') { // STOP all sending
      Serial.println("cont sending stopped");
      cont_end_timer = 0;
      cont_interval = 0;
      cont_last_sent = 0;
    }
    else if(input == 'I') { // print ISS data
      printIssData();
    }
    else if(input == 'R') { // read all registers
      radio.readAllRegs();
      Serial.println("Read all regs");
    }
    else if(input == 'r') { // reset all statistics
      cnter = exp_cnter = 0;
      snd_counter = rxvd_counter = err_counter = missed_counter = 0;
      hopCount = 0; chLastRxTime = 0;
      radio.setChannel(0);
      Serial.println("reset counters");
    }
    else if (input != '\r') // not a carriage return
    {
      sendbuffer[sendlength] = input;
      if(sendlength < 8) sendlength++;
    }

    // If the input is a carriage return, or the buffer is full:
    else if ((input == '\r')) // CR or buffer full  (was 61)
    {
      sendPkt();
      radio.hop();
    }
  }
  // send pkt
  uint32_t m = millis();
  if(cont_end_timer > m && cont_last_sent + cont_interval < m) {
    sendPkt();
    radio.hop();
    cont_last_sent = m;
  }
  hopTimer();

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
    Serial.print(") ch:"); Serial.print(radio.channel);
    Serial.print(" -> rcvd fm ");
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

    parseIssData();
    // hop channel
    chLastRxTime = millis();
    // radio.hop();
    // hopCount = 1;
  }
}

void sendPkt(void)
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
  Serial.print(") ch:"); Serial.print(radio.channel);
  Serial.print(" -> sent to ");
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

void hopTimer(void)
{
  // If a packet was not received at the expected time, hop the radio anyway
  // in an attempt to keep up.  Give up after 25 failed attempts.  Keep track
  // of packet stats as we go.  I consider a consecutive string of missed
  // packets to be a single resync.  Thx to Kobuki for this algorithm.
  if ((hopCount > 0) && ((millis() - chLastRxTime) > (hopCount * PACKET_INTERVAL + 200))) {
    chMissedPackets++;
    if (hopCount == 1) chNumResyncs++;
    if (++hopCount > 25) hopCount = 0;
    Serial.print("Retrying hop count: "); Serial.println(hopCount);
    radio.hop();
  }
}

#if 0
// Read the data from the ISS and figure out what to do with it
void processPacket() {
  // Every packet has wind speed, direction, and battery status in it
  loopData.windSpeed = radio.DATA[1];
#if 0
  Serial.print("Wind Speed: ");
  Serial.print(loopData.windSpeed);
  Serial.print("  Rx Byte 1: ");
  Serial.println(radio.DATA[1]);
#endif

  // There is a dead zone on the wind vane. No values are reported between 8
  // and 352 degrees inclusive. These values correspond to received byte
  // values of 1 and 255 respectively
  // See http://www.wxforum.net/index.php?topic=21967.50
  loopData.windDirection = 9 + radio.DATA[2] * 342.0f / 255.0f;

#if 0
  Serial.print(F("Wind Direction: "));
  Serial.print(loopData.windDirection);
  Serial.print(F("  Rx Byte 2: "));
  Serial.println(radio.DATA[2]);
#endif

  loopData.transmitterBatteryStatus = (radio.DATA[0] & 0x8) >> 3;
#if 0
  Serial.print("Battery status: ");
  Serial.println(loopData.transmitterBatteryStatus);
#endif

  // Now look at each individual packet. The high order nibble is the packet type.
  // The highest order bit of the low nibble is set high when the ISS battery is low.
  // The low order three bits of the low nibble are the station ID.

  switch (radio.DATA[0] >> 4) {
  case VP2P_TEMP:
    loopData.outsideTemperature = (int16_t)(word(radio.DATA[3], radio.DATA[4])) >> 4;
#if 1
    Serial.print(F("Outside Temp: "));
    Serial.print(loopData.outsideTemperature);
    Serial.print(F("  Rx Byte 3: "));
    Serial.print(radio.DATA[3]);
    Serial.print(F("  Rx Byte 4: "));
    Serial.println(radio.DATA[4]);
#endif
    break;
  case VP2P_HUMIDITY:
    loopData.outsideHumidity = (float)(word((radio.DATA[4] >> 4), radio.DATA[3])) / 10.0;
#if O
    Serial.print("Outside Humdity: ");
    Serial.print(loopData.outsideHumidity);
    Serial.print("  Rx Byte 3: ");
    Serial.print(radio.DATA[3]);
    Serial.print("  Rx Byte 4: ");
    Serial.println(radio.DATA[4]);
#endif
    break;
    // default:
  }
#if 0
  printFreeRam();
#endif
}

#endif

/************************************************************
 * Process the received RFM Data Packet
 * - Parse Databytes and store to g_ Variables
 * -
 ************************************************************/
void parseIssData() {
  uint16_t rawrr;
  float cph;
  byte msgID;
  uint16_t rainDiff;

  // *********************
  // wind speed (all packets)
  g_windSpeed = (float) radio.data(1) * 1.60934;

  // *********************
  // wind direction (all packets)
  // There is a dead zone on the wind vane. No values are reported between 8
  // and 352 degrees inclusive. These values correspond to received byte
  // values of 1 and 255 respectively
  // See http://www.wxforum.net/index.php?topic=21967.50
  // 0 = South
  g_windDirection = (uint16_t)(radio.data(2) * 360.0f / 255.0f);
  // convert to 180° = South
  if (g_windDirection >= 180) {
      g_windDirection -= 180;
  } else {
      g_windDirection += 180;
  }

  // *********************
  // battery status (all packets)
  g_transmitterBatteryStatus = (boolean)(radio.data(0) & 0x8) == 0x8;
  g_stationId = radio.data(0) & 0x7;

  // Now look at each individual packet. Mask off the four low order bits.
  // The highest order bit of these four bits is set high when the ISS battery is low.
  // The other three bits are the MessageID.
  msgID = (radio.data(0) & 0xf0) >>4 ;
  switch (msgID) {
    case 0x2:  // goldcap charge status (MSG-ID 2)
      g_goldcapChargeStatus = (float)((radio.data(3) << 2) + ((radio.data(4) & 0xC0) >> 6)) / 100;
      break;
    case 0x3:  // MSG ID 3: unknown - not used
      Serial.println("Message-ID 3: unknown");
      break;
    case 0x5:  // rain rate (MSG-ID 5) as number of rain clicks per hour
               // ISS will transmit difference between last two clicks in seconds
      if ( radio.data(3) == 255 ){
          // no rain
          g_rainRate = 0;
      } else {
        rawrr = radio.data(3) + ((radio.data(4) & 0x30) * 16);
        if ( (radio.data(4) & 0x40) == 0 ) {
          // HiGH rain rate
          // Clicks per hour = 3600 / (VALUE/16)
          cph = 57600 / (float) (rawrr);
        } else {
          // LOW rain rate
          // Clicks per hour = 3600 / VALUE
          cph = 3600 / (float) (rawrr);
        }
        // Rainrate [mm/h] = [Clicks/hour] * [Cupsize]
        g_rainRate = cph * 0.2;
      }
      break;
    case 0x7:  // solarRadiation (MSG-ID 7)
      g_solarRadiation = (float)((radio.data(3) * 4) + ((radio.data(4) & 0xC0) >> 6));
      break;
    case 0x8:  // outside temperature (MSG-ID 8)
      g_outsideTemperature = (float) (((radio.data(3) * 256 + radio.data(4)) / 160) -32) * 5 / 9;
      break;
    case 0x9:  // gust speed (MSG-ID 9), maximum wind speed in last 10 minutes - not used
      g_gustSpeed = (float) radio.data(3) * 1.60934;
      break;
    case 0xa:  // outside humidity (MSG-ID A)
      g_outsideHumidity = (float)(word((radio.data(4) >> 4), radio.data(3))) / 10.0;
      break;
    case 0xe:  // rain counter (MSG-ID E)
      g_rainClicks = (radio.data(3) & 0x7F);
      rainDiff = 0;
      // First run
      if (g_rainClicksLast == 255) {
        g_rainClicksLast = g_rainClicks;
      }
      if (g_rainClicks > g_rainClicksLast) {
        // Rainclicks higher than last time
        rainDiff = g_rainClicks - g_rainClicksLast;
      } else if (g_rainClicksLast > g_rainClicks) {
        // Rainclicks lower than last time (overflow)
        rainDiff = g_rainClicks + 128 - g_rainClicksLast;
      }
      g_rainClicksLast = g_rainClicks;
      g_rainClicksDay += rainDiff;
      g_rainClicksSum += rainDiff;
      break;
  }
  char tbuf[100];
  snprintf(tbuf, 99, "ISS data (stId:%d): wind sp:%d dir:%d bat:%d msgID:%x", 
              g_stationId, g_windSpeed, g_windDirection, g_transmitterBatteryStatus, msgID);
}

void printIssData(void)
{
  Serial.print("Windspeed:   ");
  Serial.println(g_windSpeed);

  Serial.print("WindDirection:   ");
  Serial.println(g_windDirection);

  Serial.print("Battery status:   ");
  Serial.println(g_transmitterBatteryStatus);

  Serial.print("Goldcap Charge Status: ");
  Serial.print(g_goldcapChargeStatus);
  Serial.println(" [V]");
  Serial.print("Rain rate: ");
  Serial.print(g_rainRate);
  Serial.println(" [mm/h]");
  Serial.print("Solar Radiation: ");
  Serial.println(g_solarRadiation);
  Serial.print("Outside Temp: ");
  Serial.print(g_outsideTemperature);
  Serial.println(" [C]");
  Serial.print("Gust Speed: ");
  Serial.print(g_gustSpeed);
  Serial.println(" [km/h]");
  Serial.print("Outside Humdity: ");
  Serial.print(g_outsideHumidity);
  Serial.println(" [%relH]");

  Serial.print("Rain Counter: ");
  Serial.print(g_rainClicks);
  Serial.println(" [clicks]");
  Serial.print("Dayly Rain Clicks: ");
  Serial.print(g_rainClicksDay);
  Serial.println(" [clicks]");
  Serial.print("Overall Rain Clicks: ");
  Serial.print(g_rainClicksSum);
  Serial.println(" [clicks]");
}


#if 0

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

#endif
