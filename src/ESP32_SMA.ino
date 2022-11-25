/*
 Talks to SMA Sunny Boy, Model SB 8000US-12
    - Gets AC power (only)
    - Sends the result to a custom LED bargraph

  Based on code found at https://github.com/stuartpittaway/nanodesmapvmonitor
 */

//Need to change SoftwareSerial/NewSoftSerial.h file to set buffer to 128 bytes or you will get buffer overruns!
//Find the line below and change
//#define _NewSS_MAX_RX_BUFF 128 // RX buffer size

#include "Arduino.h"
//#include <avr/pgmspace.h>
#include <ESP32Time.h>
#include "time.h"
//#include <NanodeMAC.h>
#include "bluetooth.h"
#include "SMANetArduino.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Update.h>
#include "site_details.h"
#include "debug.h"

#include "EspMQTTClient.h"

// A time in Unix Epoch that is "now" - used to check that the NTP server has
// sync'd the time correctly before updating the inverter
#define AFTER_NOW 1630152740

EspMQTTClient client(
    SSID,
    PASSWORD,
    MQTT_SERVER,
    "", // Can be omitted if not needed
    "", // Can be omitted if not needed
    HOST);

ESP32Time ESP32rtc;     // Time structure. Holds what time the ESP32 thinks it is.
ESP32Time nextMidnight; // Create a time structure to hold the answer to "What time (in time_t seconds) is the upcoming midnight?"

//BST Start and end dates - this needs moving into some sort of PROGMEM array for the years or calculated based on the BST logic see
//http://www.time.org.uk/bstgmtcodepage1.aspx
//static time_t SummerStart=1435888800;  //Sunday, 31 March 02:00:00 GMT
//static time_t SummerEnd=1425952800;  //Sunday, 27 October 02:00:00 GMT

//SMA inverter timezone (note inverter appears ignores summer time saving internally)
//Need to determine what happens when its a NEGATIVE time zone !
//Number of seconds for timezone
//    0=UTC (London)
//19800=5.5hours Chennai, Kolkata
//36000=Brisbane (UTC+10hrs)
#define timeZoneOffset (long)(60 * 60 * TIME_ZONE)

#define NaN_S32 (int32_t)0x80000000  // "Not a Number" representation for LONG (converted to 0 by SBFspot)
#define NaN_U32 (uint32_t)0xFFFFFFFF // "Not a Number" representation for ULONG (converted to 0 by SBFspot)

// #undef debugMsgln
// #define debugMsgln(s) (__extension__({ Serial.println(F(s)); }))
// #define debugMsgln(s) (__extension__({ __asm__("nop\n\t"); }))

// #undef debugMsg
// #define debugMsg(s) (__extension__({ Serial.print(F(s)); }))
// #define debugMsg(s) (__extension__({ __asm__("nop\n\t"); }))

/// Convert a uint64_t (unsigned long long) to a string.
/// Arduino String/toInt/Serial.print() can't handle printing 64 bit values.
/// @param[in] input The value to print
/// @param[in] base The output base.
/// @returns A String representation of the integer.
/// @note Based on Arduino's Print::printNumber()
String uint64ToString(uint64_t input, uint8_t base = 10)
{
  String result = "";
  // prevent issues if called with base <= 1
  if (base < 2)
    base = 10;
  // Check we have a base that we can actually print.
  // i.e. [0-9A-Z] == 36
  if (base > 36)
    base = 10;

  // Reserve some string space to reduce fragmentation.
  // 16 bytes should store a uint64 in hex text which is the likely worst case.
  // 64 bytes would be the worst case (base 2).
  result.reserve(16);

  do
  {
    char c = input % base;
    input /= base;

    if (c < 10)
      c += '0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}

bool blinklaststate;
void blinkLed()
{
  digitalWrite(LED_BUILTIN, blinklaststate);
  blinklaststate = !blinklaststate;
}

void blinkLedOff()
{
  if (blinklaststate)
  {
    blinkLed();
  }
}

//Do we switch off upload to sites when its dark?
#undef allowsleep

static uint64_t currentvalue = 0;
static unsigned int valuetype = 0;
static unsigned long value = 0;
static uint64_t value64 = 0;
static long lastRanTime = 0;
static long nowTime = 0;
// static unsigned long spotpowerac = 0;
static unsigned long spotpowerdc = 0;
// "datetime" stores the number of seconds since the epoch (normally 01/01/1970), AS RETRIEVED
//     from the SMA. The value is updated when data is read from the SMA, like when
//     getInstantACPower() is called.
//static unsigned long datetime=0;   // stores the number of seconds since the epoch (normally 01/01/1970)
time_t datetime = 0;

const unsigned long seventy_years = 2208988800UL;

prog_uchar PROGMEM smanet2packetx80x00x02x00[] = {0x80, 0x00, 0x02, 0x00};
prog_uchar PROGMEM smanet2packet2[] = {0x80, 0x0E, 0x01, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
prog_uchar PROGMEM SMANET2header[] = {0xFF, 0x03, 0x60, 0x65};
prog_uchar PROGMEM InverterCodeArray[] = {0x5c, 0xaf, 0xf0, 0x1d, 0x50, 0x00}; // Fake address on the SMA NETWORK
prog_uchar PROGMEM fourzeros[] = {0, 0, 0, 0};
prog_uchar PROGMEM smanet2packet6[] = {0x54, 0x00, 0x22, 0x26, 0x00, 0xFF, 0x22, 0x26, 0x00};
prog_uchar PROGMEM smanet2packet99[] = {0x00, 0x04, 0x70, 0x00};
prog_uchar PROGMEM smanet2packet0x01000000[] = {0x01, 0x00, 0x00, 0x00};

//Password needs to be 12 bytes long, with zeros as trailing bytes (Assume SMA INVERTER PIN code is 0000)
const unsigned char SMAInverterPasscode[] = {'0', '0', '0', '0', 0, 0, 0, 0, 0, 0, 0, 0};

// Function Prototypes
bool initialiseSMAConnection();
bool logonSMAInverter();
bool checkIfNeedToSetInverterTime();
bool getInstantACPower();
bool getTotalPowerGeneration();

// unsigned int lastprogress = -1;
// void setupOTAServer()
// {
//   server.on("/", HTTP_GET, []()
//             {
//               server.sendHeader("Connection", "close");
//               server.send(200, "text/plain", "login index");
//             });
//   server.on("/reboot", HTTP_GET, []()
//             {
//               debugMsgLn("Reboot requested!");
//               server.sendHeader("Connection", "close");
//               server.send(200, "text/plain", "Rebooting...");
//               ESP.restart();
//             });
//   /*handling uploading firmware file */
//   server.on(
//       "/update", HTTP_POST, []()
//       {
//         server.sendHeader("Connection", "close");
//         server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
//         ESP.restart();
//       },
//       []()
//       {
//         HTTPUpload &upload = server.upload();
//         Update.onProgress([](unsigned int progress, unsigned int total)
//                           {
//                             unsigned int pcg = progress / (total / 100);
//                             Serial.printf("Progress: %u%%\r", pcg);
//                             // Only update network every 10%
//                             if ((pcg / 5) != (lastprogress / 5))
//                             {
//                               lastprogress = pcg;
//                               debugMsg("Progress: ");
//                               debugMsg(String(pcg));
//                               debugMsgLn("%");
//                             }
//                             blinkLed();
//                           });
//         if (upload.status == UPLOAD_FILE_START)
//         {
//           debugMsg("Update: ");
//           debugMsgLn(upload.filename.c_str());
//           if (!Update.begin(UPDATE_SIZE_UNKNOWN))
//           { //start with max available size
//             Update.printError(Serial);
//           }
//         }
//         else if (upload.status == UPLOAD_FILE_WRITE)
//         {
//           /* flashing firmware to ESP*/
//           if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
//           {
//             Update.printError(Serial);
//           }
//         }
//         else if (upload.status == UPLOAD_FILE_END)
//         {
//           if (Update.end(true))
//           { //true to set the size to the current progress
//             debugMsg("Update Success!");
//             Serial.printf(" Bytes: %u", upload.totalSize);
//             debugMsgLn("\nRebooting...");
//           }
//           else
//           {
//             Update.printError(Serial);
//           }
//         }
//       });
//   server.begin();
// }

void onConnectionEstablished()
{
  client.publish(MQTT_BASE_TOPIC "LWT", "online", true);
  debugMsgLn("WiFi and MQTT connected");
  debugMsgLn("v1 Build time: " __TIME__ " date: " __DATE__);

#ifdef PUBLISH_HASS_TOPICS
  // client.publish(MQTT_BASE_TOPIC "LWT", "online", true);
  client.publish("homeassistant/sensor/" HOST "/signal/config", "{\"name\": \"" FRIENDLY_NAME " Signal Strength\", \"state_topic\": \"" MQTT_BASE_TOPIC "signal\", \"unique_id\": \"" HOST "-signal\", \"unit_of_measurement\": \"dB\", \"device\": {\"identifiers\": [\"" HOST "-device\"], \"name\": \"" FRIENDLY_NAME "\"}}", true);
  client.publish("homeassistant/sensor/" HOST "/generation_today/config", "{\"name\": \"" FRIENDLY_NAME " Power Generation Today\", \"device_class\": \"energy\", \"state_topic\": \"" MQTT_BASE_TOPIC "generation_today\", \"unique_id\": \"" HOST "-generation_today\", \"unit_of_measurement\": \"Wh\", \"state_class\": \"total_increasing\", \"device\": {\"identifiers\": [\"" HOST "-device\"]} }", true);
  client.publish("homeassistant/sensor/" HOST "/generation_total/config", "{\"name\": \"" FRIENDLY_NAME " Power Generation Total\", \"device_class\": \"energy\", \"state_topic\": \"" MQTT_BASE_TOPIC "generation_total\", \"unique_id\": \"" HOST "-generation_total\", \"unit_of_measurement\": \"Wh\", \"state_class\": \"total_increasing\", \"device\": {\"identifiers\": [\"" HOST "-device\"]} }", true);
  client.publish("homeassistant/sensor/" HOST "/instant_ac/config", "{\"name\": \"" FRIENDLY_NAME " Instantinous AC Power\", \"device_class\": \"energy\", \"state_topic\": \"" MQTT_BASE_TOPIC "instant_ac\", \"unique_id\": \"" HOST "-instant_ac\", \"unit_of_measurement\": \"W\", \"state_class\": \"measurement\", \"device\": {\"identifiers\": [\"" HOST "-device\"]} }", true);

// mosquitto_pub -h core.sf -t homeassistant/sensor/sma-monitor/generation_today/config -m '{ "name": "Power Generation Today", "device_class": "energy", "state_topic": "sma/solar/generation_today", "unique_id": "sma-monitor-generation_today", "unit_of_measurement": "Wh", "state_class": "total_increasing", "device": {"identifiers": ["sma-monitor-device"]} }'
// mosquitto_pub -h core.sf -t homeassistant/sensor/sma-monitor/generation_total/config -m '{ "name": "Power Generation Total", "device_class": "energy", "state_topic": "sma/solar/generation_total", "unique_id": "sma-monitor-generation_total", "unit_of_measurement": "Wh", "state_class": "total_increasing", "device": {"identifiers": ["sma-monitor-device"]} }'
// mosquitto_pub -h core.sf -t homeassistant/sensor/sma-monitor/instant_ac/config -m '{ "name": "Instant AC Power", "device_class": "power", "state_topic": "sma/solar/instant_ac", "unique_id": "sma-monitor-instant_ac", "unit_of_measurement": "W", "state_class": "measurement", "device": {"identifiers": ["sma-monitor-device"]} }'

// mosquitto_pub -h core.sf -t homeassistant/sensor/sma-monitor/signal/config -m '{ "name": "Signal Strength", "state_topic": "sma/solar/signal", "unique_id": "sma-monitor-signal", "unit_of_measurement": "dB", "device": {"identifiers": ["sma-monitor-device"], "name": "SMA Inverter"} }'
#endif // PUBLISH_HASS_TOPICS
}

void setup()
{
  // Get the MAC address in reverse order (not sure why, but do it here to make setup easier)
  unsigned char smaSetAddress[6] = {SMA_ADDRESS};
  for (int i = 0; i < 6; i++)
    smaBTInverterAddressArray[i] = smaSetAddress[5 - i];

  pinMode(LED_BUILTIN, OUTPUT);
  debugSetup();

  Serial.begin(115200);                      //Serial port for debugging output
  ESP32rtc.setTime(30, 24, 15, 17, 1, 2021); // 17th Jan 2021 15:24:30  // Need this to be accurate. Since connecting to the internet anyway, use NTP.

  // Connect to WiFi network
  // WiFi.begin(SSID, PASSWORD);
  // Serial.println("");

  // // Wait for connection
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   blinkLed();
  //   delay(500);
  //   Serial.print(".");
  // }

  client.setMaxPacketSize(512); // must be big enough to send home assistant config
  client.enableMQTTPersistence();
  client.enableDebuggingMessages();                                     // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater("/update");                               // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
  client.enableLastWillMessage(MQTT_BASE_TOPIC "LWT", "offline", true); // You can activate the retain flag by setting the third parameter to true

  debugMsgLn("");
  debugMsg("Connected to ");
  debugMsgLn(SSID);
  debugMsg("IP address: ");
  debugMsgLn(WiFi.localIP().toString());
  // Serial.println(WiFi.localIP());

  // Always set time to GMT timezone
  configTime(timeZoneOffset, 0, NTP_SERVER);

  // setupOTAServer();
}

void everySecond()
{
  blinkLedOff();

  // debugMsg("Connection is: ");
  // if (client.isConnected())
  //   debugMsgLn("connected");
  // else
  //   debugMsgLn("DISconnected");

  // client.publish(MQTT_BASE_TOPIC "hello", "message");

  // debugMsg("Current Time: ");
  // char timeStr[22];
  // struct tm timeinfo;
  // getLocalTime(&timeinfo);
  // strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  // debugMsgLn(timeStr);
}

void every5Minutes()
{
  if (client.isConnected())
  {
    client.publish(MQTT_BASE_TOPIC "signal", String(WiFi.RSSI()));
  }
}

unsigned long sleepuntil = 0;
void dodelay(unsigned long duration)
{
  sleepuntil = millis() + duration;
}

uint8_t mainstate = 0;
uint8_t innerstate = 0;
unsigned long nextSecond = 0;
unsigned long next5Minute = 0;
int thisminute = -1;
int checkbtminute = -1;

void loop()
{
  struct tm timeinfo;
  client.loop();
  // server.handleClient();

  if (millis() >= nextSecond)
  {
    nextSecond = millis() + 1000;
    everySecond();
  }

  if (millis() >= next5Minute)
  {
    next5Minute = millis() + 1000 * 60 * 5;
    every5Minutes();
  }

  // "delay" the main BT loop
  if (millis() < sleepuntil)
    return;

  // if in the main loop, only run at the top of the minute
  if (mainstate >= 5)
  {
    getLocalTime(&timeinfo);
    if (timeinfo.tm_min == thisminute)
      return;
    // Check we are connected to BT, if not, restart process
    if (!BTCheckConnected())
    {
      mainstate = 0;
    }
  }

  // Wait for initial NTP sync before setting up inverter
  if (mainstate == 0 && ESP32rtc.getEpoch() < AFTER_NOW)
  {
    debugMsgLn("NTP not yet sync'd, sleeping");
    dodelay(2000);
  }

  // Only bother to do anything if we are connected to WiFi and MQTT
  if (!client.isConnected())
    return;

  // Run the "main" BT loop
  blinkLed();
  switch (mainstate)
  {
  case 0:
    if (BTStart())
    {
      debugMsgLn("Next: Init connection...");
      mainstate++;
      innerstate = 0;
    }
    else
    {
      dodelay(5000);
    }
    break;

  case 1:
    if (initialiseSMAConnection())
    {
      debugMsgLn("Next: Logon...");
      mainstate++;
      innerstate = 0;
    }
    break;

    //Dont really need this...
    //InquireBlueToothSignalStrength();

  case 2:
    if (logonSMAInverter())
    {
      debugMsgLn("Next: getDailyYield...");
      mainstate++;
      innerstate = 0;
    }
    break;

  case 3:
    // Doing this to set datetime
    if (getDailyYield())
    {
      debugMsgLn("Next: Set Time...");
      mainstate++;
      innerstate = 0;
    }
    break;

  case 4:
    if (checkIfNeedToSetInverterTime())
    {
      debugMsgLn("Next: Default...");
      mainstate++;
      innerstate = 0;
    }
    break;

    // --------- regular loop -----------------

  case 5:
    if (getInstantACPower())
    {
      mainstate++;
      innerstate = 0;
    }
    break;

  case 6:
    if (getInstantDCPower())
    {
      mainstate++;
      innerstate = 0;
    }
    break;

  case 7:
    if (getDailyYield())
    {
      mainstate++;
      innerstate = 0;
    }
    break;

  case 8:
    if (getTotalPowerGeneration())
    {
      mainstate++;
      innerstate = 0;
    }
    break;

  default:
    mainstate = 5;
    innerstate = 0;
    thisminute = timeinfo.tm_min;
  }
  return;

  //getInverterName();
  //HistoricData();
  lastRanTime = millis() - 4000;

  debugMsgLn("While...");
  while (1)
  {
    //debugMsgln("Main loop");

    //HowMuchMemory();

    //Populate datetime and spotpowerac variables with latest values
    //getInstantDCPower();
    nowTime = millis();
    if (nowTime > (lastRanTime + 4000))
    {
      lastRanTime = nowTime;
      Serial.print("^");
      getInstantACPower();
    }

    //digitalClockDisplay(now());
    //debugMsgln("");

    //The inverter always runs in UTC time (and so does this program!), and ignores summer time, so fix that here...
    //add 1 hour to readings if its summer
    // DRH Temp removal of: if ((datetime>=SummerStart) && (datetime<=SummerEnd)) datetime+=60*60;

#ifdef allowsleep
    if ((ESP32rtc.getEpoch() > (datetime + 3600)) && (spotpowerac == 0))
    {
      //Inverter output hasnt changed for an hour, so put Nanode to sleep till the morning
      //debugMsgln("Bed time");

      //sleeping=true;

      //Get midnight on the day of last solar generation
      // First, create a time structure to hold the answer to "At what time (in time_t seconds) is the upcoming midnight?"
      tmElements_t tm;
      tm.Year = year(datetime) - 1970;
      tm.Month = month(datetime);
      tm.Day = day(datetime);
      tm.Hour = 23;
      tm.Hour = hour(datetime);
      tm.Minute = 59;
      tm.Second = 59;
      time_t midnight = makeTime(tm);

      //Move to midnight
      //debugMsg("Midnight ");digitalClockDisplay( midnight );debugMsgln("");

      if (ESP32rtc.getEpoch() < midnight)
      {
        //Time to calculate SLEEP time, we only do this if its BEFORE midnight
        //on the day of solar generation, otherwise we might wake up and go back to sleep immediately!

        //Workout what time sunrise is and sleep till then...
        //longitude, latitude (london uk=-0.126236,51.500152)
        unsigned int minutespastmidnight = ComputeSun(mylongitude, mylatitude, datetime, true);

        //We want to wake up at least 15 minutes before sunrise, just in case...
        checktime = midnight + minutespastmidnight - 15 * 60;
      }
    }
#endif

    //debugMsg("Wait for ");
    //digitalClockDisplay( checktime );
    //debugMsgln("");

    //Delay for approx. 4 seconds between instant AC power readings

  } // end of while(1)
} // end of loop()

int32_t get_long(unsigned char *buf)
{
  int32_t lng = 0;

  memcpy(&lng, buf, 4);

  if ((lng == (int32_t)NaN_S32) || (lng == (int32_t)NaN_U32))
    lng = 0;
  return lng;
}

//-------------------------------------------------------------------------------------------
bool checkIfNeedToSetInverterTime()
{
  //digitalClockDisplay(now());Serial.println("");
  //digitalClockDisplay(datetime);Serial.println("");

  unsigned long timediff;

  timediff = abs(datetime - ESP32rtc.getEpoch());
  debugMsg("Time diff: ");
  debugMsgLn(String(timediff));
  debugMsg("datetime: ");
  debugMsgLn(String(datetime));
  debugMsg("epoch: ");
  debugMsgLn(String(ESP32rtc.getEpoch()));

  if (timediff > 60)
  {
    //If inverter clock is out by more than 1 minute, set it to the time from NTP, saves filling up the
    //inverters event log with hundred of "change time" lines...
    setInverterTime(); //Set inverter time to now()
  }

  return true;
}

prog_uchar PROGMEM smanet2settime[] = {
    0x8c, 0x0a, 0x02, 0x00, 0xf0, 0x00, 0x6d, 0x23, 0x00, 0x00, 0x6d, 0x23, 0x00, 0x00, 0x6d, 0x23, 0x00};

void setInverterTime()
{
  //Sets inverter time for those SMA inverters which don't have a realtime clock (Tripower 8000 models for instance)

  //Payload...

  //** 8C 0A 02 00 F0 00 6D 23 00 00 6D 23 00 00 6D 23 00
  //   9F AE 99 4F   ==Thu, 26 Apr 2012 20:22:55 GMT  (now)
  //   9F AE 99 4F   ==Thu, 26 Apr 2012 20:22:55 GMT  (now)
  //   9F AE 99 4F   ==Thu, 26 Apr 2012 20:22:55 GMT  (now)
  //   01 00         ==Timezone +1 hour for BST ?
  //   00 00
  //   A1 A5 99 4F   ==Thu, 26 Apr 2012 19:44:33 GMT  (strange date!)
  //   01 00 00 00
  //   F3 D9         ==Checksum
  //   7E            ==Trailer

  //Set time to Feb

  //2A 20 63 00 5F 00 B1 00 0B FF B5 01
  //7E 5A 00 24 A3 0B 50 DD 09 00 FF FF FF FF FF FF 01 00
  //7E FF 03 60 65 10 A0 FF FF FF FF FF FF 00 00 78 00 6E 21 96 37 00 00 00 00 00 00 01
  //** 8D 0A 02 00 F0 00 6D 23 00 00 6D 23 00 00 6D 23 00
  //14 02 2B 4F ==Thu, 02 Feb 2012 21:37:24 GMT
  //14 02 2B 4F ==Thu, 02 Feb 2012 21:37:24 GMT
  //14 02 2B 4F  ==Thu, 02 Feb 2012 21:37:24 GMT
  //00 00        ==No time zone/BST not applicable for Feb..
  //00 00
  //AD B1 99 4F  ==Thu, 26 Apr 2012 20:35:57 GMT
  //01 00 00 00
  //F6 87        ==Checksum
  //7E

  //2A 20 63 00 5F 00 B1 00 0B FF B5 01
  //7E 5A 00 24 A3 0B 50 DD 09 00 FF FF FF FF FF FF 01 00
  //7E FF 03 60 65 10 A0 FF FF FF FF FF FF 00 00 78 00 6E 21 96 37 00 00 00 00 00 00 1C
  //** 8D 0A 02 00 F0 00 6D 23 00 00 6D 23 00 00 6D 23 00
  //F5 B3 99 4F
  //F5 B3 99 4F
  //F5 B3 99 4F 01 00 00 00 28 B3 99 4F 01 00 00 00
  //F3 C7 7E

  //2B 20 63 00 5F 00 DD 00 0B FF B5 01
  //7E 5A 00 24 A3 0B 50 DD 09 00 FF FF FF FF FF FF 01 00
  //7E FF 03 60 65 10 A0 FF FF FF FF FF FF 00 00 78 00 6E 21 96 37 00 00 00 00 00 00 08
  //** 80 0A 02 00 F0 00 6D 23 00 00 6D 23 00 00 6D 23 00
  //64 76 99 4F ==Thu, 26 Apr 2012 16:23:00 GMT
  //64 76 99 4F ==Thu, 26 Apr 2012 16:23:00 GMT
  //64 76 99 4F  ==Thu, 26 Apr 2012 16:23:00 GMT
  //58 4D   ==19800 seconds = 5.5 hours
  //00 00
  //62 B5 99 4F
  //01 00 00 00
  //C3 27 7E

  debugMsgLn("setInverterTime");
  time_t currenttime = ESP32rtc.getEpoch(); // Returns the ESP32 RTC in number of seconds since the epoch (normally 01/01/1970)
  //digitalClockDisplay(currenttime);
  writePacketHeader(level1packet);
  writeSMANET2PlusPacket(level1packet, 0x09, 0x00, packet_send_counter, 0, 0, 0);
  writeSMANET2ArrayFromProgmem(level1packet, smanet2settime, sizeof(smanet2settime));
  writeSMANET2Long(level1packet, currenttime);
  writeSMANET2Long(level1packet, currenttime);
  writeSMANET2Long(level1packet, currenttime);
  // writeSMANET2Long(level1packet, timeZoneOffset);
  writeSMANET2uint(level1packet, timeZoneOffset);
  writeSMANET2uint(level1packet, 0);
  writeSMANET2Long(level1packet, currenttime); //No idea what this is for...
  writeSMANET2ArrayFromProgmem(level1packet, smanet2packet0x01000000, sizeof(smanet2packet0x01000000));
  writeSMANET2PlusPacketTrailer(level1packet);
  writePacketLength(level1packet);
  sendPacket(level1packet);
  packet_send_counter++;
  //debugMsgln(" done");
}

prog_uchar PROGMEM smanet2totalyieldWh[] = {
    0x54, 0x00, 0x01, 0x26, 0x00, 0xFF, 0x01, 0x26, 0x00};

bool initialiseSMAConnection()
{
  // debugMsg("initialiseSMAConnection stage: ");
  // debugMsgLn(String(innerstate));

  unsigned char netid;
  switch (innerstate)
  {
  case 0:
    //Wait for announcement/broadcast message from PV inverter
    if (getPacket(0x0002))
      innerstate++;
    break;

  case 1:
    //Extract data from the 0002 packet
    netid = level1packet[4];

    // Now create a response and send it.
    for (int i = 0; i < sizeof(level1packet); i++)
      level1packet[i] = 0x00;

    writePacketHeader(level1packet, 0x02, 0x00, smaBTInverterAddressArray);
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packet99, sizeof(smanet2packet99));
    writeSMANET2SingleByte(level1packet, netid);
    writeSMANET2ArrayFromProgmem(level1packet, fourzeros, sizeof(fourzeros));
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packet0x01000000, sizeof(smanet2packet0x01000000));
    writePacketLength(level1packet);
    sendPacket(level1packet);
    innerstate++;
    break;

  case 2:
    // The SMA inverter will respond with a packet carrying the command '0x000A'.
    // It will return with cmdcode set to 0x000A.
    if (getPacket(0x000a))
      innerstate++;
    break;

  case 3:
    // The SMA inverter will now send two packets, one carrying the '0x000C' command, then the '0x0005' command.
    // Sit in the following loop until you get one of these two packets.
    cmdcode = readLevel1PacketFromBluetoothStream(0);
    if ((cmdcode == 0x000c) || (cmdcode == 0x0005))
      innerstate++;
    break;

  case 4:
    // If the most recent packet was command code = 0x0005 skip this next line, otherwise, wait for 0x0005 packet.
    // Since the first SMA packet after a 0x000A packet will be a 0x000C packet, you'll probably sit here waiting at least once.
    if (cmdcode == 0x0005)
    {
      innerstate++;
    }
    else
    {
      if (getPacket(0x0005))
        innerstate++;
    }
    break;

  case 5:
    //First SMANET2 packet
    writePacketHeader(level1packet, sixff);
    writeSMANET2PlusPacket(level1packet, 0x09, 0xa0, packet_send_counter, 0, 0, 0);
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packetx80x00x02x00, sizeof(smanet2packetx80x00x02x00));
    writeSMANET2SingleByte(level1packet, 0x00);
    writeSMANET2ArrayFromProgmem(level1packet, fourzeros, sizeof(fourzeros));
    writeSMANET2ArrayFromProgmem(level1packet, fourzeros, sizeof(fourzeros));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);
    sendPacket(level1packet);

    if (getPacket(0x0001) && validateChecksum())
    {
      innerstate++;
      packet_send_counter++;
    }
    break;

  case 6:
    //Second SMANET2 packet
    writePacketHeader(level1packet, sixff);
    writeSMANET2PlusPacket(level1packet, 0x08, 0xa0, packet_send_counter, 0x00, 0x03, 0x03);
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packet2, sizeof(smanet2packet2));

    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);
    sendPacket(level1packet);
    packet_send_counter++;

    innerstate++;
    break;

  default:
    return true;
  }

  return false;
}

prog_uchar PROGMEM smanet2packet_logon[] = {
    0x80, 0x0C, 0x04, 0xFD, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x84, 0x03, 0x00, 0x00, 0xaa, 0xaa, 0xbb, 0xbb};

bool logonSMAInverter()
{
  // debugMsg("logonSMAInverter stage: ");
  // debugMsgLn(String(innerstate));

  //Third SMANET2 packet
  switch (innerstate)
  {
  case 0:
    writePacketHeader(level1packet, sixff);
    writeSMANET2PlusPacket(level1packet, 0x0e, 0xa0, packet_send_counter, 0x00, 0x01, 0x01);
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packet_logon, sizeof(smanet2packet_logon));
    writeSMANET2ArrayFromProgmem(level1packet, fourzeros, sizeof(fourzeros));

    //INVERTER PASSWORD
    for (int passcodeloop = 0; passcodeloop < sizeof(SMAInverterPasscode); passcodeloop++)
    {
      unsigned char v = pgm_read_byte(SMAInverterPasscode + passcodeloop);
      writeSMANET2SingleByte(level1packet, (v + 0x88) % 0xff);
    }

    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);
    sendPacket(level1packet);

    innerstate++;
    break;

  case 1:
    if (getPacket(0x0001) && validateChecksum())
    {
      innerstate++;
      packet_send_counter++;
    }
    break;

  default:
    return true;
  }

  return false;
}

bool getDailyYield()
{
  //We expect a multi packet reply to this question...
  //We ask the inverter for its DAILY yield (generation)
  //once this is returned we can extract the current date/time from the inverter and set our internal clock
  // debugMsg("getDailyYield stage: ");
  // debugMsgLn(String(innerstate));

  switch (innerstate)
  {
  case 0:
    writePacketHeader(level1packet);
    //writePacketHeader(level1packet,0x01,0x00,smaBTInverterAddressArray);
    writeSMANET2PlusPacket(level1packet, 0x09, 0xa0, packet_send_counter, 0, 0, 0);
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packetx80x00x02x00, sizeof(smanet2packetx80x00x02x00));
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packet6, sizeof(smanet2packet6));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);

    sendPacket(level1packet);

    innerstate++;
    break;

  case 1:
    if (getPacket(0x0001))
    {
      if (validateChecksum())
      {
        packet_send_counter++;
        innerstate++;
      }
      else
        innerstate = 0;
    }
    break;

  case 2:
    //Returns packet looking like this...
    //    7E FF 03 60 65 0D 90 5C AF F0 1D 50 00 00 A0 83
    //    00 1E 6C 5D 7E 00 00 00 00 00 00 03
    //    80 01 02 00
    //    54 01 00 00 00 01 00 00 00 01
    //    22 26  //command code 0x2622 daily yield
    //    00     //unknown
    //    D6 A6 99 4F  //Unix time stamp (backwards!) = 1335469782 = Thu, 26 Apr 2012 19:49:42 GMT
    //    D9 26 00     //Daily generation 9.945 kwh
    //    00
    //    00 00 00 00
    //    18 61    //checksum
    //    7E       //packet trailer

    // Does this packet contain the British Summer time flag?
    //dumpPacket('Y');

    valuetype = level1packet[40 + 1 + 1] + level1packet[40 + 2 + 1] * 256;

    //Serial.println(valuetype,HEX);
    //Make sure this is the right message type
    if (valuetype == 0x2622)
    {
      memcpy(&value64, &level1packet[40 + 8 + 1], 8);
      //0x2622=Day Yield Wh
      // memcpy(&datetime, &level1packet[40 + 4 + 1], 4);
      datetime = get_long(level1packet + 40 + 1 + 4);
      // debugMsg("Current Time (epoch): ");
      // debugMsgLn(String(datetime));

      //setTime(datetime);
      currentvalue = value64;
      client.publish(MQTT_BASE_TOPIC "generation_today", uint64ToString(currentvalue), true);
      debugMsg("Day Yield: ");
      debugMsgLn(String((double)value64 / 1000));
    }
    innerstate++;
    break;

  default:
    return true;
  }

  return false;
}

prog_uchar PROGMEM smanet2acspotvalues[] = {
    0x51, 0x00, 0x3f, 0x26, 0x00, 0xFF, 0x3f, 0x26, 0x00, 0x0e};

bool getInstantACPower()
{
  int32_t thisvalue;
  //Get spot value for instant AC wattage
  // debugMsg("getInstantACPower stage: ");
  // debugMsgLn(String(innerstate));

  switch (innerstate)
  {
  case 0:
    writePacketHeader(level1packet);
    //writePacketHeader(level1packet,0x01,0x00,smaBTInverterAddressArray);
    writeSMANET2PlusPacket(level1packet, 0x09, 0xA1, packet_send_counter, 0, 0, 0);
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packetx80x00x02x00, sizeof(smanet2packetx80x00x02x00));
    writeSMANET2ArrayFromProgmem(level1packet, smanet2acspotvalues, sizeof(smanet2acspotvalues));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);

    sendPacket(level1packet);
    innerstate++;
    break;

  case 1:
    if (waitForMultiPacket(0x0001))
    {
      if (validateChecksum())
      {
        packet_send_counter++;
        innerstate++;
      }
      else
        innerstate = 0;
    }
    break;

  case 2:
    //value will contain instant/spot AC power generation along with date/time of reading...
    // memcpy(&datetime, &level1packet[40 + 1 + 4], 4);
    datetime = get_long(level1packet + 40 + 1 + 4);
    thisvalue = get_long(level1packet + 40 + 1 + 8);
    // memcpy(&thisvalue, &level1packet[40 + 1 + 8], 4);

    currentvalue = thisvalue;
    client.publish(MQTT_BASE_TOPIC "instant_ac", uint64ToString(currentvalue), true);

    debugMsg("AC ");
    //Serial.println(" ");
    //Serial.println("Got AC power level. ");
    //digitalClockDisplay(datetime);
    debugMsg(" Pwr=");
    //if( value != oldvalue ) {
    //Serial.println(" ");
    //Serial.print("*** Power Level = ");
    debugMsgLn(String(thisvalue));
    //Serial.println(" Watts RMS ***");
    //Serial.print(" ");
    //}
    // spotpowerac = value;

    //displaySpotValues(28);
    innerstate++;
    break;

  default:
    return true;
  }

  return false;
}

bool getTotalPowerGeneration()
{

  //Gets the total kWh the SMA inverter has generated in its lifetime...
  // debugMsg("getTotalPowerGeneration stage: ");
  // debugMsgLn(String(innerstate));

  switch (innerstate)
  {
  case 0:
    writePacketHeader(level1packet);
    writeSMANET2PlusPacket(level1packet, 0x09, 0xa0, packet_send_counter, 0, 0, 0);
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packetx80x00x02x00, sizeof(smanet2packetx80x00x02x00));
    writeSMANET2ArrayFromProgmem(level1packet, smanet2totalyieldWh, sizeof(smanet2totalyieldWh));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);

    sendPacket(level1packet);
    innerstate++;
    break;

  case 1:
    if (waitForMultiPacket(0x0001))
    {
      if (validateChecksum())
      {
        packet_send_counter++;
        innerstate++;
      }
      else
        innerstate = 0;
    }
    break;

  case 2:
    //displaySpotValues(16);
    memcpy(&datetime, &level1packet[40 + 1 + 4], 4);
    memcpy(&value64, &level1packet[40 + 1 + 8], 8);
    //digitalClockDisplay(datetime);
    debugMsg("Total Power: ");
    debugMsgLn(String((double)value64 / 1000));
    currentvalue = value64;
    client.publish(MQTT_BASE_TOPIC "generation_total", uint64ToString(currentvalue), true);
    innerstate++;
    break;

  default:
    return true;
  }

  return false;
}

//Returns volts + amps
//prog_uchar PROGMEM smanet2packetdcpower[]={  0x83, 0x00, 0x02, 0x80, 0x53, 0x00, 0x00, 0x45, 0x00, 0xFF, 0xFF, 0x45, 0x00 };
// Just DC Power (watts)
prog_uchar PROGMEM smanet2packetdcpower[] = {
    0x83, 0x00, 0x02, 0x80, 0x53, 0x00, 0x00, 0x25, 0x00, 0xFF, 0xFF, 0x25, 0x00};
bool getInstantDCPower()
{
  // This appears broken...
  return true;

  //DC
  //We expect a multi packet reply to this question...
  debugMsg("getInstantDCPower stage: ");
  debugMsgLn(String(innerstate));

  switch (innerstate)
  {
  case 0:

    writePacketHeader(level1packet);
    //writePacketHeader(level1packet,0x01,0x00,smaBTInverterAddressArray);
    writeSMANET2PlusPacket(level1packet, 0x09, 0xE0, packet_send_counter, 0, 0, 0);
    // writeSMANET2ArrayFromProgmem(level1packet, smanet2packetx80x00x02x00, sizeof(smanet2packetx80x00x02x00));
    writeSMANET2ArrayFromProgmem(level1packet, smanet2packetdcpower, sizeof(smanet2packetdcpower));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);

    sendPacket(level1packet);
    innerstate++;
    break;

  case 1:
    if (waitForMultiPacket(0x0001))
    {
      if (validateChecksum())
      {
        packet_send_counter++;
        innerstate++;
      }
      else
        innerstate = 0;
    }
    break;

  case 2:

    //displaySpotValues(28);

    //float volts=0;
    //float amps=0;

    for (int i = 40 + 1; i < packetposition - 3; i += 28)
    {
      valuetype = level1packet[i + 1] + level1packet[i + 2] * 256;
      memcpy(&value, &level1packet[i + 8], 4);

      //valuetype
      //0x451f=DC Voltage  /100
      //0x4521=DC Current  /1000
      //0x251e=DC Power /1
      //if (valuetype==0x451f) volts=(float)value/(float)100;
      //if (valuetype==0x4521) amps=(float)value/(float)1000;
      if (valuetype == 0x251e)
        spotpowerdc = value;

      memcpy(&datetime, &level1packet[i + 4], 4);
    }

    //spotpowerdc=volts*amps;

    debugMsg("DC ");
    //digitalClockDisplay(datetime);
    //debugMsg(" V=");Serial.print(volts);debugMsg("  A=");Serial.print(amps);
    debugMsg(" Pwr=");
    debugMsgLn(String(spotpowerdc));
    innerstate++;
    break;

  default:
    return true;
  }

  return false;
}

/*
//Inverter name
 prog_uchar PROGMEM smanet2packetinvertername[]={   0x80, 0x00, 0x02, 0x00, 0x58, 0x00, 0x1e, 0x82, 0x00, 0xFF, 0x1e, 0x82, 0x00};  
 
 void getInverterName() {
 
 do {
 //INVERTERNAME
 debugMsgln("InvName"));
 writePacketHeader(level1packet,sixff);
 //writePacketHeader(level1packet,0x01,0x00,sixff);
 writeSMANET2PlusPacket(level1packet,0x09, 0xa0, packet_send_counter, 0, 0, 0);
 writeSMANET2ArrayFromProgmem(level1packet,smanet2packetinvertername);
 writeSMANET2PlusPacketTrailer(level1packet);
 writePacketLength(level1packet);
 sendPacket(level1packet);
 
 waitForMultiPacket(0x0001);
 } 
 while (!validateChecksum());
 packet_send_counter++;
 
 valuetype = level1packet[40+1+1]+level1packet[40+2+1]*256;
 
 if (valuetype==0x821e) {
 memcpy(invertername,&level1packet[48+1],14);
 Serial.println(invertername);
 memcpy(&datetime,&level1packet[40+4+1],4);  //Returns date/time unit switched PV off for today (or current time if its still on)
 }
 }
 
 void HistoricData() {
 
 time_t currenttime=now();
 digitalClockDisplay(currenttime);
 
 debugMsgln("Historic data...."));
 tmElements_t tm;
 if( year(currenttime) > 99)
 tm.Year = year(currenttime)- 1970;
 else
 tm.Year = year(currenttime)+ 30;  
 
 tm.Month = month(currenttime);
 tm.Day = day(currenttime);
 tm.Hour = 10;      //Start each day at 5am (might need to change this if you're lucky enough to live somewhere hot and sunny!!
 tm.Minute = 0;
 tm.Second = 0;
 time_t startTime=makeTime(tm);  //Midnight
 
 
 //Read historic data for today (SMA inverter saves 5 minute averaged data)
 //We read 30 minutes at a time to save RAM on Arduino
 
 //for (int hourloop=1;hourloop<24*2;hourloop++)
 while (startTime < now()) 
 {
 //HowMuchMemory();
 
 time_t endTime=startTime+(25*60);  //25 minutes on
 
 //digitalClockDisplay(startTime);
 //digitalClockDisplay(endTime);
 //debugMsgln(" ");
 
 do {
 writePacketHeader(level1packet);
 //writePacketHeader(level1packet,0x01,0x00,smaBTInverterAddressArray);
 writeSMANET2PlusPacket(level1packet,0x09, 0xE0, packet_send_counter, 0, 0, 0);
 
 writeSMANET2SingleByte(level1packet,0x80);
 writeSMANET2SingleByte(level1packet,0x00);
 writeSMANET2SingleByte(level1packet,0x02);
 writeSMANET2SingleByte(level1packet,0x00);
 writeSMANET2SingleByte(level1packet,0x70);
 // convert from an unsigned long int to a 4-byte array
 writeSMANET2Long(level1packet,startTime);
 writeSMANET2Long(level1packet,endTime);
 writeSMANET2PlusPacketTrailer(level1packet);
 writePacketLength(level1packet);
 sendPacket(level1packet);
 
 waitForMultiPacket(0x0001);
 }
 while (!validateChecksum());
 //debugMsg("packetlength=");    Serial.println(packetlength);
 
 packet_send_counter++;
 
 //Loop through values
 for(int x=40+1;x<(packetposition-3);x+=12){
 memcpy(&value,&level1packet[x+4],4);
 
 if (value > currentvalue) {
 memcpy(&datetime,&level1packet[x],4);
 digitalClockDisplay(datetime);
 debugMsg("=");
 Serial.println(value);
 currentvalue=value;         
 
 //uploadValueToSolarStats(currentvalue,datetime);          
 }
 }
 
 startTime=endTime+(5*60);
 delay(750);  //Slow down the requests to the SMA inverter
 }
 }
 */
