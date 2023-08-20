




#define DEBUG true      // change to false to stop Serial output if you use a display such as LED or LCD

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <TM1637Display.h>//https://github.com/avishorp/TM1637/tree/master
#include <string.h>
#include <stdint.h>
#include <TimeLib.h>    // https://github.com/PaulStoffregen/Time
#include <Timezone.h>   // https://github.com/JChristensen/Timezone
#include <DS3232RTC.h>  // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>  // https://github.com/geneReeves/ArduinoStreaming
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define TRIGGER_PIN 0
#define CLK 21
#define DIO 20
#define RTC_INTERRUPT_PIN 9 // interrupt on GPIO 12
#define LATCHPIN 14
#define OEPIN 3
#define VSPI_MISO   MISO
#define VSPI_MOSI   MOSI
#define VSPI_SCLK   SCK
#define VSPI_SS     SS

#define UDP_LISTEN_PORT 8888
#define VSPI FSPI

#define BLOCKSY     4
uint8_t x = 0,y = 0;
uint16_t displayData[16];
uint8_t letterA[8] = {
  0b00010000,
  0b00010000,
  0b00101000,
  0b00101000,
  0b01000100,
  0b01111100,
  0b10000010,
  0b10000010
};
unsigned int  timeout   = 120; // seconds to run for
unsigned int  startTime = millis();
bool portalRunning      = false;
bool startAP            = true; // start AP and webserver if true, else start only webserver
char ntpip[40] = {'0','0','0','0','0','0','0','0','0','0',
                  '0','0','0','0','0','0','0','0','0','0',
                  '0','0','0','0','0','0','0','0','0','0',
                  '0','0','0','0','0','0','0','0','0','0'};
int datacounter = 0;
// ISR variables
volatile uint32_t sysClock, ntpAlarmCounter; // the actual clock reference & fraction in millis
volatile bool outputTimestampEnable = false;  // used to trigger output of the current time
volatile bool halfSec = false;

WiFiManagerParameter custom_ntpip("ntpipadd","ntp ip add",ntpip,40);
WiFiManager wm;
TM1637Display display(CLK, DIO);
DS3232RTC RTC;
SPIClass * vspi = NULL;
WiFiUDP ntpClient;

/* TIMEZONE */
/*
   Pre-defined timezone definition examples. Use only one of these at a time or risk running
   out of memory on a small AVR e.g. ATmega328. Replace the example UK timezone definition
   with one of these. See the Timezone library documentation for further details.

  // US Eastern Time Zone (New York, Detroit)
  TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  // Eastern Daylight Time = UTC - 4 hours
  TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   // Eastern Standard Time = UTC - 5 hours
  Timezone usET(usEDT, usEST);

  // US Central Time Zone (Chicago, Houston)
  TimeChangeRule usCDT = {"CDT", Second, dowSunday, Mar, 2, -300};
  TimeChangeRule usCST = {"CST", First, dowSunday, Nov, 2, -360};
  Timezone usCT(usCDT, usCST);

  // US Mountain Time Zone (Denver, Salt Lake City)
  TimeChangeRule usMDT = {"MDT", Second, dowSunday, Mar, 2, -360};
  TimeChangeRule usMST = {"MST", First, dowSunday, Nov, 2, -420};
  Timezone usMT(usMDT, usMST);

  // Arizona is US Mountain Time Zone but does not use DST
  Timezone usAZ(usMST, usMST);

  // US Pacific Time Zone (Las Vegas, Los Angeles)
  TimeChangeRule usPDT = {"PDT", Second, dowSunday, Mar, 2, -420};
  TimeChangeRule usPST = {"PST", First, dowSunday, Nov, 2, -480};
  Timezone usPT(usPDT, usPST);
*/
//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);

/*
//United Kingdom (London, Belfast)
TimeChangeRule BST = {"BST", Last, Sun, Mar, 1, 60};        //British Summer Time
TimeChangeRule GMT = {"GMT", Last, Sun, Oct, 2, 0};         //Standard Time
Timezone UK(BST, GMT);
*/
TimeChangeRule *tcr;        //pointer to the time change rule, use to get the TZ abbrev

#define TIME_ZONE CE  // change to CE for European local time etc.

/*********************/
/* FUNCTIONS AND ISR */
/*********************/


//volatile uint32_t sysFraction;  // for future development

void ICACHE_RAM_ATTR rtcIntISR(void);
// the RTC 1PPS signal calls this ISR
void ICACHE_RAM_ATTR rtcIntISR(void) {
  halfSec = digitalRead(RTC_INTERRUPT_PIN);
  outputTimestampEnable = true; // signals OK to print out the 
  if(!halfSec) {
    // This increments the sysClock and sets the outputTimestampEnable flag
    sysClock++; // the system timestamp is incremented every second by the RTC
    //sysFraction = millis(); // for future development
    ntpAlarmCounter++; // used to trigger NTP time fetches
  }
}// END OF rtcIntISR

// the NTP client calls this function to read the NTP packetBuffer time values
uint32_t readNtpBuffer(uint8_t * buff, uint8_t _start) {
  // NTP data is sent as Big Endian so we need to convert it for Arduino use.
  // Reads the 4 Bytes from the NTP buffer and returns temp as Little Endian
  uint32_t temp;
  uint8_t* tmp = reinterpret_cast<uint8_t*>(&temp);
  for (uint8_t x = 0; x < 4; x++) tmp[x ^ 3] = buff[_start + x];
  return temp;
}// END OF readNtpBuffer

/**************/
/* NTP CLIENT */
/*************/

// an array of NTP servers. The specific server is specified by the index number
// in the call to sendNTPrequest(uint8_t index)
const char *ntpServer[3] = {"time.windows.com", "0.uk.pool.ntp.org", "0.jp.pool.ntp.org"}; // the NTP servers for testing

#define NTP_TIMEOUT 500       // maximum time to wait for an NTP server response in milliseconds
#define NTP_ROUNDTRIP_MAX 200  // the maximinun NTP round-trip in milliseconds
#define NTP_FETCH_PERIOD 60   // how often to fetch NTP time in seconds
#define NTP_FAIL_COUNT 5     // the number of times to retry an NTP fetch before giving up
#define NTP_FAIL_RETRY  10     // time in seconds to wait before trying an NTP re-fetch
#define NTP_PACKET_SIZE 48    // the size of the NTP packet
#define NTP_70_YEARS 2208988800UL // seconds between 1900 (NTP) & 1970 (UNIX)
#define SVR_TIME_SECS 40      // buffer offset for NTP timestamp
#define SVR_TIME_FRAC 44      // buffer offset for NTP fraction
uint32_t ntpTime, ntpFraction, ntpRoundtrip;
uint8_t packetBuffer[NTP_PACKET_SIZE];

// send an NTP request to the time server at the given address (ntpServer[index])
uint32_t sendNTPrequest(uint8_t index)
{
#if DEBUG
  Serial << F("ntp_req") << endl;
#endif
  // we can send an empty packet with just the first byte header to the NTP server
  memset(&packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  // send the NTP packet to the server
  ntpClient.flush(); // discard any previously received packet data
  ntpFraction = 0;
  // send the packet and exit if error
  if (!ntpClient.beginPacket(ntpServer[index], 123)) return 1;
  ntpClient.write(packetBuffer, NTP_PACKET_SIZE);
  if (!ntpClient.endPacket()) return 2;
  // now wait for the response from the server
  ntpRoundtrip = millis();
  bool _timeOut = true;
  uint8_t packetSize;
  uint32_t timeOutTimer = millis();
  while ((millis() - timeOutTimer) < NTP_TIMEOUT && _timeOut)
  {
    delay(0); // for ESP8266 compatibility
    if (packetSize = ntpClient.parsePacket()) _timeOut = false;
  }
  ntpRoundtrip = millis() - ntpRoundtrip;
  if (_timeOut) return 3;
  if (packetSize != NTP_PACKET_SIZE) return 4;
  // we got a valid response packet so carry on and
  // extract the timestamp and fractional seconds. Convert NTP timestamp to UNIX
  while (ntpClient.available()) ntpClient.read(packetBuffer, 48);
  ntpTime = readNtpBuffer(packetBuffer, SVR_TIME_SECS) - NTP_70_YEARS;
  // the timestamp fractional seconds are picoseconds so convert to milliseconds
  // ntpFraction * 10 ^ 6 / 2 ^ 32 /1000 = milliseconds
  ntpFraction = (uint32_t)((readNtpBuffer(packetBuffer, SVR_TIME_FRAC) * pow(10, 6)) / pow(2 , 32)) / 1000UL;
  return 0;
}// END OF sendNTPrequest
void drawPixel(uint8_t x, uint8_t y, uint8_t color) 
{
  uint16_t xBlock;
  uint8_t yHighOrLow;
  uint16_t mask;
  uint16_t xPos;
  yHighOrLow = y % 2;  //if even then result is 0, 0 means the lower
  xBlock = x / 8;
  xPos = x % 8;
  int16_t yTemp;
  if (color)  //draw
  {
    if (yHighOrLow == 0) 
    {
      displayData[xBlock + (y / 2) * BLOCKSY] |= 1 << xPos;
    } else 
    {
      displayData[xBlock + (y / 2) * BLOCKSY] |= 1 << (xPos + 8);
    }
  } else  //clear
  {
    if (yHighOrLow == 0) 
    {
      displayData[xBlock + (y / 2) * BLOCKSY] &= ~(1 << xPos);
    } else 
    {
      displayData[xBlock + (y / 2) * BLOCKSY] &= ~(1 << (xPos + 8));
    }
  }
}
void sendDisplay(void)
{
  digitalWrite(OEPIN, 1);
  vspi->beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));

  for(int i = 0;i<16;i++)
  {
    vspi->transfer16(displayData[i]);
  }
  vspi->endTransaction();
  digitalWrite(LATCHPIN, 1);
  digitalWrite(LATCHPIN, 0);
  digitalWrite(OEPIN, 0);
}

/**********************************************************************************************************************************************************/
/* SETUP */
/*********/
void setup() {

  display.setBrightness(7,true);
  WiFi.disconnect();
  vspi = new SPIClass(VSPI);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); //SCLK, MISO, MOSI, SS
  pinMode(LATCHPIN, OUTPUT);
  pinMode(OEPIN, OUTPUT);
  memset(displayData,0,32);
#if DEBUG
  Serial.begin(57600);
  Serial << F("A.N.T. Accurate Ntp Time (C) Phil Morris 2018 <www.lydiard.plus.com>") << endl;
#endif
  
  RTC.begin();
  RTC.squareWave(DS3232RTC::SQWAVE_1_HZ);   // 1 Hz square wave

  sysClock = RTC.get(); // synchronise the sysClock with the RTC for initial time setting
  ntpAlarmCounter = NTP_FETCH_PERIOD - 2;  // trigger an NTP fetch on first-run
 
  wm.setHostname("NARVAL_CLOCK1");

  WiFi.mode(WIFI_STA);
  WiFi.begin("nemeth_wifi","75000000");
  delay(5000);
  Serial << (F("IP address is ")) << WiFi.localIP() << endl;

  ntpClient.begin(UDP_LISTEN_PORT);

  // set up the RTC interrupt. MUST BE A VALID EXTERNAL INTERRUPT!
  // the Arduino ESP8266 IDE can support digitalPinToInterrupt()
  // but, to be safe...

  attachInterrupt(RTC_INTERRUPT_PIN, rtcIntISR, CHANGE);

  // the RTC Interrupt output is open collector so set the pullup resistor on the interrupt pin
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);
}// END OF setup

/********/
/* LOOP */
/********/

#define MILLIS_MIN 2000UL // 2 seconds
#define MILLIS_MAX 4294967295UL - MILLIS_MIN  // value - 2000 (2 seconds)

const char* weekdays[8] = {"Err", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char* months[13] = {"Err", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

bool ntpFetchFlag = false, ntpSetFlag = false, timeSynced = false, ntpFailFlag = false;
uint32_t rtcSetDelay = 0;
uint8_t ntpFailCounter = 0;
bool connected = false;
void loop() {
  
  if (WiFi.status() == WL_CONNECTED && !connected) {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    wm.setConfigPortalBlocking(false);
    wm.startWebPortal();
    connected=true;
  }  
  wm.process(); // do processing

  // is configuration portal requested?
  if(digitalRead(TRIGGER_PIN) == LOW && (!portalRunning)) {
    if(startAP){
      Serial.println("Button Pressed, Starting Config Portal");

      wm.addParameter(&custom_ntpip);
      wm.setConfigPortalBlocking(false);
      wm.startConfigPortal("Frissházasok");
    } else {
      Serial.println("Button Pressed, Starting Web Portal");
      wm.startWebPortal();
    }  
    startTime = millis();
  }

  // ntpAlarmCounter increments each second and sets the ntpFetchFlag
  // when the required time period has elapsed.
  if (!ntpFetchFlag && ntpAlarmCounter == NTP_FETCH_PERIOD) ntpFetchFlag = true;  // fetch the NTP time

  //fetch the NTP time if millis haven't rolled over.
  // The valid window is millis > 2000 and millis < 4294967295 - 2000 and we also check that the NTP
  // round-trip time is acceptable. A basic retry counter attempts to fetch the NTP time a number of times
  // before giving up until the next scheduled NTP fetch time. The LED's first digit decimal point is lit
  // if the NTP fetch fails completely. On an LCD display, the first digit becomes '*'
  if (ntpFetchFlag && millis() > MILLIS_MIN && millis() < MILLIS_MAX) {
    // fetch the NTP timestamp from the indexed NTP server, returns 0 if successful
    // The index allows you to choose an NTP server from the server names array easily
    uint8_t result = sendNTPrequest(0);
    if (!result && ntpRoundtrip <= NTP_ROUNDTRIP_MAX) {
      // calculate the delay before updating the RTC and sysClock
      rtcSetDelay = (uint32_t)(millis() + (1000UL - ntpFraction)) + (ntpRoundtrip >> 1);
      ntpSetFlag = true;    // sync can take place
      ntpFailFlag = false;  // clear the ntpFailFlag
      ntpAlarmCounter = 0;  // clear the ntpAlarmCounter
      ntpTime++; // add 1 second as we're setting the sysClock 1 second late
    }
    else {
      // the NTP request returned an error status
#if DEBUG
      switch (result) {
        case 1:
          Serial << F("NTP beginPacket Failure!") << endl;
          break;
        case 2:
          Serial << F("NTP Client endPacket Failure!") << endl;
          break;
        case 3:
          Serial << F("NTP Timeout Error!") << endl;
          break;
        case 4:
          Serial << F("NTP Packet Size Error!") << endl;
          break;
        default:
          Serial << F("NTP Round-tript too long, aborting!") << endl;
      }
#endif
      // the NTP fetch has failed, so start counting the failures
      if (ntpFailCounter++ == NTP_FAIL_COUNT) {
        // total NTP fetch failure
        ntpAlarmCounter = 0;  // reset the ntpAlarmCounter
        ntpFailCounter = 0;   // reset the ntpFailCounter
        ntpFailFlag = true;   // set the ntpFailFlag
#if DEBUG
        Serial << F("Total NTP Fetch Failure!") << endl;
#endif
      }
      else ntpAlarmCounter -= NTP_FAIL_RETRY; // try an NTP fetch again in NTP_FAIL_RETRY seconds
    }
    ntpFetchFlag = false; // clear the fetch Alarm flag
  }

  // sync everything when rtcSetDelay has expired
  if (ntpSetFlag && millis() >= rtcSetDelay) {
    RTC.set(ntpTime); // set the RTC
    sysClock = ntpTime; // set sysClock
    ntpSetFlag = false; // clear the sync flag
    timeSynced = true;  // time DEBUG printout includes NTP timing details
    rtcSetDelay = 0;
  }

  // print out the sysClock timestamp
  if (outputTimestampEnable) {
    outputTimestampEnable = false;  // clear the trigger flag
    char buff[32];
    tmElements_t tm;
    // make the current local time using the Timezone library
    breakTime(TIME_ZONE.toLocal(sysClock, &tcr), tm);
    sprintf(buff, "%02u:%02u:%02u %s  %02u-%s-20%02u", tm.Hour, tm.Minute, tm.Second, weekdays[tm.Wday], tm.Day, months[tm.Month], tm.Year - 30);
    int num;
    num = tm.Minute*100;
    num+= tm.Second;
    if(halfSec)
    {
        memset(displayData,0x00,32);
        x++;
        if(x >31)
        {
          x = 0;
          y++;
          if(y >7)
          {
            y = 0;
          }
        }
        drawPixel(x, y, 1);
          
      sendDisplay();
      display.showNumberDecEx(num,0b01000000,false,4,0);
    }
    else
    {
     memset(displayData,0x00,32);
      sendDisplay();
      display.showNumberDecEx(num,0,false,4,0);
    }
    

#if DEBUG
    Serial.println(buff);
#endif
    
    timeSynced = false;
  }
}// END OF loop
