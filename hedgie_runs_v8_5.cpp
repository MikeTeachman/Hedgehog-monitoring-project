/* Hedgie runs version 8.5
- counts wheel turns
- finds duration of shortest wheel revolution, for each 5 minute interval
- calculates total distance, in km
- added LCD display
- displays total distance
- LCD backlight with push button
- add RTC
- store datetime of first wheel turn, and last wheel turn
- Tweet the nightly run stats at 7am
- added Ethernet card support
- added start/stop time to the tweets  
- reload prev night stats from EEPROM when button pressed
- send 5min interval distance to Sparkfun Data service
- new startup state to allow test rotations before official hedgie tracking begins
- added NTP support to update the real-time clock each morning
- snap interval recording to the unix 5-minute boundary
- DNS lookup for NTP server
- added watchdog timer (to restart when code hangs)
- change NTP support to manual, initiated by button press
- night stats are recovered when unexpected reset occurs during night
- time of unexpected reset is saved to EEPROM
- longer watchdog implementation
v8.4
- moved 7am Tweet before Sparkfun data push (attempt to fix problem of missed 7am tweet, due to suspected reset with Sparkfun push)
- added new feed:  Adafruit IO.  Logging accumulated running distance, temperature, and uptime, every 5 mins
v8.5
- discard any RTC reads where HOURS are bogus.  This is a software fix to a problem that cropped up when I2C temperature sensor was added in v8.4.  
Occasionally, the RTC read came back with HH:MM = 153:165.  Suspect I2C signal degradation due to long connect wire to I2C sensor, with non-optimal pull-up resistor.

EEPROM map
==========
800: night stats
100: debug log
200: reset log

*/

#include <stdio.h>
#include <Wire.h>  
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Dns.h>
#include <avr/wdt.h>
#include "Adafruit_IO_Client.h"
#include "Adafruit_MCP9808.h"   // temperature sensor

typedef enum
{
  STARTUP_TESTING_DISCARD_HEDGIE_STATISTICS,  // used to allow some startup wheel spins during the "office hours", but not trigger the start time to be recorded
  CAPTURE_HEDGIE_STATISTICS
} STATISTICS_CAPTURE_STATE_t;

typedef enum
{
  DETERMINE_MIRROR_LOCATION,
  WAITING_FOR_MIRROR,
  WAITING_FOR_WHITE
} WHEEL_STATE_t;

typedef enum
{
  SHORT_TIME_FORMAT,
  LONG_TIME_FORMAT,
} TIME_FORMAT_t;

typedef struct
{
  unsigned long totalDistanceInCm;
  DateTime dateTimeOfFirstRotationInDateTime;
  DateTime dateTimeOfLastRotationInDateTime;
  
} HedgieNightStats_t;

enum
{
  DEBUG_START_MARKER = 0, 
  ETHERNET_DHCP_OK, //1
  ETHERNET_DHCP_FAILED, //2
  ETHERNET_CONNECT_TO_THINGSPEAK_OK_1, //3
  ETHERNET_CONNECT_TO_THINGSPEAK_FAILED_1, //4
  ETHERNET_CONNECT_TO_THINGSPEAK_OK_2, //5
  ETHERNET_CONNECT_TO_THINGSPEAK_FAILED_2, //6
  ETHERNET_CONNECT_TO_THINGSPEAK_3, //7
  ETHERNET_CONNECT_TO_THINGSPEAK_4, //8
  ETHERNET_CONNECT_TO_THINGSPEAK_5, //9
  ETHERNET_CONNECT_TO_THINGSPEAK_6, //10
  ETHERNET_NTP_1, //11
  ETHERNET_NTP_2, //12
  ETHERNET_NTP_3, //13
  ETHERNET_NTP_4, //14
  ETHERNET_NTP_5, //15
  ETHERNET_NTP_6, //16
  ETHERNET_NTP_7, //17
  ETHERNET_NTP_8, //18
  ETHERNET_NTP_9, //19
  ETHERNET_NTP_10, //20
  ETHERNET_NTP_11, //21
  ETHERNET_NTP_12, //22
  NUMBER_OF_DEBUG_MESSAGES             // <-- keep this last
  
} DEBUG_MESSAGES;

#define CAPTOUCH_BUTTON (6)
#define PROTOSHIELD_BUTTON (7)
#define WHEEL_ROTATION_LED (8)
#define GREEN_LED (9)
#define WHEEL_CIRCUMFERENCE_IN_CM (85)
#define DELAY_BETWEEN_SAMPLES (2)
#define STARTUP_COUNT_THRESHOLD (10)
#define NUM_INTERVALS_TO_RESET 2
#define WLAN_SSID       "... your WiFi SSID..."
#define WLAN_PASS       "... your WiFi password..."
#define AIO_KEY  "==AIO Key =="

WHEEL_STATE_t detectMirror(DateTime& dateNow);
WHEEL_STATE_t detectWhite(void);
void initCountLog(void);
void initNightStats(void);
void saveNightStatsToEEPROM(void);
void loadNightStatsFromEEPROM(void);
boolean isMirror(int reference);
void initDebugMsgLog(void);
void logDebugMsg(uint8_t debugIndex, uint8_t debugMsg);
void initResetLog(void);
void saveTimeOfLastResetToEEPROM(DateTime timeOfLastReset);
DateTime readTimeOfLastResetFromEEPROM(void);
void displayTimeOfLastReset(void);
boolean isButtonPress(void);
boolean isProtoshieldButtonPress(void);
boolean isNewHour(DateTime&);
boolean isNewMinute(DateTime&);
void delaySecsWithWatchdog(uint16_t numSecDelay);
void handleButtonPress(DateTime& dateNow);
void convertCmsToKm(uint32_t cms, uint32_t *km, uint32_t *kmFraction);
uint32_t convertCmsToM(uint32_t cms);
void getTimeAsString(DateTime& dateTime, char *timeBuf_p, TIME_FORMAT_t format);
void constructTwitterMsg(char *twitterMsg);
void tweetNightStats(void);
void updateTwitterStatus(char *twitterMsg);
void sendDataToSparkFun(DateTime& dateNow, uint32_t distanceRunIntervalInCm);
void setupEthernet();
void updateRtcUsingNTP(void);
unsigned long getNTP();
void sendNTPpacket(IPAddress& address, byte *packetBuffer);
int dstOffset (DateTime time);
void displayTime(DateTime& dateNow);
boolean isValidHour(DateTime& dateNow);
int freeRam();

HedgieNightStats_t nightStats;
DateTime resetLog;
uint32_t distanceRunIntervalInCm=0;
const int EEPROMaddrForDebugLog=100;
const int EEPROMaddrForResetLog=200;
const int EEPROMaddrForNightStats=800;
int whiteSampleCount = 0;
uint16_t startupTestingCount=0;
uint8_t wdtCount = NUM_INTERVALS_TO_RESET; // number of intervals before unit will force a reset (total time is intervals x 8 seconds)
uint32_t uptimeInMinutes = 0;

STATISTICS_CAPTURE_STATE_t statisticsCaptureState = STARTUP_TESTING_DISCARD_HEDGIE_STATISTICS;
WHEEL_STATE_t wheelState = DETERMINE_MIRROR_LOCATION;
byte prevHour;
byte prevMinute;
RTC_DS1307 rtc;

// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

const char* monthStr[]={"January","February","March","April","May","June","July","August","September","October","November","December"};
const char* dayOfWeekStr[]={"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // numeric IP for Google (no DNS)

// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192,168,0,177);

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 80 is default for HTTP):
EthernetClient client;

// Create an Adafruit IO Client instance.  Notice that this needs to take a
// WiFiClient object as the first parameter, and as the second parameter a
// default Adafruit IO key to use when accessing feeds (however each feed can
// override this default key value if required, see further below).
Adafruit_IO_Client aio = Adafruit_IO_Client(client, AIO_KEY);

// Finally create instances of Adafruit_IO_Feed objects, one per feed.  Do this
// by calling the getFeed function on the Adafruit_IO_Feed object and passing
// it at least the name of the feed, and optionally a specific AIO key to use
// when accessing the feed (the default is to use the key set on the
// Adafruit_IO_Client class).
Adafruit_IO_Feed hedgieDistance = aio.getFeed("hhd");
Adafruit_IO_Feed hedgieTemperature = aio.getFeed("hht");
Adafruit_IO_Feed hedgieUptime = aio.getFeed("hhu");

// ThingSpeak connection information
#define WEBSITE      "api.thingspeak.com"
char thingtweetAPIKey[] = "=======";  // ThingSpeak settings
char thingspeeakServer[] = WEBSITE;    // name address for Google (using DNS)

// Sparkfun Data connection information
char sparkfunServer[] = "data.sparkfun.com";    // name address for data.sparkFun (using DNS)
const String publicKey = "============";
const String privateKey = "==============";

// NTP 
unsigned int localPort = 8888; //Local port to listen for UDP Packets
// IPAddress timeServer(132, 163, 4, 101); //NTP Server IP 
IPAddress timeServer; // pool.ntp.org NTP server
const int NTP_PACKET_SIZE= 48;  //NTP Time stamp is in the firth 48 bytes of the message
EthernetUDP Udp;  //UDP Instance to let us send and recieve packets
unsigned long epoch; //Unix Epoch time (NTP or RTC depending on state)
const int TZ_OFFSET = (8*3600);  //PST UTC-8

void setup()
{
  DateTime dateNow;
  
  // Watchdog timer setup.  
  noInterrupts();
  MCUSR  &= ~_BV(WDRF);
  WDTCSR  =  _BV(WDCE) | _BV(WDE);              // WDT change enable
  WDTCSR  =  _BV(WDIE) | _BV(WDP3) | _BV(WDP0); // Interrupt enable, 8 sec.
  interrupts();  

  Wire.begin();
  rtc.begin();
  tempsensor.begin();
  
  // Serial.begin(9600); 
  
  // initialize digital pins as outputs
  pinMode(WHEEL_ROTATION_LED, OUTPUT);     
  pinMode(GREEN_LED, OUTPUT);    
 
  // initialize captouch digital pin as an input, using internal pullup resistor.
  pinMode(CAPTOUCH_BUTTON, INPUT_PULLUP);
  pinMode(PROTOSHIELD_BUTTON, INPUT_PULLUP);

  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.clear();
  lcd.backlight();
  
  //rtc.adjust(DateTime(__DATE__, __TIME__));
  
  dateNow = rtc.now();
  displayTime(dateNow);
  
  wheelState = DETERMINE_MIRROR_LOCATION;
  distanceRunIntervalInCm = 0;
  
  if (dateNow.hour() >= 7 && dateNow.hour() < 22) 
  {
    // normal startup during hedgie sleeping hours (daytime)
    startupTestingCount = 0; 
    statisticsCaptureState = STARTUP_TESTING_DISCARD_HEDGIE_STATISTICS;
    initNightStats();      
  }
  else
  {
    // unexpected reset during hedgie office hours (nighttime)
    // recover the nightStats from EEPROM - this allows continuation of nightStats used for tweeting
    startupTestingCount = STARTUP_COUNT_THRESHOLD; 
    statisticsCaptureState = CAPTURE_HEDGIE_STATISTICS;
    loadNightStatsFromEEPROM(); 
    saveTimeOfLastResetToEEPROM(dateNow);  
   }
  
  prevHour = dateNow.hour();
  prevMinute = dateNow.minute();
  
  digitalWrite(WHEEL_ROTATION_LED, HIGH);
  setupEthernet();
  digitalWrite(WHEEL_ROTATION_LED, LOW);
  
  // Initialize the Adafruit IO client class 
  aio.begin();
  
  uptimeInMinutes = 0;
} 


void loop()
{
  DateTime dateNow;
  boolean newHour;
  boolean newMinute;

  dateNow = rtc.now();
  
  if (isValidHour(dateNow) == true)
  {
    newHour = isNewHour(dateNow);
    newMinute = isNewMinute(dateNow);
  }
  
  switch (wheelState)
  {
    case DETERMINE_MIRROR_LOCATION:
      if (isMirror())
      {
        wheelState = WAITING_FOR_WHITE;
      }
      else
      {
        wheelState = WAITING_FOR_MIRROR;
      }
      
      break;
  
    case WAITING_FOR_MIRROR:
      wheelState = detectMirror(dateNow);
      break;
      
    case WAITING_FOR_WHITE:
    default:
      wheelState = detectWhite();
      break;
  }
  
  // at 10pm, do a one-time prep for Hedgie's upcoming night in the office
  if (newHour && (dateNow.hour() == 22))
  {
    digitalWrite(GREEN_LED, HIGH); 
    wheelState = DETERMINE_MIRROR_LOCATION;
    initNightStats();
    saveNightStatsToEEPROM();
    initDebugMsgLog();
    initResetLog();
    distanceRunIntervalInCm=0; 
    statisticsCaptureState = STARTUP_TESTING_DISCARD_HEDGIE_STATISTICS;
    startupTestingCount=0; 
    digitalWrite(GREEN_LED, LOW); 
  }
  
  // at 7am send tweet
  if (newHour && (dateNow.hour() == 7))
  {
    digitalWrite(GREEN_LED, HIGH); 
    tweetNightStats();  // TWEET it !
    digitalWrite(GREEN_LED, LOW); 
  }
  
  // every 5 minutes push temperature and uptime data to Adafruit IO
  if (newMinute && ((dateNow.minute() % 5) == 0))
  {
    hedgieTemperature.send(tempsensor.readTempC()); 
    hedgieUptime.send(uptimeInMinutes);
    uptimeInMinutes+=5;
  }
  
  // every 5 mins between 10pm and 7am, save interval stats, and send to SparkFun cloud service
  // also send accumulated distance to Adafruit IO
  // some tricky logic here:
  // - only push once at exactly 7am
  // - only push data when the minute changes (otherwise it would keep pushing data repeatedly during every 5th minute)
  if (((dateNow.hour() >= 22) || (dateNow.hour() < 7) || (newHour && (dateNow.hour() == 7))) &&
      (newMinute && ((dateNow.minute() % 5) == 0)))
  {
    digitalWrite(GREEN_LED, HIGH);
    
    // push accumulated distance to Adafruit IO 
    hedgieDistance.send(convertCmsToM(nightStats.totalDistanceInCm)); 
      
    // Send interval stats to the Sparkfun Data service
    sendDataToSparkFun(dateNow, distanceRunIntervalInCm);
    saveNightStatsToEEPROM();
    distanceRunIntervalInCm=0;   
    digitalWrite(GREEN_LED, LOW); 
  }
  else
  {
    // during the day push 0 to Adafruit IO every 5 minutes
    if (newMinute && ((dateNow.minute() % 5) == 0))
    {
      hedgieDistance.send(0);
    }
  }
      
  handleButtonPress(dateNow);
  
  // update RTC chip using NTP, when protoshield button is pressed
  // this is done manually, because NTP sometimes returns incorrect time
  if (isProtoshieldButtonPress() == true)
  {
    updateRtcUsingNTP();  // update real-time clock 
  }
    
  delay(DELAY_BETWEEN_SAMPLES);
  
  wdtCount = NUM_INTERVALS_TO_RESET;  // restore watchdog count
} 

WHEEL_STATE_t detectMirror(DateTime& dateNow)
{
  // check for mirror
  if (isMirror())
  {
    // mirror detected !
    
    if (dateNow.hour() >= 22 || dateNow.hour() < 7)
    {
       if (statisticsCaptureState == STARTUP_TESTING_DISCARD_HEDGIE_STATISTICS)
       {
         startupTestingCount++;
         
         if (startupTestingCount >= STARTUP_COUNT_THRESHOLD)
         {
           statisticsCaptureState = CAPTURE_HEDGIE_STATISTICS;
         }
       }
       else
       {
          if (nightStats.totalDistanceInCm == 0)
          {
            nightStats.dateTimeOfFirstRotationInDateTime = rtc.now();
          }
          else
          {
            nightStats.dateTimeOfLastRotationInDateTime = rtc.now();
          }
          
          // only accumulate rotation and distance data during hedgie office hours, and when startup test rotations have been completed
          distanceRunIntervalInCm += (unsigned long) WHEEL_CIRCUMFERENCE_IN_CM;
          nightStats.totalDistanceInCm += (unsigned long) WHEEL_CIRCUMFERENCE_IN_CM;
       }
    }
    
    digitalWrite(WHEEL_ROTATION_LED, HIGH);
    wheelState = WAITING_FOR_WHITE;
  }
  else
  {
    // mirror not detected
    wheelState = WAITING_FOR_MIRROR;
  }
  
  return wheelState;
}


WHEEL_STATE_t detectWhite(void)
{
  if (!isMirror())
  {
    // detected white
    whiteSampleCount++;    
  }
  else
  {
    // detected mirror
    whiteSampleCount=0;
  }
  
  if (whiteSampleCount > 20)
  {
    //  white has been detected
    digitalWrite(WHEEL_ROTATION_LED, LOW); 
    whiteSampleCount=0;
    wheelState = WAITING_FOR_MIRROR;
  }
  else
  {
    // still waiting to get enough consecutive white samples
    wheelState = WAITING_FOR_WHITE;
  }

  return wheelState;
}

boolean isMirror(void)
{
  if (analogRead(0) > 300)
  {
    // mirror detected
    return true;
  }
  else
  {
    // white detected
    return false;
  }
}

void initDebugMsgLog(void)
{
  int i;
  
  for (i=0; i<NUMBER_OF_DEBUG_MESSAGES; i++)
  {
    EEPROM.write(EEPROMaddrForDebugLog+i, 0);
  }
  
  EEPROM.write(EEPROMaddrForDebugLog+DEBUG_START_MARKER, 55);
  EEPROM.write(EEPROMaddrForDebugLog+NUMBER_OF_DEBUG_MESSAGES, 55);
}

void logDebugMsg(uint8_t debugIndex, uint8_t debugMsg)
{
  EEPROM.write(EEPROMaddrForDebugLog+debugIndex, debugMsg);
}

void initResetLog(void)
{
  int i;
  
  for (i=0; i<sizeof(DateTime); i++)
  {
    EEPROM.write(EEPROMaddrForResetLog+i, 0);
  }
}

void saveTimeOfLastResetToEEPROM(DateTime timeOfLastReset)
{
  int i;
  
  for (i=0; i<sizeof(DateTime); i++)
  {
    EEPROM.write(EEPROMaddrForResetLog+i, ((unsigned char *)(&timeOfLastReset))[i]);
  }
}

DateTime readTimeOfLastResetFromEEPROM(void)
{
  int i;
  DateTime timeOfLastReset;

  for (i=0; i<sizeof(DateTime); i++)
  {
    ((unsigned char *)(&timeOfLastReset))[i] = EEPROM.read(EEPROMaddrForResetLog+i);
  }
  
  return timeOfLastReset;
}

void displayTimeOfLastReset(void)
{
  char timeStr[10];
  DateTime timeOfLastReset;
  
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print(F("unexpected reset"));
  lcd.setCursor(0,1); //Start at character 0 on line 1
  timeOfLastReset = readTimeOfLastResetFromEEPROM();
  getTimeAsString(timeOfLastReset, timeStr, LONG_TIME_FORMAT);
  lcd.print(timeStr);
  delaySecsWithWatchdog(2);
  lcd.noBacklight();
}

boolean isButtonPress(void)
{
  if (digitalRead(CAPTOUCH_BUTTON))
  {
    return true;
  }
  else
  {
    return false;
  }
}  

boolean isProtoshieldButtonPress(void)
{
  if (digitalRead(PROTOSHIELD_BUTTON) == LOW)
  {
    return true;
  }
  else
  {
    return false;
  }
}  

boolean isNewHour(DateTime& dateNow)
{
  if (prevHour != dateNow.hour())
  {
    prevHour = dateNow.hour();
    return true;
  }
  else
  {
    return false;
  }
}

boolean isNewMinute(DateTime& dateNow)
{
  if (prevMinute != dateNow.minute())
  {
    prevMinute = dateNow.minute();
    return true;
  }
  else
  {
    return false;
  }
}

void delaySecsWithWatchdog(uint16_t numSecDelay)
{
  int i;
  
  for (i=0; i<numSecDelay; i++)
  {
    delay (1000);
    wdtCount = NUM_INTERVALS_TO_RESET;  // restore watchdog count
  }
}


void initNightStats(void)
{
  nightStats.totalDistanceInCm = 0;
  nightStats.dateTimeOfFirstRotationInDateTime = rtc.now();
  nightStats.dateTimeOfLastRotationInDateTime = rtc.now();  
};

void saveNightStatsToEEPROM(void)
{
  int i;
  
  for (i=0; i<sizeof(HedgieNightStats_t); i++)
  {
    EEPROM.write(EEPROMaddrForNightStats+i, ((unsigned char *)(&nightStats))[i]);
  }
}

void loadNightStatsFromEEPROM(void)
{
  int i;
  
  for (i=0; i<sizeof(HedgieNightStats_t); i++)
  {
    ((unsigned char *)(&nightStats))[i] = EEPROM.read(EEPROMaddrForNightStats+i);
  }
}

void handleButtonPress(DateTime& dateNow)
{
  char timeStr[10];
  
  if (isButtonPress())
  {
    uint32_t km;
    uint32_t kmFraction;
    char timeStr[10];
    
    // check time to make sure we don't load overtop stats that have yet to be saved
    if (dateNow.hour() >= 7 && dateNow.hour() < 22)
    {
      loadNightStatsFromEEPROM();
    }
    
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print(F("km   start end")); 
    
    lcd.setCursor(0,1); 
    convertCmsToKm(nightStats.totalDistanceInCm, &km, &kmFraction);  
    lcd.print(km);  
    lcd.print(F("."));  
    lcd.print(kmFraction);  
    lcd.setCursor(5,1); 
    getTimeAsString(nightStats.dateTimeOfFirstRotationInDateTime, timeStr, SHORT_TIME_FORMAT);
    lcd.print(timeStr);
    lcd.setCursor(11,1); 
    getTimeAsString(nightStats.dateTimeOfLastRotationInDateTime, timeStr, SHORT_TIME_FORMAT);
    lcd.print(timeStr);
    delaySecsWithWatchdog(4);
    
    // display temperature
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print(F("temperature")); 
    lcd.setCursor(0,1);
    lcd.print(tempsensor.readTempC());
    delaySecsWithWatchdog(4);
    
    displayTime(dateNow);
    
    displayTimeOfLastReset();
  
    lcd.clear();
    lcd.noBacklight();
  }  
}

void flashLED(void)
{
#if 1
  digitalWrite(GREEN_LED, HIGH); 
  delay(300);
  digitalWrite(GREEN_LED, LOW); 
  delay(300);
#endif  
}

void convertCmsToKm(uint32_t cms, uint32_t *km, uint32_t *kmFraction)
{
  *km = cms/(1000L*100L);
  *kmFraction = (cms%(1000L*100L))/(100L*100L);
}

uint32_t convertCmsToM(uint32_t cms)
{
  return cms/100L;
}

// feed in DateTime...get back string with time, like "12:45 pm" or "3:45am"
void getTimeAsString(DateTime& dateTime, char *timeBuf_p, TIME_FORMAT_t format)
{
  char *ampm;
  uint8_t hour;
  uint8_t minute;
 
  hour = dateTime.hour();
  minute = dateTime.minute();
  
  if (hour == 12)
  {
    ampm = "PM";
  }
  else if(hour < 13)
  {
    ampm = "AM";
    
    // detect and correct when hour is between midnight and 1am
    if (hour == 0)
    {
      hour = 12;
    }
  }
  else
  {
    ampm = "PM";
    hour = hour - 12;
  }
  
  if (format == LONG_TIME_FORMAT)
  {
    sprintf(timeBuf_p, "%u:%.2u %s", hour, minute, ampm);
  }
  else
  {
    // don't include the AM/PM in the time string
    sprintf(timeBuf_p, "%u:%.2u", hour, minute);
  }  
}

void constructTwitterMsg(char *twitterMsg)
{
  char timeStartStr[15];
  char timeEndStr[15];
  uint32_t km;
  uint32_t kmFraction;
  DateTime timeNow;
  uint8_t dayOfWeek;
  uint8_t monthOfYear;
  uint8_t dayOfMonth;
  uint16_t year;

  
  timeNow = rtc.now();
  dayOfWeek = timeNow.dayOfWeek();
  dayOfMonth = timeNow.day();
  monthOfYear = timeNow.month();
  year = timeNow.year();
  
  // range checks to make sure string array lookups stay inbounds
  if (dayOfWeek > 6)
  {
    dayOfWeek = 0;
  }
  
  if (monthOfYear > 12)  // Note:  this is correct - in the DateTime class, the 1st Month is "1", but the first DayOfWeek is "0"
  {
    monthOfYear = 1;
  }
  
  convertCmsToKm(nightStats.totalDistanceInCm, &km, &kmFraction);  
  getTimeAsString(nightStats.dateTimeOfFirstRotationInDateTime, timeStartStr, LONG_TIME_FORMAT);
  getTimeAsString(nightStats.dateTimeOfLastRotationInDateTime, timeEndStr, LONG_TIME_FORMAT);
  logDebugMsg(ETHERNET_CONNECT_TO_THINGSPEAK_5, ETHERNET_CONNECT_TO_THINGSPEAK_5);
  
  // build twitter string
  sprintf(twitterMsg, "%s%s%sSir Charles update for %s %s %u %u:  Distance ran last night: %lu.%lu km,  Start: %s,  Finish: %s   #runhedgie", 
     "api_key=",
     thingtweetAPIKey,
     "&status=",
     dayOfWeekStr[dayOfWeek], 
     monthStr[monthOfYear-1],  // Jan = 1, but index needs to be zero for string lookup
     dayOfMonth, 
     year, 
     km, 
     kmFraction, 
     timeStartStr, 
     timeEndStr);
     
  logDebugMsg(ETHERNET_CONNECT_TO_THINGSPEAK_6, ETHERNET_CONNECT_TO_THINGSPEAK_6);

  //Serial.println(twitterMsg);
  //Serial.println(strlen(twitterMsg));
}

void blinkLed(int numberBlinks)
{
  int i;
  
  for (i=0; i<numberBlinks; i++)
  {
    flashLED();
  }
  
  delaySecsWithWatchdog(2);
}

void tweetNightStats(void)
{
  char twitterMsg[200];
   
  constructTwitterMsg(twitterMsg);
  updateTwitterStatus(twitterMsg);
}

void updateTwitterStatus(char *twitterMsg)
{
  //Serial.println(F("connecting..."));
  
  if (client.connect(thingspeeakServer, 80)) 
  {
    client.print("POST /apps/thingtweet/1/statuses/update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(strlen(twitterMsg));
    client.print("\n\n");
    client.print(twitterMsg);
    
    logDebugMsg(ETHERNET_CONNECT_TO_THINGSPEAK_OK_1, ETHERNET_CONNECT_TO_THINGSPEAK_OK_1);

    
    if (client.connected())
    {
      //Serial.println(F("Connected to ThingSpeak..."));
      //Serial.println();
      logDebugMsg(ETHERNET_CONNECT_TO_THINGSPEAK_OK_2, ETHERNET_CONNECT_TO_THINGSPEAK_OK_2);
      
    }
    else
    {
      //Serial.println(F("Connection to ThingSpeak failed"));   
      //Serial.println();
      logDebugMsg(ETHERNET_CONNECT_TO_THINGSPEAK_FAILED_2, ETHERNET_CONNECT_TO_THINGSPEAK_FAILED_2);
    }
  } 
  else 
  {
    //Serial.println(F("Connection to ThingSpeak failed"));   
    //Serial.println();
    logDebugMsg(ETHERNET_CONNECT_TO_THINGSPEAK_FAILED_1, ETHERNET_CONNECT_TO_THINGSPEAK_FAILED_1);
  }
  
  //Serial.println(F("...disconnecting"));
  //Serial.println();
  
  logDebugMsg(ETHERNET_CONNECT_TO_THINGSPEAK_3, ETHERNET_CONNECT_TO_THINGSPEAK_3);

  client.stop();
  
  logDebugMsg(ETHERNET_CONNECT_TO_THINGSPEAK_4, ETHERNET_CONNECT_TO_THINGSPEAK_4);
}

void sendDataToSparkFun(DateTime& dateNow, uint32_t distanceRunIntervalInCm)
{
  char distanceInCmAsString[10];
  char timeAsString[10];
  
  // Make a TCP connection to remote host
  if (client.connect(sparkfunServer, 80))
  {
    // Post the data! Request should look a little something like:
    // GET /input/publicKey?private_key=privateKey&light=1024&switch=0&name=Jim HTTP/1.1\n
    // Host: data.sparkfun.com\n
    // Connection: close\n
    // \n
    
    // format
    // http://data.sparkfun.com/input/[publicKey]?private_key=[privateKey]&distanceInCm=[value]&time=[value]
    
    client.print("GET /input/");
    client.print(publicKey);
    client.print("?private_key=");
    client.print(privateKey);
    
    client.print("&");
    client.print("distanceInCm");
    client.print("=");
    
    // convert distance to string
    sprintf(distanceInCmAsString, "%lu", distanceRunIntervalInCm);
    client.print(distanceInCmAsString);
    
    client.print("&");
    client.print("time");
    client.print("=");
    
    // human readable time
    getTimeAsString(dateNow, timeAsString, SHORT_TIME_FORMAT);
    client.print(timeAsString);
    
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(sparkfunServer);
    client.println("Connection: close");
    client.println();
  }
  else
  {
    // Serial.println(F("Connection failed"));
  } 
  
  // Check for a response from the server, and route it
  // out the serial port.
  while (client.connected())
  {
    if ( client.available() )
    {
      char c = client.read();
      // Serial.print(c);
    }      
  }
  
  // Serial.println();
  client.stop();
}

void setupEthernet()
{
  // Serial.println(F("Connecting Arduino to network..."));
  // Serial.println();  

  delaySecsWithWatchdog(1);
  
  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) 
  {
    // Serial.println(F("DHCP Failed, reset Arduino to try again"));
    // Serial.println();
    logDebugMsg(ETHERNET_DHCP_FAILED, ETHERNET_DHCP_FAILED);
  } 
  else
  {
    // Serial.println(F("Arduino connected to network using DHCP"));
    // Serial.println();
    logDebugMsg(ETHERNET_DHCP_OK, ETHERNET_DHCP_OK);
  } 
  
  // Serial.print(F("My IP address: "));
  // Serial.println(Ethernet.localIP());

  // give the Ethernet shield a second to initialize:
  delaySecsWithWatchdog(1);
}

void updateRtcUsingNTP(void)
{
  DateTime dateNow;
  
  digitalWrite(GREEN_LED, HIGH); 
  
  Udp.begin(localPort);
  //get the NTP timestamp
  epoch = getNTP();
  //set the RTC
  rtc.adjust(epoch);
  
  digitalWrite(GREEN_LED, LOW); 
  
  // show the time on the local LCD display
  dateNow = rtc.now();
  displayTime(dateNow);
}

unsigned long getNTP() 
{
  int packetSize;
  unsigned long epoch;
  byte packetBuffer[NTP_PACKET_SIZE];  //Buffer to hold incoming and outgoing packets
  DNSClient dns;

  dns.begin(Ethernet.dnsServerIP());
  logDebugMsg(ETHERNET_NTP_10, ETHERNET_NTP_10);
  
  if(dns.getHostByName("pool.ntp.org",timeServer)) 
  {
    //Serial.print(F("NTP server ip :"));
    //Serial.println(timeServer);
    logDebugMsg(ETHERNET_NTP_11, ETHERNET_NTP_11);
  }
  else 
  {
    //Serial.print(F("dns lookup failed"));
    logDebugMsg(ETHERNET_NTP_12, ETHERNET_NTP_12);
  }
  
  logDebugMsg(ETHERNET_NTP_1, ETHERNET_NTP_1);
  sendNTPpacket(timeServer, packetBuffer); // send an NTP packet to a time server
  logDebugMsg(ETHERNET_NTP_2, ETHERNET_NTP_2);

  delaySecsWithWatchdog(1);
  packetSize = Udp.parsePacket();
  logDebugMsg(ETHERNET_NTP_3, ETHERNET_NTP_3);
  
  if(packetSize)
  {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer
    logDebugMsg(ETHERNET_NTP_4, ETHERNET_NTP_4);
    
    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);  
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;  

    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;     
    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;  
    epoch = epoch - TZ_OFFSET;
    epoch = epoch + dstOffset(epoch);  //Adjust for DST
    
    return epoch;
    logDebugMsg(ETHERNET_NTP_5, ETHERNET_NTP_5);
  }
}

// send an NTP request to the time server at the given address 
void sendNTPpacket(IPAddress& address, byte *packetBuffer)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE); 
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49; 
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:      
  logDebugMsg(ETHERNET_NTP_6, ETHERNET_NTP_6);
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  logDebugMsg(ETHERNET_NTP_7, ETHERNET_NTP_7);
  Udp.write(packetBuffer,NTP_PACKET_SIZE);
  logDebugMsg(ETHERNET_NTP_8, ETHERNET_NTP_8);
  Udp.endPacket(); 
  logDebugMsg(ETHERNET_NTP_9, ETHERNET_NTP_9);
}

int dstOffset (DateTime time)
{
  //Receives unix epoch time and returns seconds of offset for local DST
  //Valid thru 2099 for US only, Calculations from "http://www.webexhibits.org/daylightsaving/i.html"
  //Code idea from jm_wsb @ "http://forum.arduino.cc/index.php/topic,40286.0.html"
  //Get epoch times @ "http://www.epochconverter.com/" for testing
  //DST update wont be reflected until the next time sync
  int beginDSTDay = (14 - (1 + time.year() * 5 / 4) % 7);  
  int beginDSTMonth=3;
  int endDSTDay = (7 - (1 + time.year() * 5 / 4) % 7);
  int endDSTMonth=11;
  if (((time.month() > beginDSTMonth) && (time.month() < endDSTMonth))
    || ((time.month() == beginDSTMonth) && (time.day() > beginDSTDay))
    || ((time.month() == beginDSTMonth) && (time.day() == beginDSTDay) && (time.hour() >= 2))
    || ((time.month() == endDSTMonth) && (time.day() < endDSTDay))
    || ((time.month() == endDSTMonth) && (time.day() == endDSTDay) && (time.hour() < 1)))
    return (3600);  //Add back in one hours worth of seconds - DST in effect
  else
    return (0);  //NonDST
}

void displayTime(DateTime& dateNow)
{
  char timeStr[10];
  
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print(F("Time is:"));
  lcd.setCursor(0,1); //Start at character 0 on line 1
  getTimeAsString(dateNow, timeStr, LONG_TIME_FORMAT);
  lcd.print(timeStr);
  delaySecsWithWatchdog(2);
  lcd.noBacklight();
}

boolean isValidHour(DateTime& dateNow)
{
  if (dateNow.hour() <= 23)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

ISR(WDT_vect) 
{ // Watchdog interrupt @ 8 sec. interval
  if(!--wdtCount) 
  { // Decrement sleep interval counter...
    // If it reaches zero, force a processor reset
    wdt_enable(WDTO_15MS); // turn on the WatchDog and allow it to fire
    while(1);
  }
}

