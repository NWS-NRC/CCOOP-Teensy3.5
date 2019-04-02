#include <MMTS.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
//#include <NWS_XBee.h>
#include <WProgram.h>
#include "GPS.h"
#include <EEPROM.h>
#include <math.h>
#undef int
#include <stdio.h>

#define LTC_CS 30
#define LTC_MISO  12
#define LTC_SCK  13
//
#define LTC_DELAY           5 // 5 usec delay for commands on the i2c bus

long tempMillis = 0;

long vcc = 5.0;   // only used for display purposes, if used set to the measured Vcc.
float pad = 20000; // balance/pad resistor value, set this to the measured resistance of your pad resistor
float volt = 0.0;
long int ltw = 0;
byte b0 = 0x00;
byte sig = 0x00;
float v_ref = 4.096; // Reference Voltage, 5.0 Volt for LT1021 or 3.0 for LP2950-3

int cnt = 0;

float temp = 0.0;

int led_pin = 24;
int vref_Pin = 39;
unsigned long ledTimer = 0UL;

File myFile;
byte sdReady = 0;

#define usb Serial
#define esp Serial1
#define nimbus Serial2
#define fpr Serial3
HardwareSerial &gpsSerial = Serial4;
GPS gps(&gpsSerial, true);

byte gpsActive = 0;
int gpsLoss = 0;

const int chipSelect = BUILTIN_SDCARD;//SD Card SPI select line

/************************RTC Items**********************************************/
#define RTC_I2C_ADDRESS 0x68  // This is the I2C address of the Real Time Clock
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
byte thisMonth; //used to check when month rolls over for SD operations.
byte offset = 6;
boolean dst_Observe = true;
boolean timeUpdate = false;
boolean timeUpdated = false;
unsigned long timeUpdateDelay = 0UL;
byte tempYear, tempMonth, tempDay, tempHour, tempMinute, tempSecond;
/*******************************************************************************/

/***********************Observer Selected Global Observation Time***************/
byte obRstTime = 7; // hour value to reset global readings. midnight by default until changed.
/*******************************************************************************/

/************************Battery Check Items***********************************
  const float refVolt = 5.0;
  const float R1 = 20000.0;
  const float R2 = 10000.0;
  const int battV_Pin = A1;
  float battV = 0.0;
  int battV_TX = 0;
  /*******************************************************************************/

byte function = 0x00; //  Function to be performed.

boolean menuReprint = false;

byte probeAlert = 0; //alert indicator for thermistor line: 1 = open, 2 = short
byte rebootAlert = 0; //alert indicator for reboot 1+ days from last recorded memory entry
byte updateDailyVals = 0; //variable to annotate if daily min/max needs to be updated in memory. 0 = no, 1 = yes.
byte dLogUpdated = 0; // indicates if the daily min/max info has been written to MRAM just before new day.
byte updateDailyGlobalVals = 0;//variable to annotate if global min/max needs to be updated in memory. 0 = no, 1 = yes.
byte obLogUpdated = 0; // indicates if the global ob and global min/max info has been written to MRAM at global reset time.
byte hLogUpdated = 0; // indicates if the hourly info has been written to MRAM at the top of each hour.
byte fmLogUpdated = 0; // indicates if the 5-Minute info has been written to SD Card.
byte dt_Updated = 0; //indicates if the weekly RTC sync has occured with either GPS or NTP.
byte boot = 1; // establish that the unit is booting up.
byte menuActive = 0;


/**************Modes***********/
uint32_t add = 0x00; //address variable to assign where memory pulls are coming from.

byte obMode = 0;
byte mmMode = 0;
byte hourlyMode = 0;

int temperature = 0;
int lastTemp = 0;
boolean tempSent = false;

String readingString = "";
boolean reading = false;
boolean fprUpdate = false;


int minTemp = 2555;
byte minHr = 0;
byte minMin = 0;
int maxTemp = -999;
byte maxHr = 0;
byte maxMin = 0;

int minObTemp = 2555;
byte minOb_Hr = 0;
byte minOb_Min = 0;
int maxObTemp = -999;
byte maxOb_Hr = 0;
byte maxOb_Min = 0;

float outChassisTemp = 0.0;
int outChassisTemp_TX = 0;

char* mmFiles[] = {
  "nullMM.txt", "janMM.txt", "febMM.txt", "marMM.txt", "aprMM.txt",
  "mayMM.txt", "junMM.txt", "julMM.txt",  "augMM.txt", "sepMM.txt",
  "octMM.txt", "novMM.txt", "decMM.txt"
};

char* obFiles[] = {
  "nullOBS.txt", "janOBS.txt", "febOBS.txt", "marOBS.txt", "aprOBS.txt",
  "mayOBS.txt", "junOBS.txt", "julOBS.txt", "augOBS.txt", "sepOBS.txt",
  "octOBS.txt", "novOBS.txt", "decOBS.txt"
};

char* hrFiles[] = {
  "nullHR.txt", "janHR.txt", "febHR.txt", "marHR.txt", "aprHR.txt",
  "mayHR.txt", "junHR.txt", "julHR.txt", "augHR.txt", "sepHR.txt",
  "octHR.txt", "novHR.txt", "decHR.txt"
};

char* fiveMinFiles[] = {
  "nullFM.txt", "janFM.txt", "febFM.txt", "marFM.txt", "aprFM.txt",
  "mayFM.txt", "junFM.txt", "julFM.txt", "augFM.txt", "sepFM.txt",
  "octFM.txt", "novFM.txt", "decFM.txt"
};

char siteID[6] = {'0', '0', '0', '0', '0', '0'};

const char space[]  = " ";
const char cData[]  = "Daily Data";
const char obLabel[]  = "Observation Data";
const char tempLabel[]  =  "Temp:";
const char tempLabelLong[] = "Temperature";
const char dateLabel[] = "Date:";
const char timeLabel[] = "Time:";

const char* days[] = {
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};
const char* months[] = {
  "Null", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul",
  "Aug", "Sep", "Oct", "Nov", "Dec"
};

const char zero[]  =  "0";
byte timeZero = 0;
const char doubleZero[]  =  "00";
const char one[]  =  "1";
const char lCap[] = "L";

const char slashLabel[]  = "/";
const char tab[] = "\t";
const char colon[]  =  ":";
const char pound[]  = "#";
const char plus[]  =  "+";
const char comma[]  =  ",";
const char loLabel[]  =  "Lo:";
const char hiLabel[]  =  "Hi:";

#define dAddStart  0x000001F4 //start daily min/max recording at address 500. (35 days needs 385 bytes)
#define obAddStart  0x000003E8 //start daily observation recording at address 1000. (35 days needs 455 bytes)
//#define hAddStart  0x000007D0 //start hourly value recording at address 2000. (35 days needs 5040 bytes)
#define mnMxData   0x00000190 //start of current day min/max values at address 400. (8 bytes needed)
#define obData    0x000001C2 //start of current day observation values at address 450. (8 bytes needed)
#define obRst_T   0x0000000A //observation reset time value at address 10.
#define siteIDMem 0x0000000F //address to start storage of siteID.

uint8_t eeBuff[32];

unsigned long tempCheckTimer = 0UL;

void setup()
{
  pinMode(LTC_CS, OUTPUT);
  pinMode(vref_Pin, OUTPUT);
  digitalWrite(vref_Pin, HIGH);
  digitalWrite(LTC_CS, HIGH);
  SPI.begin();
  Wire.begin(); //start up I2C comms
  usb.begin(115200); // start usb serial port
  delay(5);
  esp.begin(9600); //start up MMTS comms on Serial 2
  delay(5);
  fpr.begin(9600); //start up MMTS comms on Serial 2
  delay(5);
  nimbus.begin(9600); //start up MMTS comms on Serial 2
  delay(5);
  gps.startSerial(9600);  //start up gps comms on Serial4
  //delay(5000);
  unitInfo();
  //delay(2000);
  usb.println(F("Booting....."));
  //delay(1000);
  usb.println(F("Preparing EEPROM communications....."));
  byte tRST = EEPROM.read(obRst_T);
  usb.print(F("ObRst Time in Mem:"));
  usb.println(tRST, DEC);
  if (tRST >= 0 && tRST <= 23)
  {
    obRstTime = tRST;
    usb.println(F("Using ObRst Time from Memory"));
  }
  else
  {
    saveAutoReset();
    usb.print(F("Default ObRst Time Used: "));
    usb.println(obRstTime, DEC);
  }
  pullSiteID_Mem();

  getThermReading(); // pull thermistor/internal chassis temp

  //See if unit was rebooted same day. If so, don't discard data, else current day data needs to be cleared.
  getDateTime(); // pull current date/time from RTC and plug into variables
  if (year == 0)
  {
    delay(2000);
    if (gps.sentenceAvailable())
    {
      gps.parseSentence();
      gps_SetTime();
      getDateTime();
    }
  }
  //read last month/day logged in mram @ address 0x190
  byte tempMonth = EEPROM.read(mnMxData);
  byte tempDay = EEPROM.read(mnMxData + 1);
  usb.println(month, DEC);
  usb.println(tempMonth, DEC);
  usb.println(dayOfMonth, DEC);
  usb.println(tempDay, DEC);

  if (tempMonth == month && tempDay == dayOfMonth)
  {
    pullCurrentData();//pull stored memory to populate current min/max variables(current and global)
    usb.println(F("Same Day Reboot: Current Memory Values Retained"));
  }
  else
  {
    usb.println(F("New Day Reboot: Default Initialization Values Used"));
    pullSiteID_Mem(); //pull saved site ID from memory
    rebootAlert = 1; //use default boot variables

  }
  if (tempMonth != month)
  {
    newSD_Month();
    thisMonth = month;
  }

  /****************************Setup SD Card**********************************/
  usb.print("\r\nInitializing SD card...");
  delay(1000);
  if (!SD.begin(chipSelect))
  {
    usb.println("initialization failed!");
    sdReady = 0;
    usb.println("initialization failed. Things to check:");
    usb.println("* is a card is inserted?");
    usb.println("* Is your wiring correct?");
    usb.println("* did you change the chipSelect pin to match your shield or module?");
  }
  else
  {
    usb.println("Wiring is correct and a card is present.");
    sdReady = 1;
    delay(1000);
  }
  if (sdReady)
  {
    sdInit();
    usb.println("SD Files Ready");
    delay(1000);
  }
  usb.println(F("Boot Sequence Completed....."));
  delay(1000);
  usb.print(F("Site ID#: "));
  int i;
  for ( i = 0; i < 6; i++)
  {
    usb.print(siteID[i]);
  }
  delay(1000);
  usb.println(F("\r\nGathering Initial Temp Reading....."));
  /***************************************************************************/
  getCurrentTemp();//get most current temp
  delay(100);
  usb.println(F("\nTech Request:"));
  unsigned long checkTimer = millis();
  while ((millis() - checkTimer < 10000UL) && usb.available() == 0)
  {
    delay(50);
  }
  if (usb.available() != 0)
  {
    usb.println(F("Processing Tech Request!"));
    doFunction();
  }
  if (menuActive == 1)
  {
    while (menuActive == 1)
    {
      doMenuFunction();
      delay(100);
    }
  }
  //usb.println(F("Setting up WatchDog Timer for Sleep Operations."));
  //establish_WDT();
  usb.println(F("Beginning Normal Operation....."));
  delay(100);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  ledTimer = millis();
  tempCheckTimer = millis();
  tempMillis = millis();
}

/*************************************Main Program***********************************************/
void loop()
{
  /**************************Checking GPS Operation*******************************/
  if (gps.sentenceAvailable())
  {
    gps.parseSentence();
    //usb.println(F("Parsed"));
    gpsActive = 1;
    gpsLoss = 0;
  }
  else
  {
    if (gpsLoss < 100)
    {
      gpsLoss++;
    }
    else
    {
      gpsActive = 0;
    }
  }
  /*****************************GPS Check Complete********************************/

  checkTimers_Rollover();

  checkFPR();

  if (timeUpdated == false && timeUpdate == true && millis() >= timeUpdateDelay)
  {
    year = tempYear;
    month = tempMonth;
    dayOfMonth = tempDay;
    hour = tempHour;
    minute = tempMinute + 1;
    second = 2;
    ntpUpdateTime();
    timeUpdate = false;
    timeUpdated = true;
  }

  if ((millis() - tempCheckTimer) > 15000UL) // if 15 second timer expires, get temp and reset timer if temp is valid.
  {
    getDateTime();
    getCurrentTemp();
    getThermReading();
    transmitUpdate();
    if (updateDailyVals)
    {
      dailyUpdate();
      updateDailyVals = 0;
    }
    if (updateDailyGlobalVals)
    {
      ob_Update();
      updateDailyGlobalVals = 0;
    }
  }

  checkFPR();

  /*****************Hourly Ops********************/
  if (minute == 0 && hLogUpdated == 0)
  {
    logHourly();
    hLogUpdated = 1;
  }

  if (minute >= 5 && hLogUpdated == 1)
  {
    hLogUpdated = 0;
  }
  /**********************************************/

  /****************Daily Min/Max Ops*************/
  if ((hour == 23 && minute == 59) && dLogUpdated == 0)
  {
    logDaily();
    dLogUpdated = 1;
    if (rebootAlert)
    {
      rebootAlert = 0;
    }
  }
  if ((hour == 0 && minute >= 0) && dLogUpdated == 1)
  {
    dLogUpdated = 0;
    minTemp = 2555;
    minHr = 0;
    minMin = 0;
    maxTemp = -999;
    maxHr = 0;
    maxMin = 0;
    if (month != thisMonth)
    {
      newSD_Month(); //delete new month history files on SD card so that they are fresh for the month.
      thisMonth = month;
    }
  }

  if (hour == 0 && minute == 7 && timeUpdated == true)
  {
    timeUpdated == false;
    timeUpdate = false;
  }

  if (dayOfWeek == 0 && dt_Updated == 1)
  {
    dt_Updated = 0;
  }
  /**********************************************/

  /***************atOb Data Ops****************/
  if ((hour == obRstTime && minute == 0) && obLogUpdated == 0)
  {
    logOb();
    minObTemp = 2555;
    minOb_Hr = 0;
    minOb_Min = 0;
    maxObTemp = -999;
    maxOb_Hr = 0;
    maxOb_Min = 0;
    obLogUpdated = 1;
  }

  if ((hour == obRstTime && minute > 0) && obLogUpdated == 1)
  {
    obLogUpdated = 0;
  }
  /**********************************************/

  /**************5-Minute Log********************/
  if (minute % 5 == 0 && fmLogUpdated == 0)
  {
    logFM();
    fmLogUpdated = 1;
  }
  if (minute % 5 != 0 && fmLogUpdated == 1)
  {
    fmLogUpdated = 0;
  }

  /**********************************************/
  //enterSleep();
  checkESPSerial();
  if (ledTimer > millis())
  {
    ledTimer = millis();
  }
  if ((millis() - ledTimer) > 2000UL)
  {
    changeLED();
  }
}
/*******************************************End of Main Program*********************************/


void setDateTime() // Sets the date and time from the RTC
{
  second = (byte) ((usb.read() - 48) * 10 + (usb.read() - 48)); // Use of (byte) type casting and ascii math to achieve result.
  minute = (byte) ((usb.read() - 48) * 10 +  (usb.read() - 48));
  hour  = (byte) ((usb.read() - 48) * 10 +  (usb.read() - 48));
  dayOfWeek = (byte) (usb.read() - 48);
  dayOfMonth = (byte) ((usb.read() - 48) * 10 +  (usb.read() - 48));
  month = (byte) ((usb.read() - 48) * 10 +  (usb.read() - 48));
  year = (byte) ((usb.read() - 48) * 10 +  (usb.read() - 48));
  usb.println(second, DEC);
  usb.println(minute, DEC);
  usb.println(hour, DEC);
  usb.println(dayOfWeek, DEC);
  usb.println(dayOfMonth, DEC);
  usb.println(month, DEC);
  usb.println(year, DEC);

  second = 5;

  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write((byte)0x00);
  Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
  // bit 6 (also need to change readDateDs1307)
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(dayOfMonth));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
  delay(5);
  getDateTime();

}

void getDateTime()// Gets the date and time from the RTC
{
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write((byte)0x00);
  Wire.endTransmission();

  Wire.requestFrom(RTC_I2C_ADDRESS, 7);

  // A few of these need masks because certain bits are control bits
  second     = bcdToDec(Wire.read() & 0x7f);
  minute     = bcdToDec(Wire.read());
  hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  dayOfWeek  = bcdToDec(Wire.read());
  dayOfMonth = bcdToDec(Wire.read());
  month      = bcdToDec(Wire.read());
  year       = bcdToDec(Wire.read());

  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write((byte)0x11);
  Wire.endTransmission();
  Wire.requestFrom(RTC_I2C_ADDRESS, 2);
  byte tempMSB = Wire.read();
  //usb.println(tempMSB, BIN);
  byte tempLSB = Wire.read() >> 6;
  //usb.println(tempLSB, BIN);
  //outChassisTemp = ((((short)tempMSB << 8) | (short)tempLSB) >> 6) / 4.0;
  //outChassisTemp = (outChassisTemp * 1.8) + 32.0; // Convert Celcius to Fahrenheit
  outChassisTemp_TX = (int(outChassisTemp * 10));
  usb.print(F("Enclosure Temp: "));
  usb.print(outChassisTemp, 1);
  usb.println(F(" F"));
  usb.println(F(""));
  if (hour < 10) {
    usb.print(F("0"));
  }
  usb.print(hour, DEC);
  usb.print(F(":"));
  if (minute < 10) {
    usb.print(F("0"));
  }
  usb.print(minute, DEC);
  usb.print(F(" "));
  if (month < 10) {
    usb.print(F("0"));
  }
  usb.print(month, DEC);
  usb.print(F("/"));
  if (dayOfMonth < 10) {
    usb.print(F("0"));
  }
  usb.print(dayOfMonth, DEC);
  usb.print(F("/"));
  if (year < 10) {
    usb.print(F("0"));
  }
  usb.println(year, DEC);
}

void ntpUpdateTime()
{
  usb.println(second, DEC);
  usb.println(minute, DEC);
  usb.println(hour, DEC);
  usb.println(dayOfWeek, DEC);
  usb.println(dayOfMonth, DEC);
  usb.println(month, DEC);
  usb.println(year, DEC);

  second = 5;

  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write((byte)0x00);
  Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
  // bit 6 (also need to change readDateDs1307)
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(dayOfMonth));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
  delay(5);
  getDateTime();
}

byte decToBcd(byte val)
{
  return ( (val / 10 * 16) + (val % 10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val / 16 * 10) + (val % 16) );
}

void transmitUpdate()
{
  unsigned long memTimer = 0UL;
  int tempVal = 0;
  float floatVal = 0.0;
  /*byte txFault = 0;
  byte transferSuccess = 0;
  byte attempts = 0;
  int linesSent = 0;
  String updateString = "";
  String displayString = "";
  clearfpr();
  uint16_t crc = 0; //variable to send a check to make sure data was received correctly.
  crc += byte(month);
  crc += byte(dayOfMonth);
  crc += byte(year);
  crc += byte(hour);
  crc += byte(minute);
  tempVal = temperature;
  crc += highByte(tempVal);
  crc += lowByte(tempVal);
  tempVal = minTemp;
  crc += highByte(tempVal);
  crc += lowByte(tempVal);
  crc += byte(minHr);
  crc += byte(minMin);
  tempVal = maxTemp;
  crc += highByte(tempVal);
  crc += lowByte(tempVal);
  crc += byte(maxHr);
  crc += byte(maxMin);
  crc += byte(obRstTime);
  tempVal = minObTemp;
  crc += highByte(tempVal);
  crc += lowByte(tempVal);
  crc += byte(minOb_Hr);
  crc += byte(minOb_Min);
  tempVal = maxObTemp;
  crc += highByte(tempVal);
  crc += lowByte(tempVal);
  crc += byte(maxOb_Hr);
  crc += byte(maxOb_Min);
  //crc += highByte(rssi);
  //crc += lowByte(rssi);
  crc += highByte(outChassisTemp_TX);
  crc += lowByte(outChassisTemp_TX);
  //crc += highByte(battV_TX);
  //crc += lowByte(battV_TX);
  crc += byte(probeAlert);
  crc += byte(rebootAlert);
  updateString += "#D";
  updateString += ",";
  updateString += (month);
  updateString += ",";
  updateString += (dayOfMonth);
  updateString += ",";
  updateString += (year);
  updateString += ",";
  updateString += (hour);
  updateString += ",";
  updateString += (minute);
  updateString += ",";
  tempVal = temperature;
  updateString += highByte(tempVal);
  updateString += ",";
  updateString += lowByte(tempVal);
  updateString += ",";
  tempVal = minTemp;
  updateString += highByte(tempVal);
  updateString += ",";
  updateString += lowByte(tempVal);
  updateString += ",";
  updateString += (minHr);
  updateString += ",";
  updateString += (minMin);
  updateString += ",";
  tempVal = maxTemp;
  updateString += highByte(tempVal);
  updateString += ",";
  updateString += lowByte(tempVal);
  updateString += ",";
  updateString += (maxHr);
  updateString += ",";
  updateString += (maxMin);
  updateString += ",";
  updateString += (obRstTime);
  updateString += ",";
  tempVal = minObTemp;
  updateString += highByte(tempVal);
  updateString += ",";
  updateString += lowByte(tempVal);
  updateString += ",";
  updateString += (minOb_Hr);
  updateString += ",";
  updateString += (minOb_Min);
  updateString += ",";
  tempVal = maxObTemp;
  updateString += highByte(tempVal);
  updateString += ",";
  updateString += lowByte(tempVal);
  updateString += ",";
  updateString += (maxOb_Hr);
  updateString += ",";
  updateString += (maxOb_Min);
  updateString += ",";
  //updateString += highByte(rssi);
  //updateString += ",";
  //updateString += lowByte(rssi);
  //updateString += ",";
  updateString += highByte(outChassisTemp_TX);
  updateString += ",";
  updateString += lowByte(outChassisTemp_TX);
  updateString += ",";
  //updateString += highByte(battV_TX);
  //updateString += ",";
  //updateString += lowByte(battV_TX);
  //updateString += ",";
  updateString += (probeAlert);
  updateString += ",";
  updateString += (rebootAlert);
  updateString += ",";
  updateString += highByte(crc);
  updateString += ",";
  updateString += lowByte(crc);
  updateString += ",";*/
  /*****sent to com port for verification of data going out****/
  usb.println(F("\n*************************************************"));
  usb.print(F("Date: "));
  usb.print(month);
  usb.print(colon);
  if (dayOfMonth < 10)
  {
    usb.print(zero);
  }
  usb.print(dayOfMonth);
  usb.print(colon);
  if (year < 10)
  {
    usb.print(zero);
  }
  usb.println(year);
  usb.print(F("Time: "));
  usb.print(hour);
  usb.print(colon);
  if (minute < 10)
  {
    usb.print(zero);
  }
  usb.println(minute);
  floatVal = float(temperature);
  floatVal = floatVal / 10;
  usb.print(F("Temperature: "));
  usb.println(floatVal, 1);
  floatVal = float(minTemp);
  floatVal = floatVal / 10;
  usb.print(F("\nMinTemp: "));
  usb.print(floatVal, 1);
  usb.print(F("\t"));
  usb.print(minHr);
  usb.print(colon);
  if (minMin < 10)
  {
    usb.print(zero);
  }
  usb.println(minMin);
  floatVal = float(maxTemp);
  floatVal = floatVal / 10;
  usb.print(F("MaxTemp: "));
  usb.print(floatVal, 1);
  usb.print(F("\t"));
  usb.print(maxHr);
  usb.print(colon);
  if (maxMin < 10)
  {
    usb.print(zero);
  }
  usb.println(maxMin);
  usb.print(F("\nAt-Ob Reset Time: "));
  usb.println(obRstTime);
  floatVal = float(minObTemp);
  floatVal = floatVal / 10;
  usb.print(F("At-Ob MinTemp: "));
  usb.print(floatVal, 1);
  usb.print(F("\t"));
  usb.print(minOb_Hr);
  usb.print(colon);
  if (minOb_Min < 10)
  {
    usb.print(zero);
  }
  usb.println(minOb_Min);
  floatVal = float(maxObTemp);
  floatVal = floatVal / 10;
  usb.print(F("At-Ob MaxTemp: "));
  usb.print(floatVal, 1);
  usb.print(F("\t"));
  usb.print(maxOb_Hr);
  usb.print(colon);
  if (maxOb_Min < 10)
  {
    usb.print(zero);
  }
  usb.println(maxOb_Min);
  usb.print(F("\nProbe Status: "));
  usb.println(probeAlert);
  usb.print(F("Reboot Status: "));
  usb.println(rebootAlert);
  usb.println(F("*************************************************"));

  if (gps.newValuesSinceDataRead())
  {
    gps.dataRead();
    usb.printf("\nLocation: %f, %f altitude %f\n\r", gps.latitude, gps.longitude, gps.altitude);
  }
}


void getCurrentTemp()
{
  int tempCheck = 0;
  int tempCheck1 = 0;
  int tempCheck2 = 0;
  boolean match = false;
  byte valid = 0;
  if (boot == 0)
  {
    mmts.getCurrentTemp();
    tempCheck1 = atoi(currentTemp);
    usb.println(float(tempCheck1) / 10);
    checkFPR();
    delay(1000);
    mmts.getCurrentTemp();
    checkFPR();
    tempCheck2 = atoi(currentTemp);
    usb.println(float(tempCheck2) / 10);
    if (tempCheck1 == tempCheck2)
    {
      tempCheck = tempCheck2;
      match = true;
    }
    if (match == true)
    {
      if (tempCheck != NULL && tempCheck != (-999) && tempCheck != 2555)
      {
        if (((lastTemp - 100) <= tempCheck) && ((lastTemp + 100) >= tempCheck)) // +/- 30 degree check
        {
          temperature = tempCheck;
          lastTemp = temperature;
          valid = 1;
          tempCheckTimer = millis();
        }
      }
    }
  }
  else if (boot == 1)
  {
    delay(11000);
    mmts.getCurrentTemp();
    tempCheck1 = atoi(currentTemp);
    usb.println(float(tempCheck1) / 10);
    checkFPR();
    delay(1000);
    mmts.getCurrentTemp();
    tempCheck2 = atoi(currentTemp);
    usb.println(float(tempCheck2) / 10);
    checkFPR();
    if (tempCheck1 == tempCheck2)
    {
      tempCheck = tempCheck2;
    }
    if (tempCheck != NULL && tempCheck != 0 && tempCheck != (-999) && tempCheck != 2555)
    {
      boot = 0;
      usb.println(F("Boot sequence completed. Normal temps acquired!"));
      temperature = tempCheck;
      lastTemp = temperature;
    }
  }
  usb.print(F("\r\nNimbus Temp: "));
  if (tempCheck != NULL)
  {
    usb.println(tempCheck);
  }
  else
  {
    usb.println(F("Temperature samples did not match!"));
  }
  if (boot == 0)
  {
    usb.print(F("Recorded Temp: "));
    usb.println(temperature);
    usb.print(float(temperature) / 10);
    //float espTemp = (float(temperature) / 10);
    //String espTempString = String(espTemp);
    usb.println(F(" F"));
    /*String espString = "#T";
      espString = espString + espTempString;
      esp.println(espString);
      usb.println(espString);*/

    if (tempCheck == -999)
    {
      if (probeAlert != 1)
      {
        probeAlert = 1;
      }
      usb.println(F("Probe Circuit Open"));
    }
    if (tempCheck == 2555)
    {
      if (probeAlert != 2)
      {
        probeAlert = 2;
      }
      usb.println(F("Probe Circuit Shorted"));
    }
    if ((tempCheck != -999) && (tempCheck != 2555))
    {
      probeAlert = 0;
    }
    if (valid == 1)
    {
      if (minTemp > temperature)
      {
        minTemp = temperature;
        minHr = hour;
        minMin = minute;
        updateDailyVals = 1;
      }
      if (maxTemp < temperature)
      {
        maxTemp = temperature;
        maxHr = hour;
        maxMin = minute;
        updateDailyVals = 1;
      }
      if (minObTemp > temperature)
      {
        minObTemp = temperature;
        minOb_Hr = hour;
        minOb_Min = minute;
        updateDailyGlobalVals = 1;
      }
      if (maxObTemp < temperature)
      {
        maxObTemp = temperature;
        maxOb_Hr = hour;
        maxOb_Min = minute;
        updateDailyGlobalVals = 1;
      }
    }
  }
  if (minute % 5 == 0 && tempSent == false)
  {
    reportTemp();
    tempSent = true;
  }
  if (minute % 5 != 0 && tempSent == true)
  {
    tempSent = false;
  }
}

void reportTemp()
{
  String espString = "#T";
  if (year < 10)
  {
    espString += "0";
  }
  espString += String(year);
  espString += "/";
  if (month < 10)
  {
    espString += "0";
  }
  espString += String(month, DEC);
  espString += "/";
  if (dayOfMonth < 10)
  {
    espString += "0";
  }
  espString += String(dayOfMonth, DEC);
  espString += ",,,,,";
  if (hour < 10)
  {
    espString += "0";
  }
  espString += String(hour, DEC);
  espString += ":";
  if (minute < 10)
  {
    espString += "0";
  }
  espString += String(minute, DEC);
  espString += ":";
  if (second < 10)
  {
    espString += "0";
  }
  espString += String(second, DEC);
  espString += ",";
  float tF = float(temperature);
  tF = tF / 10;
  espString += String(tF, 1);
  esp.println(espString);
  usb.println(espString);
}


void clearObData()
{
  minObTemp = 2555;
  minOb_Hr = 0;
  minOb_Min = 0;
  maxObTemp = -999;
  maxOb_Hr = 0;
  maxOb_Min = 0;
}


void resetData()
{
  minTemp = 2555;
  minHr = 0;
  minMin = 0;
  maxTemp = -999;
  maxHr = 0;
  maxMin = 0;
  minObTemp = 2555;
  minOb_Hr = 0;
  minOb_Min = 0;
  maxObTemp = -999;
  maxOb_Hr = 0;
  maxOb_Min = 0;
}

/*************************keep function in program in case switches/buttons added later***************************/
/*void check_switches()// checks all buttons/switches and includes software debounce features.
  {
  static byte previousstate[NUMBUTTONS];
  static byte currentstate[NUMBUTTONS];
  static long lasttime;
  byte index;

  if (millis() < lasttime) {
  // we wrapped around, lets just try again
  lasttime = millis();
  }

  if ((lasttime + DEBOUNCE) > millis()) {
  // not enough time has passed to debounce
  return;
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lasttime = millis();

  for (index = 0; index < NUMBUTTONS; index++) {
  justpressed[index] = 0;       // when we start, we clear out the "just" indicators
  justreleased[index] = 0;

  currentstate[index] = digitalRead(buttons[index]);   // read the button


  //usb.print(index, DEC);
  //usb.print(": cstate=");
  //usb.print(currentstate[index], DEC);
  //usb.print(", pstate=");
  //usb.print(previousstate[index], DEC);
  //usb.print(", press=");


  if (currentstate[index] == previousstate[index]) {
  if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {
  // just pressed
  justpressed[index] = 1;
  }
  else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) {
  // just released
  justreleased[index] = 1;
  }
  pressed[index] = !currentstate[index];  // remember, digital HIGH means NOT pressed
  }
  //usb.println(pressed[index], DEC);
  previousstate[index] = currentstate[index];   // keep a running tally of the buttons
  }
  }*/
/*****************************************************************************************************************/

/**************************Mode to send commands from USB directly to MMTS CCA************************************/
void mmtsDirect()
{
  usb.print(F("\n\rMMTS Pass-Thru Mode Enabled, Hit Escape Key to Exit\n\r"));
  boolean passThruMode = true;

  while (passThruMode)
  {
    while (usb.available() > 0)
    {
      char inChar = char(usb.read());
      if (inChar == char(27))
      {
        usb.print(F("\nReturning to normal operation......"));
        delay(5000);
        while (nimbus.available() > 0)
        {
          byte inByte = nimbus.read();
          delay(5);
        }
        passThruMode = false;
      }
      else
      {
        nimbus.print(inChar);
      }
    }

    if (nimbus.available() > 0)
    {
      while (nimbus.available() > 0)
      {
        char inChar = char(nimbus.read());
        usb.print(inChar);
      }
    }
    delay(20);
  }
}

void doFunction()
{
  function = usb.read();
  usb.print(F("function: "));
  usb.println(char(function));
  switch (function)
  {
    case '#':
      mmtsDirect();
      break;

    case '?':
      getD_UnitMenu();
      menuActive = 1;
      break;

    case 'h':
      getD_UnitMenu();
      menuActive = 1;
      break;

    case 'R':
      resetData();
      break;

    default:
      break;
  }
  usb.flush();
  clearusb();
  function = 0x00;
}

void doMenuFunction()
{
  boolean selection = false;
  function = usb.read();
  switch (function)
  {
    case '?':
      getD_UnitMenu();
      break;

    case 'h':
      getD_UnitMenu();
      break;

    case 'R':
      resetData();
      break;

    case '1':
      systemInfoSetup();
      break;

    case '2':
      unitInfo();
      break;

    case '3':
      getCurrentTemp();
      break;

    case '4':
      currentObValues();
      break;

    case '5':
      currentDailyValues();
      break;

    case '6':
      clearusb(); //clear out any data in serial buffer
      usb.println(F("\r\nTHIS OPERATION WILL COMPLETELY WIPE THE"));
      usb.println(F("BUILT IN MEMORY OF THE OUTDOOR UNIT.\r\n"));
      usb.println(F("Do you wish to continue? (Y/N)"));
      selection = false;
      while (selection == false)
      {
        while (usb.available() == 0)
        {
          delay(200);
        }
        byte selectByte = usb.read();
        switch (selectByte)
        {
          case 'Y':
            selection = true;
            eeClearData(int(mnMxData), 4096);
            saveAutoReset();
            saveSiteID();
            usb.println(F("Memory Clear Request Initiated."));
            break;

          case 'y':
            selection = true;
            eeClearData(int(mnMxData), 4096);
            saveAutoReset();
            saveSiteID();
            usb.println(F("Memory Clear Request Initiated."));
            break;

          case 'N':
            selection = true;
            usb.println(F("Memory Clear Request Aborted"));
            break;

          case 'n':
            selection = true;
            usb.println(F("Memory Clear Request Aborted"));
            break;

          case char(10): //newline character...do nothing
            break;

          case char(13): //carriage return character...do nothing
            break;

          default:
            usb.println(F("Invalid response. Please select Y/N!"));
            break;
        }
      }
      break;

    case '7':
      clearusb(); //clear out any data in serial buffer
      usb.println(F("\nTHIS OPERATION WILL COMPLETELY WIPE THE"));
      usb.println(F("SD CARD MEMORY OF THE OUTDOOR UNIT.\r\n"));
      usb.println(F("\nDo you wish to continue? (Y/N)"));
      selection = false;
      while (selection == false)
      {
        while (usb.available() == 0)
        {
          delay(200);
        }
        byte selectByte = usb.read();
        switch (selectByte)
        {
          case 'Y':
            selection = true;
            sdClearData();
            usb.println(F("SD Card Clear Request Initiated."));
            break;

          case 'y':
            selection = true;
            sdClearData();
            usb.println(F("SD Card Clear Request Initiated."));
            break;

          case 'N':
            selection = true;
            usb.println(F("SD Card Clear Request Aborted"));
            break;

          case 'n':
            selection = true;
            usb.println(F("SD Card Clear Request Aborted"));
            break;

          case char(10): //newline character...do nothing
            break;

          case char(13): //carriage return character...do nothing
            break;

          default:
            usb.println(F("Invalid response. Please select Y/N!"));
            break;
        }
      }
      break;

    case '8':
      clearusb(); //clear out any data in serial buffer
      usb.println(F("\nTHIS OPERATION WILL COMPLETELY WIPE ALL"));
      usb.println(F("HISTORY DATA OF THE OUTDOOR UNIT.\r\n"));
      usb.println(F("\nDo you wish to continue? (Y/N)"));
      selection = false;
      while (selection == false)
      {
        while (usb.available() == 0)
        {
          delay(200);
        }
        byte selectByte = usb.read();
        switch (selectByte)
        {
          case 'Y':
            selection = true;
            usb.println(F("Master Clear Request Initiated."));
            wipeHistory();
            delay(1000);
            saveAutoReset();
            saveSiteID();
            usb.println(F("Master Clear Completed."));
            break;

          case 'y':
            selection = true;
            usb.println(F("Master Clear Request Initiated."));
            wipeHistory();
            delay(1000);
            saveAutoReset();
            saveSiteID();
            usb.println(F("Master Clear Completed."));
            break;

          case 'N':
            selection = true;
            usb.println(F("Master Clear Request Aborted"));
            break;

          case 'n':
            selection = true;
            usb.println(F("Master Clear Request Aborted"));
            break;

          case char(10): //newline character...do nothing
            break;

          case char(13): //carriage return character...do nothing
            break;

          default:
            usb.println(F("Invalid response. Please select Y/N!"));
            break;
        }
      }
      break;

    case '9':
      clearusb(); //clear out any data in serial buffer
      getSD_Data();
      break;

    case char(10): //newline character...do nothing
      break;

    case char(13): //carriage return character...do nothing
      break;

    case char(27):
      usb.println(F(""));
      usb.println(F(" Exiting User Menu Mode......\n"));
      menuActive = 0;

    default:
      break;
  }
  function = 0x00;
}

void dailyUpdate()
{
  eeBuff[0] = month;
  eeBuff[1] = dayOfMonth;
  eeBuff[2] = year;
  eeBuff[3] = highByte(minTemp);
  eeBuff[4] = lowByte(minTemp);
  eeBuff[5] = minHr;
  eeBuff[6] = minMin;
  eeBuff[7] = highByte(maxTemp);
  eeBuff[8] = lowByte(maxTemp);
  eeBuff[9] = maxHr;
  eeBuff[10] = maxMin;
  eeBuff[11] = thisMonth;
  eeWriteArray(int(mnMxData), 12);
  usb.println(F("Daily Values Updated"));
}

void logDaily()
{
  /*shiftDailyLog();
    eeBuff[0] = month;
    eeBuff[1] = dayOfMonth;
    eeBuff[2] = year;
    eeBuff[3] = highByte(minTemp);
    eeBuff[4] = lowByte(minTemp);
    eeBuff[5] = minHr;
    eeBuff[6] = minMin;
    eeBuff[7] = highByte(maxTemp);
    eeBuff[8] = lowByte(maxTemp);
    eeBuff[9] = maxHr;
    eeBuff[10] = maxMin;
    eeWriteArray(dAddStart, 11);*/
  usb.println(F("Daily Log Updated"));
  myFile = SD.open(mmFiles[int(month)], FILE_WRITE);
  if (myFile)
  {
    for (byte i = 0; i < 4; i++)
    {
      myFile.print(siteID[i]);
    }
    myFile.print(F(","));
    if (month < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(month, DEC);
    myFile.print(F("/"));
    if (dayOfMonth < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(dayOfMonth, DEC);
    myFile.print(F("/"));
    if (year < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(year, DEC);
    myFile.print(F(","));
    float tF = float(minTemp);
    tF = tF / 10;
    myFile.print(tF, 1);
    myFile.print(F(","));
    if (minHr < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(minHr, DEC);
    myFile.print(F(":"));
    if (minMin < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(minMin, DEC);
    myFile.print(F(","));
    tF = float(maxTemp);
    tF = tF / 10;
    myFile.print(tF, 1);
    myFile.print(F(","));
    if (maxHr < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(maxHr, DEC);
    myFile.print(F(":"));
    if (maxMin < 10)
    {
      myFile.print(F("0"));
    }
    myFile.println(maxMin, DEC);
    myFile.close();
  }
}


/*void shiftDailyLog()
  {
  //we want to overwrite the last entry with the first to last one and work out way back.
  //((min/max start address of 0x000001F4)+(11 entries * 35 days)- 11),(only shifting out one entry each day)
  uint32_t shiftAddress = (dAddStart + 385) - 11; //(-11 account for a position 0 start)
  mram.shiftData(shiftAddress, dAddStart, 11);
  usb.println(F("Daily Log Shifted"));
  }*/

void init_dailyMem()
{
  mmMode = 1;
  usb.print(F("Min/Max Mode Initiated: "));
  usb.println(mmMode);
  add = dAddStart;
  eeReadArray(int(add), 11);
  for (int x = 0; x < 11; x++)
  {
    usb.print(eeBuff[x]);
    usb.print(comma);
  }
  usb.println(space);
}

void ob_Update()
{
  eeBuff[0] = highByte(minObTemp);
  eeBuff[1] = lowByte(minObTemp);
  eeBuff[2] = minOb_Hr;
  eeBuff[3] = minOb_Min;
  eeBuff[4] = highByte(maxObTemp);
  eeBuff[5] = lowByte(maxObTemp);
  eeBuff[6] = maxOb_Hr;
  eeBuff[7] = maxOb_Min;
  eeWriteArray(int(obData), 8);
  usb.println(F("Global Values Updated"));
}

void logOb()
{
  //shiftObLog();
  usb.print(F("Temp At-Ob: "));
  usb.println(temperature);
  usb.print(F("Lo: "));
  usb.println(minObTemp);
  usb.print(F("Time: "));
  usb.print(minOb_Hr, DEC);
  usb.print(F(":"));
  usb.println(minOb_Min, DEC);
  usb.print(F("Hi: "));
  usb.println(maxObTemp);
  usb.print(F("Time: "));
  usb.print(maxOb_Hr, DEC);
  usb.print(F(":"));
  usb.println(maxOb_Min, DEC);
  String espString = "";
  espString += "#T";
  if (year < 10)
  {
    espString += "0";
  }
  espString += String(year);
  espString += "/";
  if (month < 10)
  {
    espString += "0";
  }
  espString += String(month, DEC);
  espString += "/";
  if (dayOfMonth < 10)
  {
    espString += "0";
  }
  espString += String(dayOfMonth, DEC);
  espString += ",";
  if (maxOb_Hr < 10)
  {
    espString += "0";
  }
  espString += String(maxOb_Hr, DEC);
  espString += ":";
  if (maxOb_Min < 10)
  {
    espString += "0";
  }
  espString += String(maxOb_Min, DEC);
  espString += ",";
  float tF = float(maxObTemp);
  tF = tF / 10;
  espString += String(tF, 1);
  espString += ",";
  if (minOb_Hr < 10)
  {
    espString += "0";
  }
  espString += String(minOb_Hr, DEC);
  espString += ":";
  if (minOb_Min < 10)
  {
    espString += "0";
  }
  espString += String(minOb_Min, DEC);
  espString += ",";
  tF = float(minObTemp);
  tF = tF / 10;
  espString += String(tF, 1);
  espString += ",";
  if (hour < 10)
  {
    espString += "0";
  }
  espString += String(hour, DEC);
  espString += ":";
  if (minute < 10)
  {
    espString += "0";
  }
  espString += String(minute, DEC);
  espString += ":";
  if (second < 10)
  {
    espString += "0";
  }
  espString += String(second, DEC);
  espString += ",";
  tF = float(temperature);
  tF = tF / 10;
  espString += String(tF, 1);
  esp.println(espString);
  usb.println(espString);
  /*eeBuff[0] = month;
    eeBuff[1] = dayOfMonth;
    eeBuff[2] = year;
    eeBuff[3] = highByte(temperature);
    eeBuff[4] = lowByte(temperature);
    eeBuff[5] = highByte(minObTemp);
    eeBuff[6] = lowByte(minObTemp);
    eeBuff[7] = minOb_Hr;
    eeBuff[8] = minOb_Min;
    eeBuff[9] = highByte(maxObTemp);
    eeBuff[10] = lowByte(maxObTemp);
    eeBuff[11] = maxOb_Hr;
    eeBuff[12] = maxOb_Min;
    eeWriteArray(obAddStart, 13);*/
  usb.println(F("Global Log Updated"));
  myFile = SD.open(obFiles[int(month)], FILE_WRITE);
  if (myFile)
  {
    for (byte i = 0; i < 4; i++)
    {
      myFile.print(siteID[i]);
    }
    myFile.print(F(","));
    if (month < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(month, DEC);
    myFile.print(F("/"));
    if (dayOfMonth < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(dayOfMonth, DEC);
    myFile.print(F("/"));
    if (year < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(year, DEC);
    myFile.print(F(","));
    float tF = float(temperature);
    tF = tF / 10;
    myFile.print(tF, 1);
    myFile.print(F(","));
    tF = float(minObTemp);
    tF = tF / 10;
    myFile.print(tF, 1);
    myFile.print(F(","));
    if (minOb_Hr < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(minOb_Hr, DEC);
    myFile.print(F(":"));
    if (minOb_Min < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(minOb_Min, DEC);
    myFile.print(F(","));
    tF = float(maxObTemp);
    tF = tF / 10;
    myFile.print(tF, 1);
    myFile.print(F(","));
    if (maxOb_Hr < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(maxOb_Hr, DEC);
    myFile.print(F(":"));
    if (maxOb_Min < 10)
    {
      myFile.print(F("0"));
    }
    myFile.println(maxOb_Min, DEC);
    myFile.close();
  }
}

/*void shiftObLog()
  {
  //we want to overwrite the last entry with the first to last one and work out way back.
  //((hourly start address of 0x000007D0)+(13 entries * 35 days)- 13),(only shifting out one entry each hour)
  uint32_t shiftAddress = (obAddStart + 455) - 13; //(-13 account for a position 0 start)
  mram.shiftData(shiftAddress, obAddStart, 13);
  usb.println(F("Global Log Shifted"));
  }*/

void init_obMem()
{
  obMode = 1;
  usb.println(obMode);
  usb.println(F("Observation History Initiated"));
  add = obAddStart;
  eeReadArray(int(add), 13);
  for (int x = 0; x < 13; x++)
  {
    usb.print(eeBuff[x]);
    usb.print(comma);
  }
  usb.println(space);
}

void logHourly()
{
  /*shiftHourlyLog();
    eeBuff[0] = month;
    eeBuff[1] = dayOfMonth;
    eeBuff[2] = year;
    eeBuff[3] = hour;
    eeBuff[4] = highByte(temperature);
    eeBuff[5] = lowByte(temperature);
    eeWriteArray(hAddStart, 6);*/
  usb.println(F("Hourly Log Updated"));
  myFile = SD.open(hrFiles[int(month)], FILE_WRITE);
  if (myFile)
  {
    for (byte i = 0; i < 4; i++)
    {
      myFile.print(siteID[i]);
    }
    myFile.print(F(","));
    if (month < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(month, DEC);
    myFile.print(F("/"));
    if (dayOfMonth < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(dayOfMonth, DEC);
    myFile.print(F("/"));
    if (year < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(year, DEC);
    myFile.print(F(","));
    if (hour < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(hour, DEC);
    myFile.print(F(":"));
    myFile.print(F("00"));
    myFile.print(F(","));
    float tF = float(temperature);
    tF = tF / 10;
    myFile.println(tF, 1);
    myFile.close();
  }
}



/*void shiftHourlyLog()
  {
  //we want to overwrite the last entry with the first to last one and work out way back.
  //((hourly start address of 0x000007D0)+(6 entries * 24 Hours * 35 days)- 5),(only shifting out one entry each hour)
  uint32_t shiftAddress = (hAddStart + 5040) - 6; //(-6 account for a position 0 start)
  mram.shiftData(shiftAddress, hAddStart, 6);
  usb.println(F("Hourly Log Shifted"));
  }*/

/*void init_hourlyMem()
  {
  hourlyMode = 1;
  usb.println(hourlyMode);
  usb.println(F("Hourly Mode Initiated"));
  add = hAddStart;
  eeReadArray(int(hAddStart), 6);
  for (int x = 0; x < 6; x++)
  {
    usb.print(eeBuff[x]);
    usb.print(comma);
  }
  usb.println(space);
  }*/


void logFM()
{
  myFile = SD.open(fiveMinFiles[int(month)], FILE_WRITE);
  if (myFile)
  {
    /*for (byte i = 0; i < 4; i++)
    {
      myFile.print(siteID[i]);
    }
    myFile.print(F(","));*/
    if (month < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(month, DEC);
    myFile.print(F("/"));
    if (dayOfMonth < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(dayOfMonth, DEC);
    myFile.print(F("/"));
    if (year < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(year, DEC);
    myFile.print(F(","));
    if (hour < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(hour, DEC);
    myFile.print(F(":"));
    if (minute < 10)
    {
      myFile.print(F("0"));
    }
    myFile.print(minute, DEC);
    myFile.print(F(","));
    float tF = float(temperature);
    tF = tF / 10;
    myFile.print(tF, 1);
    myFile.print(F(","));
    myFile.println(outChassisTemp, 1);
    //myFile.print(F(","));
    //myFile.println(battV, 1);
    myFile.close();
    usb.println(F("5-Minute Log Updated"));
  }
}

void pullSiteID_Mem()
{
  eeReadArray(int(siteIDMem), 6);
  for (byte y = 0; y < 6; y++)
  {
    siteID[y] = eeBuff[y];
    delay(5);
  }
  usb.println(space);
  delay(10);
}

void pullObRstTime_Mem()
{
  obRstTime = EEPROM.read(int(obRst_T));
  delay(5);
}

void saveAutoReset()
{
  EEPROM.write(int(obRst_T), obRstTime);
}

void saveSiteID()
{
  for (byte y = 0; y < 6; y++)
  {
    eeBuff[y] = siteID[y];
    delay(5);
  }
  eeWriteArray(int(siteIDMem), 6);
}

void userAutoReset_Update()
{
  byte resetBuff[2];
  int index = 0;
  boolean exit = false;
  usb.println(space);
  usb.print(F("Current 'AT-OB' Auto Reset Time: "));
  usb.print(obRstTime);
  usb.println(F("\r\nENTER THE NEW AUTO-RESET HOUR AS 2 DIGITS"));
  usb.println(F("Example:(0600 = 06, 1100 = 11, 2200 = 22)"));
  while (exit == false)
  {
    while (usb.available() == 0)
    {
      delay(200);
    }
    while (usb.available() > 0 || (index < 1 && exit == false))
    {
      if (usb.available() > 0)
      {
        byte entry = usb.read();
        if (entry == 48 || entry == 49 || entry == 50) // 0 or 1 or 2
        {
          usb.print(entry - 48);
          resetBuff[index] = entry - 48;
          index++;
        }
        else if (entry == char(27))
        {
          clearusb();
          usb.println(space);
          usb.println(F("EXITING 'AT-OB' AUTO-RESET ENTRY PROCESS"));
          index = 1;
          exit = true;
        }
        else
        {
          usb.println(F("INCORRECT ENTRY. FIRST DIGIT MUST BE 0, 1, 2"));
        }
      }
    }
    while (usb.available() > 0 || (index < 2 && exit == false))
    {
      if (usb.available() > 0)
      {
        byte entry = usb.read();
        if (entry >= 48 && entry <= 57) // 0 or 1 or 2
        {
          usb.print(entry - 48);
          resetBuff[index] = entry - 48;
          index++;
        }
        else if (entry == char(27))
        {
          clearusb();
          usb.println(space);
          usb.println(F("EXITING 'AT-OB' AUTO-RESET ENTRY PROCESS"));
          index = 2;
          exit = true;
        }
        else
        {
          //do nothing
        }
      }
    }
    if (exit == false)
    {
      obRstTime = byte((resetBuff[0] * 10) + (resetBuff[1]));
      usb.println(space);
      usb.print(F("AT-OB Auto-Reset Hour Entered: "));
      usb.println(obRstTime);
      usb.println(space);
      exit = true;
    }
  }
  saveAutoReset();
  usb.println(F("New AT-OB Auto-Reset Saved To Memory\r\n\r\n"));
  systemInfoSetup();
}

void pullCurrentData()
{
  eeReadArray(int(mnMxData) + 3, 9);
  minTemp = int(word(eeBuff[0], eeBuff[1]));
  minHr = eeBuff[2];
  minMin = eeBuff[3];
  maxTemp = int(word(eeBuff[4], eeBuff[5]));
  maxHr = eeBuff[6];
  maxMin = eeBuff[7];
  thisMonth = eeBuff[8];

  eeReadArray(int(obData), 8);
  minObTemp = int(word(eeBuff[0], eeBuff[1]));
  minOb_Hr = eeBuff[2];
  minOb_Min = eeBuff[3];
  maxObTemp = int(word(eeBuff[4], eeBuff[5]));
  maxOb_Hr = eeBuff[6];
  maxOb_Min = eeBuff[7];
  pullSiteID_Mem();
}

void sdInit()
{
  for (int x = 1; x < 13; x++)
  {
    myFile = SD.open(mmFiles[x], FILE_WRITE);
    myFile.close();
    delay(5);
    myFile = SD.open(obFiles[x], FILE_WRITE);
    myFile.close();
    delay(5);
    myFile = SD.open(hrFiles[x], FILE_WRITE);
    myFile.close();
    delay(5);
    myFile = SD.open(fiveMinFiles[x], FILE_WRITE);
    myFile.close();
    delay(5);
  }
}

void eeClearData(int add, int num)
{
  int eeAdd = add;
  for (int i = add; i < num; i++)
  {
    EEPROM.write(eeAdd, 0x00);
    eeAdd++;
  }
}

void sdClearData()
{
  for (int x = 1; x < 13; x++)
  {
    SD.remove(mmFiles[x]);
    SD.remove(obFiles[x]);
    SD.remove(hrFiles[x]);
    SD.remove(fiveMinFiles[x]);
  }
  sdInit();
}

void newSD_Month()
{
  SD.remove(mmFiles[int(month)]);
  SD.remove(obFiles[int(month)]);
  SD.remove(hrFiles[int(month)]);
  SD.remove(fiveMinFiles[int(month)]);
  myFile = SD.open(mmFiles[int(month)], FILE_WRITE);
  myFile.close();
  delay(5);
  myFile = SD.open(obFiles[int(month)], FILE_WRITE);
  myFile.close();
  delay(5);
  myFile = SD.open(hrFiles[int(month)], FILE_WRITE);
  myFile.close();
  delay(5);
  myFile = SD.open(fiveMinFiles[int(month)], FILE_WRITE);
  myFile.close();
  delay(5);
}

void wipeHistory()
{
  eeClearData(int(mnMxData), 4096);
  sdClearData();
}

void obSDRead_Internal(int fileIndex)
{
  File readFile = SD.open(obFiles[fileIndex], FILE_READ);
  if (readFile)
  {
    usb.println(F("File Start:"));
    // read from the file until there's nothing else in it:
    while (readFile.available())
    {
      usb.print(char(readFile.read()));
      delay(5);
    }
    // close the file:
    readFile.close();
    usb.println(F("File Stop:"));
    delay(50);
  }
  else {
    // if the file didn't open, print an error:
    usb.print(F("error opening: "));
    usb.println(obFiles[fileIndex]);
  }
}

void mmSDRead_Internal(int fileIndex)
{
  File readFile = SD.open(mmFiles[fileIndex], FILE_READ);
  if (readFile)
  {
    usb.println(F("File Start:"));
    // read from the file until there's nothing else in it:
    while (readFile.available())
    {
      usb.print(char(readFile.read()));
      delay(5);
    }
    // close the file:
    readFile.close();
    usb.println(F("File Stop:"));
    delay(50);
  }
  else {
    // if the file didn't open, print an error:
    usb.print(F("error opening: "));
    usb.println(mmFiles[fileIndex]);
  }
}

void hrSDRead_Internal(int fileIndex)
{
  File readFile = SD.open(hrFiles[fileIndex], FILE_READ);
  if (readFile)
  {
    usb.println(F("File Start:"));
    // read from the file until there's nothing else in it:
    while (readFile.available())
    {
      usb.print(char(readFile.read()));
      delay(5);
    }
    // close the file:
    readFile.close();
    usb.println(F("File Stop:"));
    delay(50);
  }
  else {
    // if the file didn't open, print an error:
    usb.print(F("error opening: "));
    usb.println(hrFiles[fileIndex]);
  }
}

void fiveMinSDRead_Internal(int fileIndex)
{
  File readFile = SD.open(fiveMinFiles[fileIndex], FILE_READ);
  if (readFile)
  {
    usb.println(F("File Start:"));
    // read from the file until there's nothing else in it:
    while (readFile.available())
    {
      usb.print(char(readFile.read()));
      delay(5);
    }
    // close the file:
    readFile.close();
    usb.println(F("File Stop:"));
    delay(50);
  }
  else {
    // if the file didn't open, print an error:
    usb.print(F("error opening: "));
    usb.println(fiveMinFiles[fileIndex]);
  }
}

void clearusb()
{
  while (usb.available() > 0)
  {
    byte inByte = usb.read();
  }
}

void clearnimbus()
{
  while (nimbus.available() > 0)
  {
    byte inByte = nimbus.read();
  }
}

void clearfpr()
{
  while (fpr.available() > 0)
  {
    byte inByte = fpr.read();
  }
}

void getSD_Data()
{
  boolean sdMenu = false;
  char sdBuff[7];
  int fileMonth = 0;
  byte index = 0;
  boolean selectType = false;
  boolean selectMonth = false;
  boolean exit = false;
  boolean singleDigitMonth = true;

  clearusb();
  usb.println(F("\nSELECT THE HISTORY FILE TYPE YOU WANT TO RECOVER\n"));
  usb.println(F("1. Observation History (At-Ob Readings)"));
  usb.println(F("2. Daily History (Midnight to Midnight)"));
  usb.println(F("3. Hourly Data"));
  usb.println(F("4. 5-Minute Readings"));
  usb.println(F("ESC = Main Menu\n\n"));
  while (exit == false)
  {
    while (selectType == false)
    {
      while (usb.available() == 0)
      {
        delay(200);
      }
      char selectChar = char(usb.read());

      if (selectChar >= '1' && selectChar <= '4')
      {
        sdBuff[index] = selectChar;
        index++;
        //usb.println(index);
        selectType = true;
      }
      else if (selectChar == char(27))
      {
        selectType = true;
        exit = true;
        menuReprint = true;
      }
      else if (selectChar == char(10)) //newline
      {
        //do nothing...
      }
      else
      {
        usb.println(F("Invalid response. Please select #1 through #4!"));
      }
    }

    while (selectMonth == false && exit == false)
    {
      boolean entryComplete = false;
      usb.println(F("\nSelect Month and press ENTER when done."));
      usb.println(F("1. Jan\t5. May\t 9. Sep"));
      usb.println(F("2. Feb\t6. Jun\t10. Oct"));
      usb.println(F("3. Mar\t7. Jul\t11. Nov"));
      usb.println(F("4. Apr\t8. Aug\t12. Dec"));
      usb.println(F("ESC = SD File Menu\n\n"));
      while (entryComplete == false && exit == false)
      {
        while (index == 1 && exit == false)
        {
          while (usb.available() == 0)
          {
            delay(200);
          }
          char selectChar = char(usb.read());
          if (selectChar >= '0' && selectChar <= '9')
          {
            sdBuff[index] = selectChar;
            index++;
          }
          else if (selectChar == char(10))
          {
            //do nothing...
          }
          else if (selectChar == char(27)) //Escape key
          {
            exit = true;
            sdMenu = true;
          }
          else
          {
            entryComplete = true;
            usb.println(F("Invalid response. Please select #1 through #12 and press ENTER!"));
          }
        }
        while (index == 2 && exit == false)
        {
          while (usb.available() == 0)
          {
            delay(200);
          }
          char selectChar = char(usb.read());
          if (selectChar >= '0' && selectChar <= '2') //limit 2 digit entries to 10-12.
          {
            sdBuff[index] = selectChar;
            index++;
            selectMonth = true;
            singleDigitMonth = false;
            usb.println(F("Press Enter"));
          }
          else if (selectChar == char(13))
          {
            entryComplete = true;
            selectMonth = true;
            index++;

          }
          else if (selectChar == char(10))
          {
            //do nothing...
          }
          else if (selectChar == char(27)) //Escape key
          {
            exit = true;
            sdMenu = true;
          }
          else
          {
            usb.println(F("Invalid response. Please select #1 through #12 and press ENTER!"));
            index = 1;
            entryComplete = true;
            selectMonth = false;
          }
        }
        if ((index == 2 || index == 3) && entryComplete == false)
        {
          while (usb.available() == 0)
          {
            delay(200);
          }
          byte inByte = usb.read();
          if (inByte == char(13))
          {
            entryComplete = true;
          }
          else if (inByte == char(27))
          {
            entryComplete = true;
            exit = true;
            sdMenu = true;
          }
        }
      }
    }
    if (exit == false)
    {
      clearusb();
      usb.println(F("\nFILE REQUEST IS ABOUT TO BE SENT."));
      usb.println(F("IF YOU WISH TO CAPTURE THE DATA ENSURE YOUR TERMINAL EMULATOR"));
      usb.println(F("IS SET UP TO CAPTURE OR LOG THE DOWNLOAD."));
      usb.println(F("\nPress 'ANY KEY' to continue."));
      if (singleDigitMonth)
      {
        fileMonth = int(sdBuff[1] - 48);
      }
      else
      {
        fileMonth = int(((sdBuff[1] - 48) * 10) + (sdBuff[2] - 48));
      }

      while (usb.available() == 0)
      {
        delay(200);
      }
      byte inByte = usb.read();//aknowledging "any" entry
      exit = true;
      if (sdBuff[0] == '1')
      {
        obSDRead_Internal(fileMonth);
      }
      else if (sdBuff[0] == '2')
      {
        mmSDRead_Internal(fileMonth);
      }
      else if (sdBuff[0] == '3')
      {
        hrSDRead_Internal(fileMonth);
      }
      else if (sdBuff[0] == '4')
      {
        fiveMinSDRead_Internal(fileMonth);
      }
      usb.println(F("\n"));
    }
  }
  if (sdMenu == true)
  {
    getSD_Data();
  }
}

void currentObValues()
{
  float tempLo_Ob = float(minObTemp) / 10;
  float tempHi_Ob = float(maxObTemp) / 10;
  usb.println(space);
  usb.println(F("Current Observation Day"));
  usb.print(loLabel);
  usb.print(tempLo_Ob, 1);
  usb.print(tab);
  if (minOb_Hr < 10)
  {
    usb.print(zero);
  }
  usb.print(minOb_Hr);
  usb.print(colon);
  if (minOb_Min < 10)
  {
    usb.print(zero);
  }
  usb.println(minOb_Min);
  usb.print(hiLabel);
  usb.print(tempHi_Ob, 1);
  usb.print(tab);
  if (maxOb_Hr < 10)
  {
    usb.print(zero);
  }
  usb.print(maxOb_Hr);
  usb.print(colon);
  if (maxOb_Min < 10)
  {
    usb.print(zero);
  }
  usb.println(maxOb_Min);
  usb.println(space);
}

void currentDailyValues()
{
  float tempTemp = float(temperature) / 10;
  float tempLo = float(minTemp) / 10;
  float tempHi = float(maxTemp) / 10;
  usb.println(space);
  usb.println(F("Current Daily Values (midnight to midnight)"));
  usb.print(tempLabel);
  usb.println(tempTemp, 1);
  usb.print(loLabel);
  usb.print(tempLo, 1);
  usb.print(tab);
  if (minHr < 10)
  {
    usb.print(zero);
  }
  usb.print(minHr);
  usb.print(colon);
  if (minMin < 10)
  {
    usb.print(zero);
  }
  usb.println(minMin);
  usb.print(hiLabel);
  usb.print(tempHi, 1);
  usb.print(tab);
  if (maxHr < 10)
  {
    usb.print(zero);
  }
  usb.print(maxHr);
  usb.print(colon);
  if (maxMin < 10)
  {
    usb.print(zero);
  }
  usb.println(maxMin);
  usb.println(space);
}


void unitInfo()
{
  usb.println(space);
  usb.println(F("National Weather Service"));
  usb.println(F("CCOOP"));
  usb.println(F("Software Version: v1.9"));
  usb.println(F("Dated: 10/02/18\n"));
}

void userDT_Update()
{
  byte dtBuff[6];
  byte dtComplete = 0;
  byte tmBuff[4];
  byte tmComplete = 0;
  byte index = 0;
  usb.print(F("\r\nCurrent Date: "));
  usb.print(month);
  usb.print(slashLabel);
  usb.print(dayOfMonth);
  usb.print(slashLabel);
  usb.println(year);

  usb.print(F("Enter Date in MMDDYY Format (no /'s): "));
  while (dtComplete == 0)
  {
    if (usb.available() > 0)
    {
      byte readByte = usb.read();
      byte inByte = readByte - 48;
      if ((inByte >= 0) && (inByte <= 9))
      {
        dtBuff[index] = inByte;
        usb.print(dtBuff[index]);
        index++;
      }
      else if (readByte == char(27)) //ESC Key
      {
        usb.println(F("\r\n\nDate / Time Entry Canceled!\r\n"));
        menuReprint = true;
        return;
      }
      else
      {
        index = 0;
        usb.println(F("\r\nInvalid Entry. Start Date Entry Over."));
        usb.print(F("Enter Date in MMDDYY Format: "));
      }
    }
    if (index == 6)
    {
      dtComplete = 1;
      index = 0;
      usb.println(F("\n\n"));
    }
  }

  month = ((dtBuff[0] * 10) + dtBuff[1]);
  dayOfMonth = ((dtBuff[2] * 10) + dtBuff[3]);
  year = ((dtBuff[4] * 10) + dtBuff[5]);
  usb.print(F("Current Time: "));
  if (hour < 10)
  {
    usb.print(zero);
  }
  usb.print(hour);
  if (minute < 10)
  {
    usb.print(zero);
  }
  usb.println(minute);
  usb.print(F("Enter Time in 24Hr Format (ex. 0725): "));
  while (tmComplete == 0)
  {
    if (usb.available() > 0)
    {
      byte readByte = usb.read();
      byte inByte = readByte - 48;
      if ((inByte >= 0) && (inByte <= 9))
      {
        tmBuff[index] = inByte;
        usb.print(tmBuff[index]);
        index++;
      }
      else if (readByte == char(27)) //ESC Key
      {
        usb.println(F("\r\n\nDate / Time Entry Canceled!\r\n"));
        menuReprint = true;
        return;
      }
      else
      {
        index = 0;
        usb.println(F("\r\nInvalid Entry. Start Time Entry Over."));
        usb.print(F("Enter Time in 24Hr Format (ex. 0725): "));
      }
    }
    if (index == 4)
    {
      tmComplete = 1;
      usb.println(F("\n\n"));
    }


  }

  hour = ((tmBuff[0] * 10) + tmBuff[1]);
  minute = ((tmBuff[2] * 10) + tmBuff[3]);
  delay(10);

  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write((byte)0x00);
  Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
  // bit 6 (also need to change readDateDs1307)
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(dayOfMonth));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
  delay(5);
  getDateTime();

  menuReprint = true;
}

void getD_UnitMenu()
{
  if (menuReprint)
  {
    //usb.println(F("C"));
  }
  usb.println(F("------------------------------------"));
  usb.println(F("*******Command Summary*******"));
  usb.println(F("1. Set System Parameters"));
  usb.println(F("2. Query Unit Information"));
  usb.println(F("3. Send Temperature Update Request"));
  usb.println(F("4. Current Observation Values"));
  usb.println(F("5. Current Daily Values"));
  usb.println(F("6. Clear Memory History"));
  usb.println(F("7. Clear SD Card History"));
  usb.println(F("8. Master Clear Device (both #6 and #7)"));
  usb.println(F("9. SD Card Data Recovery"));
  usb.println(F("R. Reset Current Hi/Lo Values"));
  usb.println(F("ESC = Release unit control"));
  usb.println(F("------------------------------------"));
  menuReprint = false;
}

void systemInfoSetup()
{
  clearusb();
  usb.println(space);
  usb.println(F("1. Set System ID#"));
  usb.println(F("2. Set Date / Time"));
  usb.println(F("3. Set AT-OB Auto Reset Hour"));
  usb.println(F("ESC = Main Menu"));
  while (usb.available() == 0)
  {
    delay(200);
  }
  char selectChar = char(usb.read());
  switch (selectChar)
  {
    case '1':
      setSystemID();
      delay(10);
      updateSysIDMem(byte(1));
      delay(10);
      pullSiteID_Mem();
      break;

    case '2':
      userDT_Update();
      break;

    case '3':
      userAutoReset_Update();
      break;

    case char(27):
      getD_UnitMenu();
      break;

    default:
      break;
  }
}

void setSystemID()
{
  char sysIDBuff[6];
  byte index = 0;
  boolean exit = false;
  usb.println(space);
  usb.print(F("Current Site ID: "));
  for (byte i = 0; i < 6; i++)
  {
    usb.print(siteID[i]);
    delay(2);
  }
  usb.println(F("\r\nENTER THE NEW 6 DIGIT SYSTEM ID#"));
  while (exit == false)
  {
    while (usb.available() == 0)
    {
      delay(200);
    }
    while (usb.available() > 0 || index < 6)
    {
      char entry = char(usb.read());
      if (entry >= '0' && entry <= '9')
      {
        usb.print(entry);
        sysIDBuff[index] = entry;
        index++;
        delay(10);
      }

      else if (entry == char(27))
      {
        clearusb();
        usb.println(space);
        usb.println(F("EXITING SYSTEM ID# ENTRY PROCESS"));
        index = 6;
        exit = true;
      }
      else
      {
        //DO NOTHING....
      }
    }
    if (exit == false)
    {
      usb.println(space);
      usb.print(F("System ID Entered: "));
      for (byte x = 0; x < 6; x++)
      {
        siteID[x] = sysIDBuff[x];
        usb.print(siteID[x]);
      }
      usb.println(space);
      exit = true;
    }
  }

}

void updateSysIDMem(byte c)
{
  usb.println(space);
  if (c == 1)
  {
    usb.print(F("Current Site ID: "));
    for (byte i = 0; i < 6; i++)
    {
      usb.print(siteID[i]);
      eeBuff[i] = siteID[i];
      delay(2);
    }
  }
  eeWriteArray(siteIDMem, 6);
  usb.print(F("\r\n\r\n"));
  if (c == 1)
  {
    systemInfoSetup();
  }
}

void eeReadArray(int add, int num)
{
  int eeAdd = add;
  for (int i = 0; i < num; i++)
  {
    eeBuff[i] = EEPROM.read(eeAdd);
    eeAdd++;
  }
}

void eeWriteArray(int add, int num)
{
  int eeAdd = add;
  for (int i = 0; i < num; i++)
  {
    EEPROM.write(eeAdd, eeBuff[i]);
    eeAdd++;
  }
}

void checkTimers_Rollover()
{
  if (millis() < tempCheckTimer)//account for millis() rollover
  {
    tempCheckTimer = millis();
  }
}

void gps_SetTime()//set time based on front panel input
{
  second = byte(0);
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write((byte)0x00);
  Wire.write(decToBcd(gps.seconds));    // 0 to bit 7 starts the clock
  Wire.write(decToBcd(gps.minute));
  Wire.write(decToBcd(gps.hour));      // If you want 12 hour am/pm you need to set
  // bit 6 (also need to change readDateDs1307)
  Wire.write(decToBcd(0));
  Wire.write(decToBcd(gps.day));
  Wire.write(decToBcd(gps.month));
  Wire.write(decToBcd(gps.year));
  Wire.endTransmission();
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  usb.print(":");
  if (digits < 10)
    usb.print('0');
  usb.print(digits);
}

float Thermistor(float volt)
{
  long Resistance;
  float Temp;  // Dual-Purpose variable to save space.

  Resistance = ((pad * v_ref / (v_ref - volt)) - pad);
  usb.print(F("\nResistance: "));
  usb.println(Resistance);
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.000969711 + (0.000232887 * Temp) + (0.0000000801420 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius
  // Uncomment this line for the function to return Fahrenheit instead.
  //temp = (Temp * 9.0) / 5.0 + 32.0;                 // Convert to Fahrenheit
  return Temp;                                      // Return the Temperature
}

void getThermReading()
{
  byte i;
  float tempAvg = 0.0;
  digitalWrite(vref_Pin, LOW);
  delay(500);
  for (i = 0; i < 8; i++)
  {
    temp = getADC();     // read ADC and  convert it to Celsius
    //delay(1000);
    if (i > 2)
    {
      tempAvg += temp;
    }
  }
  temp = tempAvg / 5;
  temp = Thermistor(temp);
  usb.print(temp, 1);
  usb.println(F(" C"));
  temp = (temp * 9.0) / 5.0 + 32.0; // converts to  Fahrenheit
  outChassisTemp = temp;
  usb.print(temp, 1);
  usb.println(F(" F"));
  digitalWrite(vref_Pin, HIGH);
  ~i;
}


float getADC()
{
  digitalWrite(LTC_CS, LOW);
  delayMicroseconds(1);
  if (digitalRead(LTC_MISO) == LOW) {   // ADC Converter ready ?
    ltw = 0;
    sig = 0;

    b0 = SPI_read();             // read 4 bytes adc raw data with SPI
    if ((b0 & 0x20) == 0) sig = 1; // is input negative ?
    b0 &= 0x1F;                  // discard bit 25..31
    ltw |= b0;
    ltw <<= 8;
    b0 = SPI_read();
    ltw |= b0;
    ltw <<= 8;
    b0 = SPI_read();
    ltw |= b0;
    ltw <<= 8;
    b0 = SPI_read();
    ltw |= b0;

    delayMicroseconds(1);

    digitalWrite(LTC_CS, HIGH);      // LTC2400 CS HIGH
    delay(200);

    if (sig) ltw |= 0xf0000000;    // if input negative insert sign bit
    ltw = ltw / 16;                // scale result down , last 4 bits have no information
    volt = ltw * v_ref / 16777216; // max scale

    //usb.print(cnt++);
    //usb.print(";  ");
  }
  digitalWrite(LTC_CS, HIGH); // LTC2400 CS hi
  //delay(20);
  //printFloat(volt, 8);
  //usb.println(F(""));
  return volt;
}

byte SPI_read()
{
  return SPI.transfer(0x00);
}

void checkESPSerial()
{
  //Serial.println(F("Checking"));
  //delay(1000);
  if (esp.available() > 0)
  {
    //Serial.println(F("Serial2 Info Received!"));
    while (esp.available() > 0)
    {
      char inChar = char(esp.read());
      if (inChar == '#')
      {
        doESPCommand();
      }
      else
      {
        Serial.print(inChar);
      }
      delay(10);
    }
  }
}
void doESPCommand()
{
  char commandChar = char(esp.read());
  switch (commandChar)
  {
    case 'T':
      if (timeUpdated == false && timeUpdate == false)
      {
        Serial.println(F("Processing Time Update"));
        processTimeUpdate();
      }
      else
      {
        while (esp.available() > 0)
        {
          byte c  = esp.read();
          Serial.print(char(c));
        }
      }
      break;
    default:
      break;
  }

}

void processTimeUpdate()
{
  String msgArray;
  char *strings[8];
  char *ptr = NULL;
  //String temp = "";
  boolean done = false;
  int index = 0;
  while (esp.available() > 0 && done == false)
  {
    char inChar = char(esp.read());
    if (inChar == '\n' || inChar == '\r')
    {
      done = true;
    }
    else
    {
      msgArray += inChar;
      delay(5);
    }
  }
  char arrays[msgArray.length() + 1];
  msgArray.toCharArray(arrays, msgArray.length() + 1);
  ptr = strtok(arrays, ",");  // takes a list of delimiters
  while (ptr != NULL)
  {
    strings[index] = ptr;
    index++;
    ptr = strtok(NULL, ",");  // takes a list of delimiters
  }
  //Serial.println(index);
  // print the tokens
  for (int n = 0; n < index; n++)
  {
    Serial.print(strings[n]);
    Serial.print(F("  "));
  }
  Serial.println(F("Hey!"));
  String tempM;
  tempM = String(strings[0]);
  delay(2);
  tempYear  = byte(tempM.toInt());
  delay(2);
  Serial.println(tempYear, DEC);
  tempM = String(strings[1]);
  delay(2);
  tempMonth  = byte(tempM.toInt());
  delay(2);
  Serial.println(tempMonth, DEC);
  tempM = String(strings[2]);
  delay(2);
  tempDay  = byte(tempM.toInt());
  delay(2);
  Serial.println(tempDay, DEC);
  tempM = String(strings[3]);
  delay(2);
  tempHour  = byte(tempM.toInt());
  delay(2);
  Serial.println(tempHour, DEC);
  tempM = String(strings[4]);
  delay(2);
  tempMinute  = byte(tempM.toInt());
  delay(2);
  Serial.println(tempMinute, DEC);
  tempM = String(strings[5]);
  delay(2);
  tempSecond  = byte(tempM.toInt());
  delay(2);
  Serial.println(tempSecond, DEC);
  unsigned long tempD = (60 - tempM.toInt()) * 1000UL;
  timeUpdateDelay = millis() + tempD;
  timeUpdate = true;
}
/*char inChar;
  if (esp.available() > 0)
  {
  while (esp.available() > 0 && (inChar != '\r' || inChar != '\n'))
  {
    inChar = char(esp.read());
    usb.print(inChar);
    delay(5);
  }
  delay(100);
  }
  }*/

void processReading()
{
  char d;
  while (fpr.available() > 0)
  {
    for (int x = 0; x < fpr.available(); x++)
    {
      d = char(fpr.read());
      if (d != '\r' && d != '\n')
      {
        readingString = readingString + String(d);
        delay(5);
      }
    }
    delay(100);
  }
  fprUpdate = true;
}

void bypass()
{
  if (usb.available() > 0)
  {
    while (usb.available() > 0)
    {
      byte b = usb.read();
      delay(5);
    }
  }
  if (fpr.available() > 0)
  {
    while (fpr.available() > 0)
    {
      byte c = fpr.read();
      delay(5);
    }
  }
  byte bypassMode = 1;
  while (bypassMode)
  {
    if (fpr.available() > 0)
    {
      while (fpr.available() > 0)
      {
        char a = char(fpr.read());
        usb.print(a);
      }
    }
    if (usb.available() > 0)
    {
      while (usb.available() > 0)
      {
        char d = char(usb.read());
        if (d == '_')
        {
          bypassMode = 0;
        }
        else
        {
          fpr.print(d);
        }
      }
    }
  }
}

void checkFPR()
{
  if (fpr.available() > 0) //2.5 minute temp pull
  {
    while (fpr.available() > 0)
    {
      //usb.println(F("Available!!"));
      byte c = fpr.read();
      //usb.println(c);
      if (c == 10)
      {
        //readingTimer = millis();
        delay(20);
        c = fpr.read();
        Serial.print(c);
        if (c == 35)
        {
          readingString = "#";
          delay(5);
          processReading();
        }
      }
    }
    if (fprUpdate == true)
    {
      String fprString = "#F";
      fprString = fprString + readingString;
      usb.print(F("Sent: "));
      usb.println(fprString);
      esp.println(fprString);
      fprUpdate = false;
    }
  }
}
void changeLED()
{
  digitalWrite(led_pin, !digitalRead(led_pin));
  ledTimer = millis();
}


