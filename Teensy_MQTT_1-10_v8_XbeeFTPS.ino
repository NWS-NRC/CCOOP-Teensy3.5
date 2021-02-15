#include <SD.h>
#include <Adafruit_SleepyDog.h>
#include <MMTS.h>
#include <Wire.h>
#include <SPI.h>
#include <WProgram.h>
#include <EEPROM.h>
#include <math.h>
#undef int
#include <stdio.h>

#define LTC_CS 30
#define LTC_MISO  12
#define LTC_SCK  13

#define D111D 25
boolean dActive = false;
boolean dInfo_Sent = false;
int precipMon = 0;
int precipInterval = 15;
String rainFinal = "";
String rainBatt = "";

String aString = "";
String tString = "";
String dString = "";
String pString = "";
//
#define LTC_DELAY  5 // 5 usec delay for commands on the i2c bus

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
float rainD = 0.0;

int led_pin = 24;
int vref_Pin = 39;
int xbee_ResetPin = 22;
unsigned long ledTimer = 0UL;

File myFile;
byte sdReady = 0;

#define usb Serial
#define xbee Serial5  //xbee cellular is Serial 5
#define nimbus Serial2
#define fpr Serial3

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
unsigned long espResetTimer = 0UL;
byte tempYear, tempMonth, tempDay, tempHour, tempMinute, tempSecond;
byte lastTempTx, lastPrecipTx;
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
int tChk_Increment = 50;
boolean recheck_Temp = false;
boolean tempSent = false;
int temp_problems = 0;
boolean temp_issue_alert = false;

String readingString = "";
String missingPrecip = "M";
boolean no_Precip = true;
boolean no_d_Precip = true;
float current_Precip = -99.99;
boolean reading = false;
boolean fprUpdate = false;
unsigned long precip_Timer = 0UL;
boolean precip_Enabled = false;
int precip_problems = 0;
boolean precip_issue_alert = false;

byte d_Month = 0;
byte d_Day = 0;
byte d_Year = 0;
int minTemp = 2555;
byte minHr = 25;
byte minMin = 61;
int maxTemp = -999;
byte maxHr = 25;
byte maxMin = 61;
float daily_precipTotal = -99.99;
float daily_precipStart = 99.99;
boolean newDayP = false;
boolean endDayP = false;
byte d_Update_Hr = 25;
byte d_Update_Mn = 61;
int d_prevDayTemp = 0;
boolean d_DQ = true;

byte atob_Month = 0;
byte atob_Day = 0;
byte atob_Year = 0;
int minObTemp = 2555;
byte minOb_Hr = 25;
byte minOb_Min = 61;
int maxObTemp = -999;
byte maxOb_Hr = 25;
byte maxOb_Min = 61;
float atob_precipTotal = -99.99;
float atob_precipStart = 99.99;
boolean atOb_newDayP = false;
boolean atOb_endDayP = false;
byte atob_Update_Hr = 25;
byte atob_Update_Mn = 61;
int atob_prevDayTemp = 0;
boolean atob_DQ = true;

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

char* softwareFile[] = {
  "Null_Update.txt", "software.bin"
};

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
#define obData    0x000000C8 //start of current day observation values at address 450. (8 bytes needed)
#define obRst_T   0x0000000A //observation reset time value at address 10.


uint8_t eeBuff[32];

unsigned long tempCheckTimer = 0UL;

boolean xbee_boot = true;
unsigned long xbeeTimer = 0UL;
unsigned long xbee_ResetTimer = 0UL;
boolean xbee_Ready = 1;

unsigned long pFailure_Interval = 3000000UL; //50 minutes

//******reboot tennsy sequence*****//
#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART()(*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))


void setup()
{
  pinMode(LTC_CS, OUTPUT);
  pinMode(vref_Pin, OUTPUT);
  pinMode(xbee_ResetPin, OUTPUT);
  pinMode(D111D, INPUT_PULLUP);
  digitalWrite(vref_Pin, HIGH);
  digitalWrite(xbee_ResetPin, HIGH);
  digitalWrite(LTC_CS, HIGH);
  SPI.begin();
  Wire.begin(); //start up I2C comms
  usb.begin(115200); // start usb serial port
  delay(5);
  xbee.begin(9600); //start up MMTS comms on Serial 2
  delay(5);
  int dValue = digitalRead(D111D);
  if (dValue == HIGH)
  {
    dActive = false;
    fpr.begin(9600); //start up MMTS comms on Serial 2
  }
  else
  {
    dActive = true;
    fpr.begin(115200); //start up MMTS comms on Serial 2
  }
  delay(5);
  nimbus.begin(9600); //start up MMTS comms on Serial 2
  delay(5);
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

  usb.print(F("Checking for D111 Configuration...."));
  if (dActive == true)
  {
    usb.println(F("D111D is active configuration"));
  }
  else
  {
    usb.println(F("D111E is active configuration"));
  }

  //getThermReading(); // pull thermistor/internal chassis temp

  //See if unit was rebooted same day. If so, don't discard data, else current day data needs to be cleared.
  getDateTime(); // pull current date/time from RTC and plug into variables
  pullCurrent_d_Data();
  usb.println(maxTemp, DEC);
  usb.println(maxTemp, DEC);
  usb.println(daily_precipTotal, 2);
  usb.println(daily_precipStart, 2);
  pullCurrent_atob_Data();
  usb.println(F("Current vs. AtOb Stored Information:"));
  usb.print(F("Month: "));
  usb.print(month, DEC);
  usb.print(F("  "));
  usb.println(atob_Month, DEC);
  usb.print(F("Day: "));
  usb.print(dayOfMonth, DEC);
  usb.print(F("  "));
  usb.println(atob_Day, DEC);
  usb.print(F("Year: "));
  usb.print(year, DEC);
  usb.print(F("  "));
  usb.println(atob_Year, DEC);

  if (atob_Month == month)
  {
    if (((atob_Day == dayOfMonth) || (atob_Day == dayOfMonth - 1)))// && (hour < obRstTime)) //still within a current atob cycle
    {
      if ((atob_Update_Hr == obRstTime) && (atob_Update_Mn < 30))
      {
        atob_DQ = false; //all data should be good for calculations
        atOb_newDayP = true;
      }
      else
      {
        atob_DQ = true; //data needs office to QC. Add "Q" to precip on RR3.
        atOb_newDayP = false;
        atOb_endDayP = false;

        if (atob_precipTotal == -99.99)
        {
          reset_atob_Precip();
          atOb_newDayP = false;
          atOb_endDayP = false;
        }
      }
    }
    else
    {
      reset_atob_Data();
      if ((hour == obRstTime) && (minute <= 30))
      {
        atob_DQ = false;
        atOb_newDayP = false;
        atOb_endDayP = false;
      }
      else
      {
        atob_DQ = true;
        atOb_newDayP = false;
        atOb_endDayP = false;
      }
    }
  }
  else
  {
    reset_atob_Data();
    if ((hour == obRstTime) && (minute <= 30))
    {
      atob_DQ = false;
      atOb_newDayP = false;
      atOb_endDayP = false;
    }
    else
    {
      atob_DQ = true;
      atOb_newDayP = false;
      atOb_endDayP = false;
    }
  }
  if (d_Month == month)
  {
    if (d_Day == dayOfMonth)
    {
      //usb.println(F("1"));
      if ((d_Update_Hr == 0) && (d_Update_Mn <= 30))
      {
        //usb.println(F("2"));
        d_DQ = false; //all data should be good for calculations
        newDayP = false;
      }
      else
      {
        //usb.println(F("3"));
        d_DQ = true; //data needs office to QC. Add "Q" to precip on RR3.
        newDayP = false;
        endDayP = false;

        if (daily_precipTotal == -99.99)
        {
          //usb.println(F("4"));
          reset_d_Precip();
          newDayP = false;
          endDayP = false;
        }
      }
    }
    else
    {
      //usb.println(F("5"));
      reset_d_Data();
      if ((hour == 0) && (minute <= 30))
      {
        //usb.println(F("6"));
        d_DQ = false;
        newDayP = false;
        endDayP = false;
      }
      else
      {
        //usb.println(F("7"));
        d_DQ = true;
        newDayP = false;
        endDayP = false;
      }
    }
  }
  else
  {
    //usb.println(F("8"));
    reset_d_Data();
    if ((hour == 0) && (minute <= 30))
    {
      //usb.println(F("9"));
      d_DQ = false;
      newDayP = false;
      endDayP = false;
    }
    else
    {
      //usb.println(F("10"));
      d_DQ = true;
      newDayP = false;
      endDayP = false;
    }
  }
  if (atob_Month != month)
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
  delay(1000);
  usb.println(F("\r\nGathering Initial Temp Reading....."));
  getCurrentTemp();//get most current temp
  usb.println(F("Beginning Normal Operation....."));
  delay(100);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  ledTimer = millis();
  tempCheckTimer = millis();
  tempMillis = millis();
  lastTempTx = minute;
  lastPrecipTx = minute;
  Watchdog.enable(300000);
  while (xbee.available() > 0)
  {
    char c = char(xbee.read());
    delay(10);
  }
  xbeeTimer = millis();
  xbee_ResetTimer = millis();
}

/*************************************Main Program***********************************************/
void loop()
{
  Watchdog.reset();
  if ((millis() - xbee_ResetTimer) >= 86400000UL)
  {
    usb.print(F("Xbee Reset Millis: "));
    usb.println(millis());
    usb.print(F("Xbee Reset XBee ResetTimer: "));
    usb.println(xbee_ResetTimer);
    resetXbee();
  }
  if (xbee_Ready == 0 && ((millis() - xbeeTimer) > 10000))
  {
    xbee_Ready = 1;
  }
  if (xbee_Ready == 1)
  {
    if (tString != "")
    {
      xbee_Ready = 0;
      xbee.println(tString);
      usb.println(F("Sending temp!!"));
      usb.println(tString);
      tString = "";
      xbeeTimer = millis();
    }
    else if (pString != "")
    {
      xbee_Ready = 0;
      xbee.println(pString);
      usb.println(F("Sending precip!!"));
      usb.println(pString);
      pString = "";
      xbeeTimer = millis();
    }
    else if (dString != "")
    {
      xbee_Ready = 0;
      xbee.println(dString);
      usb.println(F("Sending daily!!"));
      usb.println(dString);
      dString = "";
      xbeeTimer = millis();
    }
    else if (aString != "")
    {
      xbee_Ready = 0;
      xbee.println(aString);
      usb.println(F("Sending alert!!"));
      usb.println(aString);
      aString = "";
      xbeeTimer = millis();
    }
    else
    {
      ;;
    }
  }

  checkTimers_Rollover();

  if (dActive == false) {
    checkFPR();
  }

  if (precip_Enabled)
  {
    if (((millis() - precip_Timer) >= pFailure_Interval) && (precip_issue_alert == false))
    {
      precip_issue_alert = true;
      precip_problems += 1;
    }
  }

  if (timeUpdated == false && timeUpdate == true && millis() > timeUpdateDelay)
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

  if (dActive == false) {
    checkFPR();
  }

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
  if ((hour == 0 && minute == 0) && dLogUpdated == 0)
  {
    logDaily();
    reset_d_Data();
    d_DQ = false;
    if (endDayP == true)
    {
      newDayP = true;
      endDayP = false;
    }
    else
    {
      newDayP == false;
      endDayP == false;
    }
    dLogUpdated = 1;
  }
  if ((hour == 0 && minute > 0) && dLogUpdated == 1)
  {
    dLogUpdated = 0;
    if (month != thisMonth)
    {
      newSD_Month(); //delete new month history files on SD card so that they are fresh for the month.
      thisMonth = month;
    }
  }

  if (hour == 0 && minute == 7 && timeUpdated == true)
  {
    timeUpdated = false;
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
    reset_atob_Data();
    atob_DQ = false;
    if (atOb_endDayP == true)
    {
      atOb_newDayP = true;
      atOb_endDayP = false;
    }
    else
    {
      atOb_newDayP == false;
      atOb_endDayP == false;
    }
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

  checkXbeeSerial();

  if (dActive == true && (lastPrecipTx != minute) && (minute % 15 == 0))
  {
    checkFPRD();
    lastPrecipTx = minute;
  }

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
  outChassisTemp_TX = (int(outChassisTemp * 10));
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
  usb.print(F("\nNew Day Precip: "));
  usb.println(atOb_newDayP);
  usb.print(F("End Day Precip: "));
  usb.println(atOb_endDayP);
  usb.print(F("At-Ob DQ: "));
  usb.println(atob_DQ);

  usb.println(F("*************************************************"));
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
    if (dActive == false) {
      checkFPR();
    }
    delay(1000);
    mmts.getCurrentTemp();
    if (dActive == false) {
      checkFPR();
    }
    tempCheck2 = atoi(currentTemp);
    usb.println(float(tempCheck2) / 10);
    if (tempCheck1 == tempCheck2)
    {
      tempCheck = tempCheck2;
      match = true;
    }
    if (match == true)
    {
      if (tempCheck != NULL && (tempCheck < 1500) && (tempCheck >= (-990)))
      {
        if (((lastTemp - 50) <= tempCheck) && ((lastTemp + 50) >= tempCheck) && recheck_Temp == false) // +/- 5 degree check (normal or 2nd valid reading after bad reading.)
        {
          temperature = tempCheck;
          lastTemp = temperature;
          valid = 1;
          tempCheckTimer = millis();
        }
        else if (((lastTemp - 50) <= tempCheck) && ((lastTemp + 50) >= tempCheck) && recheck_Temp == true) // +/- 5 degree check (1st valid reading after bad reading)
        {
          lastTemp = tempCheck;
          recheck_Temp = false;
          valid = 0;
        }
        else if (((lastTemp - tChk_Increment) <= tempCheck) && ((lastTemp + tChk_Increment) >= tempCheck) && recheck_Temp == true) // +/- varying degree check
        {
          lastTemp = tempCheck;
          valid = 0;
          tChk_Increment = 50; //reset the expanded +/- band
          tempCheckTimer = millis();
        }
        else
        {
          //add +/- 5 degrees to check band (starts at +/-10, then checks for +/-20......+/- additional 10 degrees till it captures reported temps.
          tChk_Increment = tChk_Increment + 50;
          recheck_Temp = true;
        }
      }
    }
  }
  else if (boot == 1)
  {
    delay(1000);
    mmts.getCurrentTemp();
    tempCheck1 = atoi(currentTemp);
    usb.println(float(tempCheck1) / 10);
    if (dActive == false) {
      checkFPR();
    }
    delay(1000);
    mmts.getCurrentTemp();
    tempCheck2 = atoi(currentTemp);
    usb.println(float(tempCheck2) / 10);
    if (dActive == false) {
      checkFPR();
    }
    if (tempCheck1 == tempCheck2)
    {
      tempCheck = tempCheck2;
    }
    if (tempCheck != NULL && tempCheck != 0 && tempCheck >= (-990) && tempCheck <= 1500)
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
      xbee.println(espString);
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
      if ((d_prevDayTemp != 0) && (temperature <= 1500) && (temperature >= -990))
      {
        if ((temperature < minTemp) && (temperature <= d_prevDayTemp))
        {
          minTemp = temperature;
          minHr = hour;
          minMin = minute;
          updateDailyVals = 1;
        }
        if ((temperature > maxTemp) && (temperature >= d_prevDayTemp))
        {
          maxTemp = temperature;
          maxHr = hour;
          maxMin = minute;
          updateDailyVals = 1;
        }
      }
      else if ((d_prevDayTemp == 0) && (temperature <= 1500) && (temperature >= -990))
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
      }
      else
      {
        ;;
      }
      if ((atob_prevDayTemp != 0) && (temperature <= 1500) && (temperature >= -990))
      {
        if ((temperature < minObTemp) && (temperature <= atob_prevDayTemp))
        {
          minObTemp = temperature;
          minOb_Hr = hour;
          minOb_Min = minute;
          updateDailyGlobalVals = 1;
        }
        if ((temperature > maxObTemp) && (temperature >= atob_prevDayTemp))
        {
          maxObTemp = temperature;
          maxOb_Hr = hour;
          maxOb_Min = minute;
          updateDailyGlobalVals = 1;
        }
      }
      else if ((atob_prevDayTemp == 0) && (temperature <= 1500) && (temperature >= -990))
      {
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
  tString = espString;
  //usb.println(espString);
}

void reset_d_Data()
{
  if (minTemp != 2555)
  {
    minTemp = temperature;
    minHr = hour;
    minMin = minute;
    maxTemp = temperature;
    maxHr = hour;
    maxMin = minute;
    d_prevDayTemp = temperature;
  }
  else
  {
    minTemp = 1500;
    minHr = 25;
    minMin = 61;
    maxTemp = -990;
    maxHr = 25;
    maxMin = 61;
    d_prevDayTemp = 0;
  }
  if (current_Precip != -99.99)
  {
    daily_precipTotal = 0.00;
    daily_precipStart = current_Precip;
    d_Update_Hr = hour;
    d_Update_Mn = minute;
    no_d_Precip = true;
  }
  else
  {
    daily_precipTotal = -99.99;
    daily_precipStart = 99.99;
    d_Update_Hr = 25;
    d_Update_Mn = 61;
    no_d_Precip = true;
  }
  dailyUpdate();
}

void reset_d_Precip()
{
  if (current_Precip != -99.99)
  {
    daily_precipTotal = 0.00;
    daily_precipStart = current_Precip;
    d_Update_Hr = hour;
    d_Update_Mn = minute;
    no_d_Precip = true;
  }
  else
  {
    daily_precipTotal = -99.99;
    daily_precipStart = 99.99;
    d_Update_Hr = 25;
    d_Update_Mn = 61;
    no_d_Precip = true;
  }
  dailyUpdate();
}

void reset_atob_Data()
{
  if (minObTemp != 2555)
  {
    minObTemp = temperature;
    minOb_Hr = hour;
    minOb_Min = minute;
    maxObTemp = temperature;
    maxOb_Hr = hour;
    maxOb_Min = minute;
    atob_prevDayTemp = temperature;
  }
  else
  {
    minObTemp = 1500;
    minOb_Hr = 25;
    minOb_Min = 61;
    maxObTemp = -990;
    maxOb_Hr = 25;
    maxOb_Min = 61;
    atob_prevDayTemp = 0;
  }
  if (current_Precip != -99.99)
  {
    atob_precipTotal = 0.00;
    atob_precipStart = current_Precip;
    atob_Update_Hr = hour;
    atob_Update_Mn = minute;
    no_Precip = true;
  }
  else
  {
    atob_precipTotal = -99.99;
    atob_precipStart = 99.99;
    atob_Update_Hr = 25;
    atob_Update_Mn = 61;
    no_Precip = true;
  }
  ob_Update();
}

void reset_atob_Precip()
{
  if (current_Precip != -99.99)
  {
    atob_precipTotal = 0.00;
    atob_precipStart = current_Precip;
    atob_Update_Hr = hour;
    atob_Update_Mn = minute;
    no_Precip = true;
  }
  else
  {
    atob_precipTotal = -99.99;
    atob_precipStart = 99.99;
    atob_Update_Hr = 25;
    atob_Update_Mn = 61;
    no_Precip = true;
  }
  ob_Update();
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
  eeBuff[11] = highByte(d_prevDayTemp);
  eeBuff[12] = lowByte(d_prevDayTemp);
  eeBuff[13] = thisMonth;
  eeBuff[14] = highByte(int(daily_precipTotal * 100));
  eeBuff[15] = lowByte(int(daily_precipTotal * 100));
  eeBuff[16] = highByte(int(daily_precipStart * 100));
  eeBuff[17] = lowByte(int(daily_precipStart * 100));
  eeBuff[18] = d_Update_Hr;
  eeBuff[19] = d_Update_Mn;
  eeWriteArray(int(mnMxData), 20);
  usb.println(F("Daily Values Updated"));
}

void pullCurrent_d_Data()
{
  eeReadArray(int(mnMxData), 20);
  d_Month = eeBuff[0];
  d_Day = eeBuff[1];
  d_Year = eeBuff[2];
  minTemp = int(word(eeBuff[3], eeBuff[4]));
  if ((minTemp >> 15) == 1) minTemp = minTemp - 65535;
  minHr = eeBuff[5];
  minMin = eeBuff[6];
  maxTemp = int(word(eeBuff[7], eeBuff[8]));
  if ((maxTemp >> 15) == 1) maxTemp = maxTemp - 65535;
  maxHr = eeBuff[9];
  maxMin = eeBuff[10];
  d_prevDayTemp = int(word(eeBuff[11], eeBuff[12]));
  if ((d_prevDayTemp >> 15) == 1) d_prevDayTemp = d_prevDayTemp - 65535;
  thisMonth = eeBuff[13];
  int tempPull = int(word(eeBuff[14], eeBuff[15]));
  if ((tempPull >> 15) == 1) tempPull = tempPull - 65535;
  daily_precipTotal = float(tempPull) / 100;
  tempPull = int(word(eeBuff[16], eeBuff[17]));
  if ((tempPull >> 15) == 1) tempPull = tempPull - 65535;
  daily_precipStart = float(tempPull) / 100;
  d_Update_Hr = eeBuff[18];
  d_Update_Hr = eeBuff[19];
}

void logDaily()
{
  usb.println(F("Daily Log Updated"));
  String dailyPrecip;
  if (no_d_Precip == true)
  {
    dailyPrecip = missingPrecip;
  }
  else if (d_DQ == true || newDayP == false || endDayP == false)
  {
    dailyPrecip = String(daily_precipTotal, 2);
    dailyPrecip += "Q";
  }
  else
  {
    dailyPrecip = String(daily_precipTotal, 2);
  }
  myFile = SD.open(mmFiles[int(month)], FILE_WRITE);
  if (myFile)
  {
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
    myFile.print(F(","));
    myFile.print(dailyPrecip);
    myFile.close();
  }
}

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
  eeBuff[0] = month;
  eeBuff[1] = dayOfMonth;
  eeBuff[2] = year;
  eeBuff[3] = highByte(minObTemp);
  eeBuff[4] = lowByte(minObTemp);
  eeBuff[5] = minOb_Hr;
  eeBuff[6] = minOb_Min;
  eeBuff[7] = highByte(maxObTemp);
  eeBuff[8] = lowByte(maxObTemp);
  eeBuff[9] = maxOb_Hr;
  eeBuff[10] = maxOb_Min;
  eeBuff[11] = highByte(atob_prevDayTemp);
  eeBuff[12] = lowByte(atob_prevDayTemp);
  eeBuff[13] = highByte(int(atob_precipTotal * 100));
  eeBuff[14] = lowByte(int(atob_precipTotal * 100));
  eeBuff[15] = highByte(int(atob_precipStart * 100));
  eeBuff[16] = lowByte(int(atob_precipStart * 100));
  eeBuff[17] = atob_Update_Hr;
  eeBuff[18] = atob_Update_Mn;
  eeWriteArray(int(obData), 19);
  usb.println(F("Global Values Updated"));
}

void pullCurrent_atob_Data()
{
  eeReadArray(int(obData), 19);
  atob_Month = eeBuff[0];
  atob_Day = eeBuff[1];
  atob_Year = eeBuff[2];
  minObTemp = int(word(eeBuff[3], eeBuff[4]));
  if ((minObTemp >> 15) == 1) minObTemp = minObTemp - 65535;
  minOb_Hr = eeBuff[5];
  minOb_Min = eeBuff[6];
  maxObTemp = int(word(eeBuff[7], eeBuff[8]));
  if ((maxObTemp >> 15) == 1) maxObTemp = maxObTemp - 65535;
  maxOb_Hr = eeBuff[9];
  maxOb_Min = eeBuff[10];
  atob_prevDayTemp = int(word(eeBuff[11], eeBuff[12]));
  if ((atob_prevDayTemp >> 15) == 1) atob_prevDayTemp = atob_prevDayTemp - 65535;
  int tempPull = int(word(eeBuff[13], eeBuff[14]));
  if ((tempPull >> 15) == 1) tempPull = tempPull - 65535;
  atob_precipTotal = float(tempPull) / 100;
  tempPull = int(word(eeBuff[15], eeBuff[16]));
  if ((tempPull >> 15) == 1) tempPull = tempPull - 65535;
  atob_precipStart = float(tempPull) / 100;
  atob_Update_Hr = eeBuff[17];
  atob_Update_Mn = eeBuff[18];
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
  String ppTotal = "";
  if (no_Precip == true)
  {
    ppTotal = missingPrecip;
  }
  else
  {
    if (atob_precipTotal <= 0.02)
    {
      atob_precipTotal = 0.00;
      ppTotal = String(atob_precipTotal, 2);
    }
    else
    {
      ppTotal = String(atob_precipTotal, 2);
    }
  }
  usb.print(F("Total Precip: "));
  usb.println(ppTotal);
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
  espString += ",";
  if (ppTotal != missingPrecip)
  {
    if (atob_DQ == true || atOb_newDayP == false || atOb_endDayP == false)
    {
      ppTotal = ppTotal + "Q";
    }
  }
  espString += ppTotal;
  dString = espString;
  //usb.println(espString);
  usb.println(F("Global Log Updated"));
  myFile = SD.open(obFiles[int(month)], FILE_WRITE);
  if (myFile)
  {
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
    myFile.print(F(","));
    myFile.print(ppTotal);
    myFile.close();
  }
}

void logHourly()
{
  usb.println(F("Hourly Log Updated"));
  myFile = SD.open(hrFiles[int(month)], FILE_WRITE);
  if (myFile)
  {
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

void logFM()
{
  myFile = SD.open(fiveMinFiles[int(month)], FILE_WRITE);
  if (myFile)
  {
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

void pullObRstTime_Mem()
{
  obRstTime = EEPROM.read(int(obRst_T));
  delay(5);
}

void saveAutoReset()
{
  EEPROM.write(int(obRst_T), obRstTime);
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
  usb.println(F("Memory Erased"));
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
  usb.println(F("SD Card Cleared"));
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

void clearXbee()
{
  while (xbee.available() > 0)
  {
    byte inByte = xbee.read();
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
  usb.println(F("CCOOP-XBee"));
  usb.println(F("Software Version: v0.9 w/FTPS support"));
  usb.println(F("Dated: 01/10/21\n"));
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
  if (precip_Enabled)
  {
    if (millis() < precip_Timer)
    {
      precip_Timer = millis();
    }
  }
  if (xbeeTimer > millis())
  {
    xbeeTimer = millis();
  }
  if (xbee_ResetTimer > millis())
  {
    xbee_ResetTimer = millis();
  }
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

void checkXbeeSerial()
{
  String inString = "";
  if (xbee.available() > 0)
  {
    xbee_ResetTimer = millis();
    byte inByte = xbee.read();
    switch (inByte)
    {
      case 35:
        usb.println(F("Command Recieved1"));
        while (xbee.available() == 0)
        {
          delay(1);
        }
        processHash();
        break;

      default:
        byte nextByte;
        while (xbee.available() > 0)
        {
          delay(2);
          byte inByte2 = xbee.read();
          if (inByte2 == 35)
          {
            usb.println(F("Command Recieved2"));
            processHash();
          }
          else
          {
            //usb.println(F("Command Recieved"));
            usb.print(char(inByte2));
          }
        }
    }
  }
}

void processHash()
{
  if (xbee.available() == 0)
  {
    delay(1);
  }
  byte inByte = xbee.read();
  switch (inByte)
  {
    case 84:
      usb.println(F("\r\nXBee---Start Time Command"));
      delay(2);
      doCommand(1);
      break;
    case 70:
      usb.println(F("\r\nXBee---File Transfer Command"));
      delay(2);
      doCommand(2);
      break;
    default:
      usb.println(char(inByte));
      break;
  }
}

void doCommand(int n)
{
  switch (n)
  {
    case 1:
      usb.println(F("Time Command Recieved"));
      processTimeUpdate();
      break;
    case 2:
      usb.println(F("File Transfer Command Recieved"));
      process_FileTransferRequest();
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
  while (xbee.available() > 0 && done == false)
  {
    char inChar = char(xbee.read());
    usb.print(inChar);
    if (inChar == '\r')
    {
      ;;
    }
    else if (inChar == '\n')
    {
      done = true;
    }
    else
    {
      msgArray += inChar;
      delay(1);
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
  tempM = String(strings[6]);
  delay(2);
  obRstTime = byte(tempM.toInt());
  saveAutoReset();
}

void process_FileTransferRequest()
{
  bool info_transfer = false;
  String file_Info = ""; //establish string to build file information from
  unsigned long file_timer = millis();
  while (xbee.available() == 0 && (millis() - file_timer < 10000UL))
  {
    delay(1);
  }
  if (xbee.available() > 0)
  {
    while (xbee.available() > 0)
    {
      char d  = char(xbee.read());
      file_Info += d;
      delay(2);
    }
    info_transfer = true;
  }
  else
  {
    ;; //nothing more to send
  }
  if (info_transfer == true)
  {
    delay(2);
    int file_Size = file_Info.toInt();
    usb.print(F("File Size: "));
    usb.println(file_Size);
    if (file_Size > 0)
    {
      receive_File(file_Size);
    }
  }
  else
  {
    ;; //no files to process
  }
}

void receive_File(int f_size)
{
  byte chksum = 0;
  byte bad_transfers = 0;
  int chunks = f_size / 200;
  int last_chunk = f_size % 200;
  int totalBytes = 0;
  bool file_complete = false;
  unsigned long file_timer = 0;
  if (SD.exists(softwareFile[1]))
  {
    SD.remove(softwareFile[1]);
    delay(2);
    myFile = SD.open(softwareFile[1], FILE_WRITE);
  }
  else
  {
    myFile = SD.open(softwareFile[1], FILE_WRITE);
  }
  while (xbee.available() > 0) //clear out serial buffer
  {
    byte y = xbee.read();
    delay(1);
  }
  if (myFile)
  {
    bool good_rx = true;
    for (int i = 0; i < chunks; i++)
    {
      file_timer = millis();
      if (bad_transfers < 5)
      {
        if (good_rx == true)
        {
          changeLED();
          xbee.println(F("!S"));
          Watchdog.reset();
          changeLED();
        }
        else
        {
          while (xbee.available() > 0)
          {
            byte d = xbee.read(); //bad read clear out buffer
          }
          i = i - 1;
          xbee.println(F("!R")); // resend last packet block
          Watchdog.reset();
        }
        while (xbee.available() < 200 && (millis() - file_timer < 10000UL))
        {
          delay(1);
        }
        if (xbee.available() == 200) // ready to read
        {
          byte chunk_data[200];
          byte cnt = 0;
          while (xbee.available() > 0)
          {
            byte c = xbee.read();
            chunk_data[cnt] = c;
            cnt++;
          }
          if (cnt == 200)
          {
            totalBytes += 200;
            usb.println();
            usb.println(F("200 Bytes Received"));
            usb.print(F("Total Bytes: "));
            usb.println(totalBytes);
            myFile.write(chunk_data, 200);
            good_rx = true; //send next chunk of data
          }
          else
          {
            //"Invalid transfer read"
            good_rx = false;
            bad_transfers += 1;
          }
        }
        else
        {
          usb.print(F("Bytes Ready: "));
          usb.println(xbee.available());
          usb.println(F("Invalid transfer: bytes sent was too short"));
          good_rx = false;
          bad_transfers += 1;
        }
      }
      else
      {
        usb.println(F("Exceeded number of file transfer tries!!"));
      }
    }
    while (file_complete == false && bad_transfers < 5)
    {
      file_timer = millis();
      if (good_rx == true)
      {
        changeLED();
        xbee.println(F("!S"));
        Watchdog.reset();
        changeLED();
      }
      else
      {
        xbee.println(F("!R"));
        Watchdog.reset();
      }
      while (xbee.available() < last_chunk && (millis() - file_timer < 10000UL))
      {
        delay(1);
      }
      if (xbee.available() == last_chunk)
      {
        byte l_chunk_data[last_chunk];
        cnt = 0;
        while (xbee.available() > 0)
        {
          byte e = xbee.read();
          l_chunk_data[cnt] = e;
          cnt++;
        }
        if (cnt == last_chunk)
        {
          totalBytes += cnt;
          usb.println(F("\r\nFinal Transfer Received"));
          usb.print(F("Total Bytes: "));
          usb.println(totalBytes);
          good_rx = true;
          myFile.write(l_chunk_data, last_chunk);
          myFile.close();
          xbee.println(F("!X"));//cancel transfer session
          file_complete = true;
        }
        else
        {
          usb.println(F("Invalid transfer"));
          good_rx = false;
          bad_transfers += 1;
        }
      }
      else
      {
        while (xbee.available() > 0)
        {
          byte d = xbee.read(); //bad read clear out buffer
        }
        usb.println(F("Invalid transfer: bytes sent did not match size!"));
        good_rx = false;
        bad_transfers += 1;
      }
    }
    if (file_complete == false)
    {
      xbee.println(F("!X"));
      usb.println(F("Exceeded number of file transfer tries!!"));
      myFile.close();
      SD.remove(softwareFile[1]);
    }
    else
    {
      usb.println(F("File Transfer Success!!"));
      usb.println(F("Rebooting in 5 seconds to institute changes!"));
      delay(5000);
      teensy_reboot();      
    }    
  }
  else
  {
    usb.println(F("Could not establish file on SD Card!!"));
  }
}

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

void checkFPR()
{
  if (fpr.available() > 0) //15 min precip pull
  {
    while (fpr.available() > 0)
    {
      //usb.println(F("Available!!"));
      byte c = fpr.read();
      usb.println(c);
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
          if (precip_Enabled == false)
          {
            precip_Enabled = true;
          }
          precip_Timer = millis();
          if (precip_issue_alert == true)
          {
            precip_issue_alert = false;
            if (precip_problems > 0)
            {
              precip_problems -= 1;
            }
          }
        }
      }
    }
    if (fprUpdate == true)
    {
      String fprString = "#F";
      fprString = fprString + readingString;
      usb.print(F("Sent: "));
      usb.println(fprString);
      pString = fprString;
      lastPrecipTx = minute;
      fprUpdate = false;
      int comma_Cnt = 0;
      int pLen_Cnt = 0;
      String tempP = "";
      boolean valid_P = false;
      int pStart_Marker = 0;

      for (int x = 0; x < readingString.length(); x++)
      {
        if (comma_Cnt == 2)
        {
          pLen_Cnt++;
        }
        if (readingString[x] == ',')
        {
          comma_Cnt++;
        }
        if (comma_Cnt == 2 && pStart_Marker == 0)
        {
          pStart_Marker = x;
        }
        if (comma_Cnt == 3)
        {
          x = readingString.length();
          pLen_Cnt--;
        }
      }
      usb.print(F("Precip Length: "));
      usb.println(pLen_Cnt);
      usb.print(F("pStart_Marker: "));
      usb.println(pStart_Marker);

      if (pLen_Cnt == 4 && readingString[pStart_Marker + 2] == '.')
      {
        valid_P = true;
        tempP = String(readingString[pStart_Marker + 1]) + String(readingString[pStart_Marker + 2]) + String(readingString[pStart_Marker + 3]) + String(readingString[pStart_Marker + 4]);
      }
      if (pLen_Cnt == 5 && readingString[pStart_Marker + 3] == '.')
      {
        valid_P = true;
        tempP = String(readingString[pStart_Marker + 1]) + String(readingString[pStart_Marker + 2]) + String(readingString[pStart_Marker + 3]) + String(readingString[pStart_Marker + 4]) + String(readingString[pStart_Marker + 5]);
      }

      if (valid_P == false)
      {
        usb.println(F("Invalid precip reading"));
      }
      else
      {
        no_Precip = false;
        no_d_Precip = false;
        getDateTime();
        if (hour == obRstTime && minute <= 30)
        {
          atOb_newDayP = true;
        }
        if (hour == 0 && minute <= 30)
        {
          newDayP = true;
        }
        if (hour == 23 && minute <= 59 && minute >= 30)
        {
          endDayP = true;
        }
        if (obRstTime == 0)
        {
          if (hour == 23 && minute <= 59 && minute >= 30)
          {
            atOb_endDayP = true;
          }
        }
        else if (hour == obRstTime - 1 && minute <= 59 && minute >= 30)
        {
          atOb_endDayP = true;
        }
        else
        {
          ;;
        }
        current_Precip = tempP.toFloat();
        usb.print(F("Precip Rcvd: "));
        usb.println(current_Precip, 2);
        if (atob_precipStart == 99.99)
        {
          atob_precipStart = current_Precip;
          atob_Update_Hr = hour;
          atob_Update_Mn = minute;
          atob_precipTotal = 0.00;
          if (hour == obRstTime && minute <= 30)
          {
            atob_DQ = false;
          }
          else
          {
            atob_DQ = true;
          }
        }
        else
        {
          if ((atob_precipStart - current_Precip) >= 2.0)
          {
            //usb.println(F("Maint conducted. No at-ob calculations processed. Precip start value reset."));
            atob_precipStart = current_Precip - atob_precipTotal;
          }
          else
          {
            //usb.println(F("Not Maint"));
            atob_precipTotal = current_Precip - atob_precipStart;
          }
        }
      }
      if (daily_precipStart == 99.99)
      {
        daily_precipStart = current_Precip;
        d_Update_Hr = hour;
        d_Update_Mn = minute;
        daily_precipTotal = 0.00;
        if (hour == 0 && minute <= 30)
        {
          d_DQ = false;
        }
        else
        {
          d_DQ = true;
        }
      }
      else
      {
        if ((daily_precipStart - current_Precip) >= 2.0)
        {
          //usb.println(F("Maint conducted. No daily calculations processed. Precip start value reset."));
          daily_precipStart = current_Precip - daily_precipTotal;
        }
        else
        {
          daily_precipTotal = current_Precip - daily_precipStart;
        }
      }
      usb.print(F("at-ob precip: "));
      usb.println(atob_precipTotal, 2);
      usb.print(F("daily precip: "));
      usb.println(daily_precipTotal, 2);
      usb.print(F("\ncurrent precip: "));
      usb.println(current_Precip, 2);
      ob_Update();
      dailyUpdate();
    }
  }
}
void checkFPRD()
{
  char fprReading[22];
  char fprBatt[20];
  char fprDT[20];
  byte index = 0;
  String tempDT = "";
  fpr.write(byte(13));
  delay(1000);
  byte ready = 0;
  unsigned long waitTimer = millis(); //create a timer instance to allow the MMTS enough time to respond.
  while (fpr.available() == 0 && millis() - waitTimer < 10000UL)
  {
    delay(2); //if no data available for up to 10 second.....just wait.
  }
  waitTimer = millis();
  if (fpr.available() > 0)
  {
    while (fpr.available() > 0 && ready == 0 && millis() - waitTimer < 10000UL)
    {
      char inchar = char(fpr.read());
      usb.print(inchar);
      delay(1000);
      if (inchar == '>')
      {
        ready = 1;
        if (precip_Enabled == false)
        {
          precip_Enabled = true;
        }
        precip_Timer = millis();
        if (precip_issue_alert == true)
        {
          precip_issue_alert = false;
          if (precip_problems > 0)
          {
            precip_problems -= 1;
          }
        }
        usb.println(F("Ready!!"));
      }
      else
      {
        usb.print(F(".."));
      }
    }
    byte promptCheck = 0;
    // send "MEAS" w/Carriage Return. Delay needed because the MMTS board needs the slower command input.
    ready = 0;
    usb.println(F("TIME"));
    fpr.println(F("TIME"));
    usb.println();

    waitTimer = millis(); //create a timer instance to allow the MMTS enough time to respond.
    while (fpr.available() == 0 && millis() - waitTimer < 10000UL)
    {
      delay(2); //if no data available for up to 10 second.....just wait.
    }
    for (byte x = 0; x < 22; x++) // empty out current precip array
    {
      fprDT[x] = ' ';
    }
    waitTimer = millis(); //reset the timer
    promptCheck = 0;
    while (millis() - waitTimer < 2000UL)
    {
      while (fpr.available() > 0 && promptCheck == 0) // as long as there is data waiting in the receive buffer for processing....
      {
        byte inByte = fpr.read();
        // if received data is a number or a "-" symbol add it to the array holding temp.
        if (char(inByte) == '/' || char(inByte) == ':' || (((inByte - 48) >= 0) && ((inByte - 48) <= 9)))
        {
          //usb.print(char(inByte));
          fprDT[index] = char(inByte); // increase day in the array
          index++; // increment position in the temperature array for next byte of data
          if (index == 10)
          {
            fprDT[index] = ',';
            index++;
          }
          //Serial.print(F("valid char: "));
          //Serial.println(char(inByte));
        }
        else if (char(inByte) == '>')
        {
          //tempCnt = index;
          promptCheck = 1;
          usb.println(F(""));
        }
        else
        {
          ;;//do nothing with the other data...spaces, ".", CR, LF, etc...
        }
        delay(25); //give time for bytes to be received
      }

      while (fpr.available() > 0)
      {
        byte inbyte = fpr.read();
        delay(5);
      }
      fprDT[17] = '0';
      fprDT[18] = '0';
      tempDT = "";
      for (byte x = 5; x < 10; x++)
      {
        tempDT = tempDT + String(fprDT[x]);
      }
      tempDT = tempDT + String(fprDT[4]);
      tempDT = tempDT + "20";//add 20 to year for 2019 style reading
      for (byte x = 2; x < 4; x++)
      {
        tempDT = tempDT + String(fprDT[x]);
      }
      for (byte x = 10; x < 19; x++)
      {
        tempDT = tempDT + String(fprDT[x]);
      }
    }
    usb.println(tempDT);

    // send "MEAS" w/Carriage Return. Delay needed because the MMTS board needs the slower command input.
    ready = 0;
    usb.println();
    fpr.println(F("!LAST"));
    usb.println();
    index = 0;
    waitTimer = millis(); //create a timer instance to allow the MMTS enough time to respond.
    while (fpr.available() == 0 && millis() - waitTimer < 10000UL)
    {
      delay(2); //if no data available for up to 10 second.....just wait.
    }
    for (byte x = 0; x < 20; x++) // empty out current precip array
    {
      fprReading[x] = ' ';
    }
    waitTimer = millis(); //reset the timer
    promptCheck = 0;
    while (millis() - waitTimer < 2000UL)
    {
      while (fpr.available() > 0 && promptCheck == 0) // as long as there is data waiting in the receive buffer for processing....
      {
        byte inByte = fpr.read();

        // if received data is a number or a "-" symbol add it to the array holding temp.
        if (char(inByte) == '-' || char(inByte) == '.' || (((inByte - 48) >= 0) && ((inByte - 48) <= 9)))
        {
          usb.print(char(inByte));
          fprReading[index] = char(inByte); // increase day in the array
          index++; // increment position in the temperature array for next byte of data
          //Serial.print(F("valid char: "));
          //Serial.println(char(inByte));
        }
        else if (char(inByte) == '>' || char(inByte) == ',')
        {
          //tempCnt = index;
          promptCheck = 1;
          usb.println(F(""));
        }
        else
        {
          //do nothing with the other data...spaces, ".", CR, LF, etc...
        }
        delay(25); //give time for bytes to be received
      }

      while (fpr.available() > 0)
      {
        byte inbyte = fpr.read();
        delay(5);
      }
      rainFinal = "";
      for (byte x = 0; x < index; x++)
      {
        rainFinal = rainFinal + String(fprReading[x]);
      }
      if (rainFinal == "" || rainFinal == " ")
      {
        rainFinal = "M";
      }
    }
    //****
    usb.println();
    fpr.println(F("!BATT"));
    usb.println();

    waitTimer = millis(); //create a timer instance to allow the MMTS enough time to respond.
    while (fpr.available() == 0 && millis() - waitTimer < 10000UL)
    {
      delay(2); //if no data available for up to 10 second.....just wait.
    }
    for (byte x = 0; x < 20; x++) // empty out current precip array
    {
      fprBatt[x] = ' ';
    }
    waitTimer = millis(); //reset the timer
    promptCheck = 0;
    while (millis() - waitTimer < 2000UL)
    {
      while (fpr.available() > 0 && promptCheck == 0) // as long as there is data waiting in the receive buffer for processing....
      {
        byte inByte = fpr.read();
        usb.print(char(inByte));
        // if received data is a number or a "-" symbol add it to the array holding temp.
        if (char(inByte) == '-' || char(inByte) == '.' || (((inByte - 48) >= 0) && ((inByte - 48) <= 9)))
        {
          fprBatt[index] = char(inByte); // increase day in the array
          index++; // increment position in the temperature array for next byte of data
          //Serial.print(F("valid char: "));
          //Serial.println(char(inByte));
        }
        else if (char(inByte) == '>' || char(inByte) == ',')
        {
          //tempCnt = index;
          promptCheck = 1;
          usb.println(F(""));
        }
        else
        {
          ;;//do nothing with the other data...spaces, ".", CR, LF, etc...
        }
        delay(100); //give time for bytes to be received
      }

      while (fpr.available() > 0)
      {
        byte inbyte = fpr.read();
        delay(5);
      }
      rainBatt = "";
      for (byte x = 0; x < index; x++)
      {
        rainBatt = rainBatt + String(fprBatt[x]);
      }
    }
    if (rainFinal == "M")
    {
      usb.print(F("No Precip Response."));
    }
    else
    {
      no_Precip = false;
      no_d_Precip = false;
      getDateTime();
      if (hour == obRstTime && minute <= 30)
      {
        atOb_newDayP = true;
      }
      if (hour == 0 && minute <= 30)
      {
        newDayP = true;
      }
      if (hour == 23 && minute <= 59 && minute >= 30)
      {
        endDayP = true;
      }
      if (obRstTime == 0)
      {
        if (hour == 23 && minute <= 59 && minute >= 30)
        {
          atOb_endDayP = true;
        }
      }
      else if (hour == obRstTime - 1 && minute <= 59 && minute >= 30)
      {
        atOb_endDayP = true;
      }
      else
      {
        ;;
      }
      float temp_Precip = rainFinal.toFloat();
      usb.print(F("Precip Rcvd: "));
      usb.println(temp_Precip, 2);
      if (atob_precipStart == 99.99)
      {
        atob_precipStart = temp_Precip;
        current_Precip = temp_Precip;
        atob_Update_Hr = hour;
        atob_Update_Mn = minute;
        atob_precipTotal = 0.00;
        if (hour == obRstTime && minute <= 30)
        {
          atob_DQ = false;
        }
        else
        {
          atob_DQ = true;
        }
      }
      else
      {
        if ((temp_Precip - current_Precip) > 0.02)
        {
          current_Precip = temp_Precip;
        }
        if ((atob_precipStart - current_Precip) >= 2.0)
        {
          usb.println(F("Maint conducted. No at-ob calculations processed. Precip start value reset."));
          atob_precipStart = current_Precip - atob_precipTotal;
        }
        else
        {
          usb.println(F("Not Maint"));
          atob_precipTotal = current_Precip - atob_precipStart;
        }
      }
      if (daily_precipStart == 99.99)
      {
        daily_precipStart = current_Precip;
        d_Update_Hr = hour;
        d_Update_Mn = minute;
        daily_precipTotal = 0.00;
        if (hour == 0 && minute <= 30)
        {
          d_DQ = false;
        }
        else
        {
          d_DQ = true;
        }
      }
      else
      {
        if ((daily_precipStart - current_Precip) >= 2.0)
        {
          usb.println(F("Maint conducted. No daily calculations processed. Precip start value reset."));
          daily_precipStart = current_Precip - daily_precipTotal;
        }
        else
        {
          daily_precipTotal = current_Precip - daily_precipStart;
        }
      }
      usb.print(F("at-ob precip: "));
      usb.println(atob_precipTotal, 2);
      usb.print(F("daily precip: "));
      usb.println(daily_precipTotal, 2);
      usb.print(F("\ncurrent precip: "));
      usb.println(current_Precip, 2);
      ob_Update();
      dailyUpdate();
    }
    if (rainBatt == "" || rainBatt == " ")
    {
      rainBatt = "M";
    }
    else
    {
      float rainB = rainBatt.toFloat();
      rainBatt  = "";
      rainBatt  = String(rainB, 1);
    }
    usb.print(F("FPR-D: "));
    usb.print(rainFinal);
    usb.print(F(", "));
    usb.print(rainBatt);
    usb.println();
    fpr.println(F("!BYE"));
    usb.println();
    unsigned long waitTimer2 = millis(); //create a timer instance to allow the MMTS enough time to respond.
    while (fpr.available() == 0 && millis() - waitTimer2 < 10000UL)
    {
      delay(2); //if no data available for up to 10 second.....just wait.
    }
    while (fpr.available() > 0)
    {
      byte inByte = fpr.read();
      usb.print(char(inByte));
      delay(5);
    }
    String fprString = "#FPrecip,";
    String tempM = tempDT;
    fprString = fprString + tempM + "," + rainFinal + "," + rainBatt;
    usb.println(fprString);
    pString = fprString;
    dInfo_Sent = true;
  }
  else
  {
    if (precip_issue_alert == false)
    {
      precip_issue_alert = true;
    }
    precip_problems += 1;
  }
}

void changeLED()
{
  digitalWrite(led_pin, !digitalRead(led_pin));
  ledTimer = millis();
}

void resetXbee()
{
  usb.print(F("Resetting XBee....."));
  digitalWrite(xbee_ResetPin, LOW);
  delay(1000);
  digitalWrite(xbee_ResetPin, HIGH);
  xbee_ResetTimer = millis();
  usb.println(F("Complete!"));
}

void teensy_reboot()
{
  WRITE_RESTART(0x5FA0004);
}
