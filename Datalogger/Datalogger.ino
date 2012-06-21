/*
  SD card datalogger
 
 This example shows how to log data from three analog sensors 
 to an SD card using the SD library.
 	
 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 
 created  24 Nov 2010
 updated 2 Dec 2010
  by Tom Igoe
 
 This example code is in the public domain.	 
 */

#include <SD.h>
#include <Time.h>  
#include <avr/sleep.h>
#include <avr/wdt.h>

long timeSleep = 0;  // total time due to sleep
float calibv = 0.93; // ratio of real clock with WDT clock
volatile byte isrcalled = 0;  // WDT vector flag

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by unix time_t as ten ascii digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define SAMPLEDELAY 15000  // Delay before making a sample

int sample = 0;

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 10;
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  setSyncProvider( requestSync);  //set function to call when sync required
  Serial.println("Waiting for sync message");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  if(Serial.available() ) 
  {
    processSyncMessage();
  }
  if(timeStatus()!= timeNotSet) 
  {
    while(true)
    {
      // Sample
      String dataString = "";
      sample = analogRead(0);

      dataString += String(dayStr(weekday()));
      dataString += String(",");
      dataString += String(day());
      dataString += String(",");
      dataString += String(monthShortStr(month()));
      dataString += String(",");
      dataString += String(year());
      dataString += String(",");

      dataString += String(hour());
      dataString += String(":");
      dataString += String(minute());
      dataString += String(":");
      dataString += String(second());
      dataString += String(",");
      
      dataString += String(sample);
    
      // Write
      WriteValToCard(dataString);

      // Wait
      delay(SAMPLEDELAY);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void WriteValToCard(String val)
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) 
  {
    dataFile.println(val);
    dataFile.close();
    // print to the serial port too:
    Serial.println(val);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening datalog.txt");
  } 

}

/////////////////////////////////////////////////////////////////////////////////////////////////
void processSyncMessage() {
  // if time sync available from serial port, update time and return true
  while(Serial.available() >=  TIME_MSG_LEN ){  // time message consists of a header and ten ascii digits
    char c = Serial.read() ; 
    Serial.print(c);  
    if( c == TIME_HEADER ) {       
      time_t pctime = 0;
      for(int i=0; i < TIME_MSG_LEN -1; i++){   
        c = Serial.read();          
        if( c >= '0' && c <= '9'){   
          pctime = (10 * pctime) + (c - '0') ; // convert digits to a number    
        }
      }   
      setTime(pctime);   // Sync Arduino clock to the time received on the serial port
    }  
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Internal function: Start watchdog timer
// byte psVal - Prescale mask
void WDT_On (byte psVal)
{
  // prepare timed sequence first
  byte ps = (psVal | (1<<WDIE)) & ~(1<<WDE);
  cli();
  wdt_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = ps;
  sei();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Internal function.  Stop watchdog timer
void WDT_Off() {
  cli();
  wdt_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  sei();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Calibrate watchdog timer with millis() timer(timer0)
void calibrate() {
  // timer0 continues to run in idle sleep mode
  set_sleep_mode(SLEEP_MODE_IDLE);
  long tt1=millis();
  doSleep(256);
  long tt2=millis();
  calibv = 256.0/(tt2-tt1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Estimated millis is real clock + calibrated sleep time
long estMillis() {
  return millis()+timeSleep;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Delay function
void sleepCPU_delay(long sleepTime) {
  ADCSRA &= ~(1<<ADEN);  // adc off
  PRR = 0xEF; // modules off

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  int trem = doSleep(sleepTime*calibv);
  timeSleep += (sleepTime-trem);

  PRR = 0x00; //modules on
  ADCSRA |= (1<<ADEN);  // adc on
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// internal function.  
int doSleep(long timeRem) {
  byte WDTps = 9;  // WDT Prescaler value, 9 = 8192ms

  isrcalled = 0;
  sleep_enable();
  while(timeRem > 0) {
    //work out next prescale unit to use
    while ((0x10<<WDTps) > timeRem && WDTps > 0) {
      WDTps--;
    }
    // send prescaler mask to WDT_On
    WDT_On((WDTps & 0x08 ? (1<<WDP3) : 0x00) | (WDTps & 0x07));
    isrcalled=0;
    while (isrcalled==0) {
      // turn bod off
      MCUCR |= (1<<BODS) | (1<<BODSE);
      MCUCR &= ~(1<<BODSE);  // must be done right before sleep
      sleep_cpu();  // sleep here
    }
    // calculate remaining time
    timeRem -= (0x10<<WDTps);
  }
  sleep_disable();
  return timeRem;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// wdt int service routine
ISR(WDT_vect) {
  WDT_Off();
  isrcalled=1;
}



