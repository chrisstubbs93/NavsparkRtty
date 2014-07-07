#include "GNSS.h"
#include <util/crc16.h>

#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50    // Baud rate for use with RFM22B Max = 600
 
char datastring[80];
char txstring[80];
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
unsigned int count=0;

uint8_t gps_hour = 0;
uint8_t gps_min = 0;
uint8_t gps_sec = 0;
double lat = 0;
double lng = 0;
unsigned int alt = 0;
uint16_t gps_sats = 0;
uint16_t gln_sats = 0;

uint8_t gpio_bit = 0;
uint8_t led = 13;

void setup() {
GnssConf.init();
// put your setup code here, to run once:
gnss_gpio_set_output(gpio_bit);
pinMode(led, OUTPUT);
Serial.config(STGNSS_UART_8BITS_WORD_LENGTH, STGNSS_UART_1STOP_BITS, STGNSS_UART_NOPARITY);
Serial.begin();

Serial.print("Timer Status:");
uint8_t tmr0 = Timer0.every(20, tmr_task); 
Serial.println(tmr0);

}

void loop()
{
// put your main code here, to run repeatedly:
//sprintf(datastring,"$$$$$M0UPU,%04u,RTTY TEST BEACON RTTY TEST BEACON",count); // Puts the text in the datastring
}

void task_called_after_GNSS_update(void)
{
  GnssInfo.update();
  gps_hour = GnssInfo.time.hour(); 
  gps_min = GnssInfo.time.minute();
  gps_sec = GnssInfo.time.second();
    
  lat = GnssInfo.location.latitude();
  lng = GnssInfo.location.longitude();
  
  alt = GnssInfo.altitude.meters();
  
  gps_sats = GnssInfo.satellites.numGPSInUse(NULL); 
  gln_sats = GnssInfo.satellites.numGLNInUse(NULL); 
    
}

void tmr_task(void)
{
  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) {
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission, take a copy of the string so it doesn't change mid transmission.
    sprintf(datastring, "$$$$SPARK,%i,%02d%02d%02d,%.6f,%.6f,%i,%i,%i", count++, gps_hour, gps_min, gps_sec, lat, lng, alt, gps_sats, gln_sats);
    crccat(datastring + 4); //add checksum (lunars code)
    Serial.print(datastring);
    strcpy(txstring,datastring);
    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;
    break;
  case 2: // Grab a char and lets go transmit it.
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else
    {
      txstatus=0; // Should be finished
      txj=0;
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1);
      else rtty_txbit(0);  
      txc = txc >> 1;
      break;
    }
    else
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    }
  case 4:
    if(STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }
 
  }
}
  
  
  void rtty_txbit (int bit)
{
  if (bit)
  {
    digitalWrite(led, HIGH); // High
  }
  else
  {
    digitalWrite(led, LOW); // Low
  }
}

uint16_t crccat(char *msg)
{
  uint16_t x;
  for(x = 0xFFFF; *msg; msg++)
    x = crc_xmodem_update(x, *msg);
  snprintf(msg, 8, "*%04X\n", x);
  return(x);
}

uint16_t crc_xmodem_update (uint16_t crc, uint8_t data)
  {
      int i;
      crc = crc ^ ((uint16_t)data << 8);
      for (i=0; i<8; i++)
      {
          if (crc & 0x8000)
              crc = (crc << 1) ^ 0x1021;
          else
              crc <<= 1;
      }
      return crc;
  }
 
