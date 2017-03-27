#include "Arduino.h"
#include "SoftwareSerial.h"

#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control

// how many times remain to sleep before wake up
int nbr_remaining;

SoftwareSerial mySerial(10, 11); 

ISR(WDT_vect)
{
        // not hanging, just waiting
        // reset the watchdog
        wdt_reset();
}

// function to configure the watchdog: let it sleep 8 seconds before firing
// when firing, configure it for resuming program execution
void configure_wdt(void)
{
 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds

  sei();                           // re-enable interrupts

  // reminder of the definitions for the time before firing
  // delay interval patterns:
  //  16 ms:     0b000000
  //  500 ms:    0b000101
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001
 
}

// Put the Arduino to deep sleep. Only an interrupt can wake it up.
void sleep(int ncycles)
{  
  nbr_remaining = ncycles; // defines how many cycles should sleep

  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  // Turn off the ADC while asleep.
  power_adc_disable();
 
  while (nbr_remaining > 0){ // while some cycles left, sleep!

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point if the
  // watchdog is configured for resume rather than restart
 
  // When awake, disable sleep mode
  sleep_disable();
  
  // we have slept one time more
  nbr_remaining = nbr_remaining - 1;
 
  }
 
  // put everything on again
  power_all_enable();
 
}

const unsigned char dofly_DuanMa[10]={
  0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
//位码
unsigned char const dofly_WeiMa[]={
  0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f};
long previousMillis = 0; 
const int analogInPin = A0;
int latch = 8;
int srclk = 9;
int ser = 10;

unsigned char displayTemp[8];
unsigned int Pm25 = 0;
unsigned int Pm10 = 0;

//把数据用串行转并行输出到595
void SendByte(unsigned char dat)
{    
  static unsigned char i; 
        
   for(i=0;i<8;i++)
    {
     digitalWrite(srclk,0);
     digitalWrite(ser,bitRead(dat,7-i));//这里也可以用arduino自带的函数操作，主用于595类型的输出，自查帮助文件。
     digitalWrite(srclk,1);
     }
}
//德飞莱的模块是2个595级联，段码和位码都是用折3根线控制，所以需要连续2个字节
void Send2Byte(unsigned char dat1,unsigned char dat2)
{    
   SendByte(dat1);
   SendByte(dat2);      
}
//锁存输出
void Out595(void)
{
        digitalWrite(latch,1);
        digitalWrite(latch,0);
}
void DisPm25data(unsigned int value)
{
  int i;
  if(value/1000)
  {
    displayTemp[0]=dofly_DuanMa[value/1000%10];//这里最后是加小数点，共阳和共阴是相反的。
    displayTemp[1]=dofly_DuanMa[value/100%10];
    displayTemp[2]=dofly_DuanMa[value/10%10];
  }
  else
  {
    if((value/100) != 0)
    displayTemp[0]=dofly_DuanMa[value/100%10];//这里最后是加小数点，共阳和共阴是相反的。
    displayTemp[1]=dofly_DuanMa[value/10%10]|0x80;
    displayTemp[2]=dofly_DuanMa[value%10];    
  }
  for(i=0;i<8;i++)
  {
    Send2Byte(dofly_WeiMa[i],displayTemp[i]);
    Out595();
  }
}
void DisPm10data(unsigned int value)
{
  int i;
  if(value/1000)
  {
    displayTemp[4]=dofly_DuanMa[value/1000%10];//这里最后是加小数点，共阳和共阴是相反的。
    displayTemp[5]=dofly_DuanMa[value/100%10];
    displayTemp[6]=dofly_DuanMa[value/10%10];
  }
  else
  {
    if((value/100) != 0)
    displayTemp[4]=dofly_DuanMa[value/100%10];//这里最后是加小数点，共阳和共阴是相反的。
    displayTemp[5]=dofly_DuanMa[value/10%10]|0x80;
    displayTemp[6]=dofly_DuanMa[value%10];    
  }
  for(i=0;i<8;i++)
  {
    Send2Byte(dofly_WeiMa[i],displayTemp[i]);
    Out595();
  }
}
void Display()
{
  //int i;
  //for(i=0;i<8;i++)
   // displayTemp[i]=0; 
     //DisPm25data(Pm25);
     //DisPm10data(Pm10);
     Serial.print("PM25 : ");
     Serial.print(Pm25);
     Serial.print("       PM10 : ");
     Serial.println(Pm10);
}
void ProcessSerialData()
{
  uint8_t mData = 0;
  uint8_t i = 0;
  uint8_t mPkt[10] = {0};
  uint8_t mCheck = 0;
 while (mySerial.available() > 0) 
  {  
    // from www.inovafitness.com
    // packet format: AA C0 PM25_Low PM25_High PM10_Low PM10_High 0 0 CRC AB
     mData = mySerial.read();     delay(2);//wait until packet is received
    if(mData == 0xAA)//head1 ok
     {
        mPkt[0] =  mData;
        mData = mySerial.read();
        if(mData == 0xc0)//head2 ok
        {
          mPkt[1] =  mData;
          mCheck = 0;
          for(i=0;i < 6;i++)//data recv and crc calc
          {
             mPkt[i+2] = mySerial.read();
             delay(2);
             mCheck += mPkt[i+2];
          }
          mPkt[8] = mySerial.read();
          delay(1);
	  mPkt[9] = mySerial.read();
          if(mCheck == mPkt[8])//crc ok
          {
            mySerial.flush();
            //Serial.write(mPkt,10);

            Pm25 = (uint16_t)mPkt[2] | (uint16_t)(mPkt[3]<<8);
            Pm10 = (uint16_t)mPkt[4] | (uint16_t)(mPkt[5]<<8);
            if(Pm25 > 9999)
             Pm25 = 9999;
            if(Pm10 > 9999)
             Pm10 = 9999;            
            //get one good packet
             return;
          }
        }      
     }
   } 
}

void setup() {
  // 循环设置，把对应的端口都设置成输出
  // 这里使用德飞莱串口数码管模块，595芯片控制
    //pinMode(latch, OUTPUT); 
    //pinMode(ser, OUTPUT); 
    //pinMode(srclk, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    configure_wdt();
    
    Serial.begin(9600,SERIAL_8N1);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    Pm25=0;
    Pm10=0;
    Display();
    mySerial.begin(9600);
    
}
// 主循环

int time = 5;


void loop() {
  sleep(5);
  ProcessSerialData();
  Display();
  delay(1000);
  
}











