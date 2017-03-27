#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control

unsigned int Pm25 = 0;
unsigned int Pm10 = 0;

// how many times remain to sleep before wake up
int nbr_remaining; 

// pin on which a led is attached on the board
#define led 13

// interrupt raised by the watchdog firing
// when the watchdog fires during sleep, this function will be executed
// remember that interrupts are disabled in ISR functions
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
 while (Serial.available() > 0) 
  {  
    // from www.inovafitness.com
    // packet format: AA C0 PM25_Low PM25_High PM10_Low PM10_High 0 0 CRC AB
     mData = Serial.read();     delay(2);//wait until packet is received
    if(mData == 0xAA)//head1 ok
     {
        mPkt[0] =  mData;
        mData = Serial.read();
        if(mData == 0xc0)//head2 ok
        {
          mPkt[1] =  mData;
          mCheck = 0;
          for(i=0;i < 6;i++)//data recv and crc calc
          {
             mPkt[i+2] = Serial.read();
             delay(2);
             mCheck += mPkt[i+2];
          }
          mPkt[8] = Serial.read();
          delay(1);
    mPkt[9] = Serial.read();
          if(mCheck == mPkt[8])//crc ok
          {
            Serial.flush();
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

void setup(){
  Serial.begin(9600, SERIAL_8N1);
  Pm25=0;
    Pm10=0;
    Display();
  // use led 13 and put it in low mode
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  
  delay(1000);
  
  // configure the watchdog
  configure_wdt();

  // blink twice
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 

}

void loop(){

  // sleep for a given number of cycles (here, 5 * 8 seconds) in lowest power mode
  sleep(1);

  // usefull stuff should be done here before next long sleep
  // blink three times
  ProcessSerialData();
  Display();
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 

}
