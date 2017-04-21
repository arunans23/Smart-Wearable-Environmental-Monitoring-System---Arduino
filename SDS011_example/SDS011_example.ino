// SDS011 dust sensor example
// -----------------------------
//
// By R. Zschiegner (rz@madavi.de).
// April 2016

#include <SDS011.h>

float p10,p25;
int error;

SDS011 my_sds;

void setup() {
	my_sds.begin(2,3);
	Serial.begin(9600);
}

int count = 1;

void loop() {
	error = my_sds.read(&p25,&p10);
	if (! error) {
    Serial.println(count);
		Serial.println("P2.5: "+String(p25));
		Serial.println("P10:  "+String(p10));
	}
	delay(100);
  count++;

// error = my_sds.read(&p25,&p10);
// if (! error) {
//  
//    Serial.println("2");
//    Serial.println("P2.5: "+String(p25));
//    Serial.println("P10:  "+String(p10));
//  }
//  delay(100);
//
//  error = my_sds.read(&p25,&p10);
//  if (! error) {
//    
//    Serial.println("3");
//    Serial.println("P2.5: "+String(p25));
//    Serial.println("P10:  "+String(p10));
//  }
//  delay(100);
//
//  error = my_sds.read(&p25,&p10);
//  if (! error) {
//    
//    Serial.println("4");
//    Serial.println("P2.5: "+String(p25));
//    Serial.println("P10:  "+String(p10));
//  }
//  delay(100);
//
//  error = my_sds.read(&p25,&p10);
//  if (! error) {
//    
//    Serial.println("5");
//    Serial.println("P2.5: "+String(p25));
//    Serial.println("P10:  "+String(p10));
//  }
//  delay(100);
//
// my_sds.sleep();
// delay(5000);
// my_sds.wakeup();
// delay(1000);
//
// error = my_sds.read(&p25,&p10);
//  if (! error) {
//    
//    Serial.println("6");
//    Serial.println("P2.5: "+String(p25));
//    Serial.println("P10:  "+String(p10));
//  }
//  delay(100);
}