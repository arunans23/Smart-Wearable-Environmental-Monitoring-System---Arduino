#include <SD.h>
#include <SPI.h>
#include <SDS011.h>

float p10, p25;
int error;

SDS011 my_sds;
File myFile;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  my_sds.begin(2, 3);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

}

void loop() {
  // put your main code here, to run repeatedly:

}
