# Smart-Wearable-Environmental-Monitoring-System---Arduino

This is a arduino project which can measure the PM2.5 particle density in the surrounding. 
PM2.5 particles means particles which exist in the environment and whose size is less than 2.5 micrometers. 
Since it is too small it is easily inhaled by the respiratory system without being detected. 
As a result human beings get sick and some even get lung cancer.

This device will be made wearable will monitor the surround 24*7 and it records data in a SD card module. 

#Products needed. 
1. SDS011 PM2.5 sensor module
2. Arduino Nano or Uno
3. Arduino SD card module




##Pin configurations: (Module -- Arduino Nano)

#SDS011

1. Rx   -- Tx
2. Tx   -- Rx
3. VCC  -- 5V
4. GND  -- GND


#SD Card Module

1. CS   -- D4
2. SCK  -- D13
3. MOSI -- D11
4. MISO -- D12
5. VCC  -- 5V
6. GND  -- GND

#DS3231

1. GND  -- GND
2. VCC  -- 5V
3. SDA  -- A4
4. SCL  -- A5




