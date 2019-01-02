/*******************************************************************************
   Copyright (c) 2018 Vekotinverstas / Forum Virium Helsinki

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   LoRaWAN part is heavily copied from lmic library's examples.

   Do not forget to define the radio type correctly
   #define CFG_eu868 1
   in
   libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h or from your BOARDS.txt.

 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "settings.h"
// Sensor support libraries
#include "SDS011.h"
#include <HardwareSerial.h>

#include "SSD1306.h" 

#include "bsec.h"



#include "QuickStats.h"

QuickStats stats;

// LoRa payload
#define payloadSize 18
static uint8_t payload[payloadSize];

// I2C settings
// TODO: Check correct ESP32 pins
#define SDA 2
#define SCL 1



SDS011 sds011;

#define DUST_SERIAL_RX  39
#define DUST_SERIAL_TX  36
//#define SDS011_RXPIN 39
//#define SDS011_TXPIN 36

HardwareSerial dustSerial(1);

uint8_t sds011_ok = 0;
#define pm_array_size 120
uint32_t pm_array_counter = 0;
float sds011_pm25[pm_array_size];
float sds011_pm10[pm_array_size];

SSD1306 display(0x3c, 21, 22);


Bsec iaqSensor;
String output;

static osjob_t sendjob;

void setup() {
    pinMode(16,OUTPUT);
      digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);

  while (!Serial); // wait for Serial to be initialized
  Serial.begin(115200);
  delay(100);     // per sample code on RF_95 test
  Serial.println(F("Starting"));
  // SDS011 serial
//  Serial2.begin(9600, SERIAL_8N1, DUST_SERIAL_RX /*SDS011_RXPIN*/, DUST_SERIAL_TX /*SDS011_TXPIN*/);
  dustSerial.begin(9600, SERIAL_8N1, DUST_SERIAL_RX, DUST_SERIAL_TX);
  init_sensors();
  lmic_init();
  // Start job
  do_send(&sendjob);
  // Initialise payload
  for (uint8_t i=0; i < payloadSize; i++) {
    payload[i] = 0;
  }
}

String msg;
String bme680Msg1;
String bme680Msg2;
String loraMsg;

void loop() {
  unsigned long now;
  now = millis();
  ostime_t ostime = os_getTime();
  
  if ((now & 512) != 0) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }
  String m = read_sensors();

  display.clear();
  display.drawString(0, 0, String(now) + " ms, " + String(ostime) + " ticks");
  display.drawString(0,10, msg);
  display.drawString(0,20, m);
  display.drawString(0,30, bme680Msg1);
  display.drawString(0,40, bme680Msg2);
  display.drawString(0,50, loraMsg);
  
  display.display();

  os_runloop_once();
}

uint32_t addToPayload(uint8_t pl[], uint16_t val, uint32_t i) {
  pl[i++] = val >> 8;
  pl[i++] = val & 0x00FF;
  return i;
}

/**
   Generate payload which is an uint8_t array full of uint16_t values meaning
   bytes 0 and 1 contain the first uint16_t value and so on.
*/
void generatePayload() {
  float min25 = 0;
  float max25 = 0;
  float avg25 = 0;
  float med25 = 0;

  float min10 = 0;
  float max10 = 0;
  float avg10 = 0;
  float med10 = 0;

  // More examples for statistics
  // https://github.com/dndubins/QuickStats/blob/master/examples/statistics/statistics.ino
  if (pm_array_counter > 0) {
    min25 = stats.minimum(sds011_pm25, pm_array_counter);
    max25 = stats.maximum(sds011_pm25, pm_array_counter);
    avg25 = stats.average(sds011_pm25, pm_array_counter);
    med25 = stats.median(sds011_pm25, pm_array_counter);
    min10 = stats.minimum(sds011_pm10, pm_array_counter);
    max10 = stats.maximum(sds011_pm10, pm_array_counter);
    avg10 = stats.average(sds011_pm10, pm_array_counter);
    med10 = stats.median(sds011_pm10, pm_array_counter);
  }
  char buffer [100];
  int cx;
  cx = snprintf ( buffer, 100, "Values to send: min2.5 %.1f max2.5 %.1f avg2.5 %.1f med2.5 %.1f min10 %.1f max10 %.1f avg10 %.1f med10 %.1f",
                  min25, max25, avg25, med25, min10, max10, avg10, med10 );

msg = String(med25)+ " " + String(med10);
  Serial.println(buffer);
  uint16_t tmp;
  uint8_t i = 0;

  // 2 first bytes defines protocol
  payload[i++] = 0x2A;
  payload[i++] = 0x2A;
  i = addToPayload(payload, (uint16_t)(min25 * 10), i);
  i = addToPayload(payload, (uint16_t)(max25 * 10), i);
  i = addToPayload(payload, (uint16_t)(avg25 * 10), i);
  i = addToPayload(payload, (uint16_t)(med25 * 10), i);
  i = addToPayload(payload, (uint16_t)(min10 * 10), i);
  i = addToPayload(payload, (uint16_t)(max10 * 10), i);
  i = addToPayload(payload, (uint16_t)(avg10 * 10), i);
  i = addToPayload(payload, (uint16_t)(med10 * 10), i);

  pm_array_counter = 0;
}
