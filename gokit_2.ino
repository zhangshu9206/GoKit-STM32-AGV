#include <DHT.h>
#include <I2Cdev.h>
#include <MemoryFree.h>
#include <MsTimer2.h>
#include <SSD1306.h>
#include <ChainableLED.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Wire.h"
#include "GoKit.h"
#include "MotorCar.h"

void setup()
{

  GoKit_Init();
  #if (DEBUG==1)
  Serial.println("GoKit init  OK!");
  Serial.print("freeMemory()=");
  Serial.println(freeMemory());
  #endif
  #if(MYSERIAL_DATA==1)
  mySerial.println("GoKit init  OK! - mySerial");
  #endif   
  #if(MYSERIAL1_DATA==1)
  mySerial_1.println("GoKit init  OK! - mySerial_1");
  #endif
  
}
void loop()
{  
  GoKit_Handle();
}

