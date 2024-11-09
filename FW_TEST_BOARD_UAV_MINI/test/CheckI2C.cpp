#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define WIRE Wire

#define TXD_1 (PA9)
#define RXD_1 (PA10)
#define SDA_1 (PB7)
#define SCL_1 (PB6)

HardwareSerial Serial1(RXD_1, TXD_1);

void setup() {
  WIRE.setSDA(SDA_1);
  WIRE.setSCL(SCL_1);
  WIRE.setClock(400000);  
  WIRE.begin();    
  // WIRE.begin();

  Serial1.begin(115200);
  while (!Serial1)
    delay(10);
  Serial1.println("\nI2C Scanner");
}


void loop() {
  byte error, address;
  int nDevices;

  Serial1.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial1.print("I2C device found at address 0x");
      if (address<16) Serial1.print("0");
      Serial1.print(address,HEX);
      Serial1.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial1.print("Unknown error at address 0x");
      if (address<16) Serial1.print("0");
      Serial1.println(address,HEX);
    }
  }
  if (nDevices == 0) Serial1.println("No I2C devices found\n");
  else Serial1.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}