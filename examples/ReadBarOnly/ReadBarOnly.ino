#include "chartools.h"
#include "Wire.h"
#include "sensorbar.h"
//#include "sx1509_library.h"

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);

void setup()
{
  mySensorBar.init();
  //Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop()
{
  int16_t i;
  uint8_t rawValue = mySensorBar.getRaw();
  //print binary binary
  Serial.print("Bin value of input: ");
  for( i = 7; i >= 0; i-- )
  {
    Serial.print((rawValue >> i) & 0x01);
  }
  Serial.println("b\n");
  //print raw
  char myString[9];
  hexString(myString, rawValue, 2);
  Serial.print("Hex value of bar: 0x");
  Serial.println(myString);
  //get vectors
  Serial.print("Position (-127 to 127): ");
  //Serial.println(mySensorBar.getPosition());
  Serial.print("Density, bits detected (of 8): ");
  //Serial.println(mySensorBar.getDensity());
  Serial.println("");
  delay(500);
}



