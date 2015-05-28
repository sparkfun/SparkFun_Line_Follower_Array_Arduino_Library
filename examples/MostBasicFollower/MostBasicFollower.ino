#include "chartools.h"
#include "Wire.h"
#include "sensorbar.h"
#include <RedBot.h>  // This line "includes" the RedBot library into your sketch.
// Provides special objects, methods, and functions for the RedBot.

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);

RedBotMotors motors; // Instantiate the motor control object. This only needs
// to be done once.

#define IDLE_STATE 0
#define READ_LINE 1
#define GO_FORWARD 2
#define GO_LEFT 3
#define GO_RIGHT 4

uint8_t state;


void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  mySensorBar.init();

}

void loop()
{
//  int16_t i;
//  uint8_t rawValue = mySensorBar.getRaw();
//  //print binary binary
//  Serial.print("Bin value of input: ");
//  for( i = 7; i >= 0; i-- )
//  {
//    Serial.print((rawValue >> i) & 0x01);
//  }
//  Serial.println("b\n");
//  //print raw
//  char myString[9];
//  hexString(myString, rawValue, 2);
//  Serial.print("Hex value of bar: 0x");
//  Serial.println(myString);
//  //get vectors
//  Serial.print("Position (-127 to 127): ");
//  Serial.println(mySensorBar.getPosition());
//  Serial.print("Density, bits detected (of 8): ");
//  Serial.println(mySensorBar.getDensity());
//  Serial.println("");
//  delay(500);

  uint8_t nextState = state;
  switch (state) {
  case IDLE_STATE:
    motors.stop();       // Stops both motors
    nextState = READ_LINE;
    break;
  case READ_LINE:
    if( mySensorBar.getDensity() < 4 )
    {
      nextState = GO_FORWARD;
      if( mySensorBar.getPosition() > 50 )
      {
        nextState = GO_RIGHT;
      }
      if( mySensorBar.getPosition() < -50 )
      {
        nextState = GO_LEFT;
      }
    }
    else
    {
      nextState = IDLE_STATE;
    }
    break;
  case GO_FORWARD:
    motors.rightMotor(100);
    motors.leftMotor(-100);
    nextState = READ_LINE;
    break;
  case GO_LEFT:
    motors.rightMotor(80);
    motors.leftMotor(-0);
    nextState = READ_LINE;
    break;
  case GO_RIGHT:
    motors.rightMotor(0);
    motors.leftMotor(-80);
    nextState = READ_LINE;
    break;
  default:
    motors.stop();       // Stops both motors
    break;
  }
  state = nextState;
  //delay(100);
}




