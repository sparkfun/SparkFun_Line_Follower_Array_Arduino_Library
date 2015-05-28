#include "chartools.h"
#include "Wire.h"
#include "sensorbar.h"
#include <RedBot.h>

//Protos
void updateDriveState( void );


// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);

CircularStack positionHistory;
RedBotMotors motors; // Instantiate the motor control object. This only needs
// to be done once.


// State machine states
#define IDLE_STATE 0
#define READ_LINE 1
#define HOLD 2

// Drive states
#define DRIVE_STOP 0
#define DRIVE_GO 1
#define DRIVE_GO_HARD 2
#define DRIVE_LEFT 3
#define DRIVE_LEFT_HARD 4
#define DRIVE_RIGHT 5
#define DRIVE_RIGHT_HARD 6

uint8_t state;
uint8_t driveState;
uint8_t targetDriveState;

int8_t centerThreshold = 15;
int8_t wayOutThreshold = 65;
uint16_t holdTime = 3;

uint16_t msTick = 0;
uint16_t holdTicks = 0;

int8_t lastAverage = 0;


void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop()
{
  uint8_t densityOK = 0;
  //Wait 1 ms
  delay(1);

  //Increment the tick
  msTick++;

  //Do ms related activities
  //update drive state
  updateDriveState();
  int temp = mySensorBar.getDensity();
  if( (temp < 4)&&(temp > 0) )
  {
    positionHistory.pushElement( mySensorBar.getPosition());
  }

  //If the tick is too high, decrement it and do long scale activities
  if( msTick > 20 )
  {
    Serial.println( state );
    msTick -= 20;
    uint8_t nextState = state;  //default state
    switch (state) {
    case IDLE_STATE:
      targetDriveState = DRIVE_STOP;       // Stops both motors
      nextState = READ_LINE;
      break;
    case READ_LINE:
      if( (temp < 4)&&(temp > 0) )
      {
        //Density OK
        densityOK = 1;
        lastAverage = positionHistory.averageLast( 50 );
      }
      if( densityOK == 1 )
      {
        if( (lastAverage < centerThreshold)&&(lastAverage > -centerThreshold) )
        {
          //In the center-- Go hard!
          targetDriveState = DRIVE_GO_HARD;
        }
        else
        {
          if(lastAverage > 0 )
          {
            //in the left side
            if( (lastAverage < wayOutThreshold)&&(lastAverage > centerThreshold) )
            {
              //not that far out left
              targetDriveState = DRIVE_LEFT;
            }
            else
            {
              //Way out!
              targetDriveState = DRIVE_LEFT_HARD;
              nextState = HOLD;
            }
          }
          else
          {
            //in the right side
            if( (lastAverage > -wayOutThreshold)&&(lastAverage < -centerThreshold) )
            {
              //not that far out right
              targetDriveState = DRIVE_RIGHT;
            }
            else
            {
              //Way out!
              targetDriveState = DRIVE_RIGHT_HARD;
              nextState = HOLD;
            }
          }
        }
      }
    case HOLD:
      holdTicks++;
      if( holdTicks > holdTime )
      {
        holdTicks = 0;
        nextState = READ_LINE;
      }
      break;
    default:
      targetDriveState = DRIVE_STOP;
      break;
    }
    state = nextState;
  }

}


void updateDriveState( void )
{
  if( driveState != targetDriveState )
  {
    driveState = targetDriveState;
    switch (driveState) {
    case DRIVE_STOP:
      motors.stop();
      break;
    case DRIVE_GO:
      motors.rightMotor(100);
      motors.leftMotor(-100);
      break;
    case DRIVE_GO_HARD:
      motors.rightMotor(200);
      motors.leftMotor(-200);
      break;
    case DRIVE_LEFT:
      motors.rightMotor(80);
      motors.leftMotor(-50);
      break;
    case DRIVE_LEFT_HARD:
      motors.rightMotor(100);
      motors.leftMotor(-0);
      break;
    case DRIVE_RIGHT:
      motors.rightMotor(50);
      motors.leftMotor(-80);
      break;
    case DRIVE_RIGHT_HARD:
      motors.rightMotor(0);
      motors.leftMotor(-100);
      break;
    default:
      motors.stop();       // Stops both motors
      break;
    }
  }
}












