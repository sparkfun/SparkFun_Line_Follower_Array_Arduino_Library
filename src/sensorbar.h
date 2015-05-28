/******************************************************************************
sensorbar.h

RedBot Sensor Bar Library

Marshall Taylor, SparkFun Engineering

5-27-2015

https://github.com/sparkfun/RedBot_Line_Follower_Bar

This is a library for reading the sensor bar's data by I2C.  It was originally
adapted from the ""SparkFun SX1509 IO Expander Breakout Arduino Library" that
was written by Jim Lindblom.

Resources:
Relies on the I2C driver (wire.h).  Declaring an object of type SensorBar and
then calling .init() causes wire.h to operate

Development environment specifics:
Tested on the RedBot 328 based arduino board.

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef SENSORBAR_H
#define SENSORBAR_H

#include "stdint.h"
//#include "Arduino.h"

#define RECEIVE_TIMEOUT_VALUE 1000	// Timeout for I2C receive

// These are used for setting LED driver to linear or log mode:
#define LINEAR		0
#define LOGARITHMIC	1

// These are used for clock config:
#define INTERNAL_CLOCK	2
#define EXTERNAL_CLOCK	1

class SensorBar
{
  public:
    SensorBar(uint8_t address, uint8_t resetPin = 255, uint8_t interruptPin = 255, uint8_t oscillatorPin = 255);
    uint8_t init(void);
    uint8_t getRaw( void );
    int8_t getPosition( void );
    uint8_t getDensity( void );
    void scan( void );
    //move input to private eventually
//    uint8_t input;

    void reset(bool hardware);
//    void debounceEnable(uint8_t pin);
//    void enableInterrupt(uint8_t pin, uint8_t riseFall);
//    unsigned int interruptSource(void);
//    void configClock(uint8_t oscSource = 2, uint8_t oscPinFunction = 0, uint8_t oscFreqOut = 0, uint8_t oscDivider = 1);


  //private:
    uint8_t lastBarRawValue;
    uint8_t deviceAddress; // I2C Address of SX1509

    // Pin definitions:
    uint8_t pinInterrupt;
    uint8_t pinOscillator;
    uint8_t pinReset;

    // Read Functions:
    uint8_t readByte(uint8_t registerAddress);
    unsigned int readWord(uint8_t registerAddress);
    void readBytes(uint8_t firstRegisterAddress, uint8_t * destination, uint8_t length);
    // Write functions:
    void writeByte(uint8_t registerAddress, uint8_t writeValue);
    void writeWord(uint8_t registerAddress, unsigned int writeValue);
    void writeBytes(uint8_t firstRegisterAddress, uint8_t * writeArray, uint8_t length);
};

//****************************************************************************//
//
//  Circular stack
//
//****************************************************************************//

//Class CircularStack is int16_t
//Does not care about over-running real data ( if request is outside length's bounds )
//The underlying machine writes [48], [49], [0], [1] ... 
#define CSTACK_MAX_LENGTH 500

class CircularStack
{
public:
    CircularStack();
    int16_t getElement( uint8_t ); //zero is the push location
    void pushElement( int16_t );
    int16_t averageLast( uint8_t );
    uint8_t recordLength( void );
private:
    int16_t cStackData[CSTACK_MAX_LENGTH];
    int16_t cStackLastPtr;
    uint8_t cStackElementsUsed;
};



#endif // SENSORBAR_H
