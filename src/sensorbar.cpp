/******************************************************************************
sensorbar.cpp

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

#include "sensorbar.h"
#include "stdint.h"
#include "Wire.h"
#include "sx1509_registers.h"

//****************************************************************************//
//
//  Config and init functions
//
//****************************************************************************//
SensorBar::SensorBar(uint8_t address, uint8_t resetPin, uint8_t interruptPin, uint8_t oscillatorPin)
{
  // Store the received parameters into member variables
  deviceAddress = address;
  pinInterrupt = interruptPin;
  pinOscillator = oscillatorPin;
  pinReset = resetPin;

}

//void sx1509Class::debounceConfig(uint8_t configValue)
//{
//  // First make sure clock is configured
//  uint8_t tempuint8_t = readByte(REG_MISC);
//  if ((tempuint8_t & 0x70) == 0)
//  {
//    tempuint8_t |= (1 << 4);	// Just default to no divider if not set
//    writeByte(REG_MISC, tempuint8_t);
//  }
//  tempuint8_t = readByte(REG_CLOCK);
//  if ((tempuint8_t & 0x60) == 0)
//  {
//    tempuint8_t |= (1 << 6);	// default to internal osc.
//    writeByte(REG_CLOCK, tempuint8_t);
//  }
//
//  configValue &= 0b111;	// 3-bit value
//  writeByte(REG_DEBOUNCE_CONFIG, configValue);
//}
//
//void sx1509Class::debounceEnable(uint8_t pin)
//{
//  unsigned int debounceEnable = readWord(REG_DEBOUNCE_ENABLE_B);
//  debounceEnable |= (1 << pin);
//  writeWord(REG_DEBOUNCE_ENABLE_B, debounceEnable);
//}


uint8_t SensorBar::init(void)
{
  uint8_t returnVar = 0;
  if (pinInterrupt != 255)
  {
    pinMode(pinInterrupt, INPUT_PULLUP);
  }

  // Begin I2C
  Wire.begin();

  // If the reset pin is connected
  if (pinReset != 255)
    reset(1);
  else
    reset(0);

  // Communication test. We'll read from two registers with different
  // default values to verify communication.
  unsigned int testRegisters = 0;
  testRegisters = readWord(REG_INTERRUPT_MASK_A);	// This should return 0xFF00
  // Then read a uint8_t that should be 0x00
  if (testRegisters == 0xFF00)
  {
    returnVar = 1;
  }
  else
  {
    returnVar = 0;
  }
  
  writeByte(REG_DIR_A, 0xFF);
  return returnVar;
}

void SensorBar::reset(bool hardware)
{
  // if hardware bool is set
  if (hardware)
  {
    // Check if bit 2 of REG_MISC is set
    // if so nReset will not issue a POR, we'll need to clear that bit first
    uint8_t regMisc = readByte(REG_MISC);
    if (regMisc & (1 << 2))
    {
      regMisc &= ~(1 << 2);
      writeByte(REG_MISC, regMisc);
    }
    // Reset the SX1509, the pin is active low
    pinMode(pinReset, OUTPUT);	// set reset pin as output
    digitalWrite(pinReset, LOW);	// pull reset pin low
    delay(1);	// Wait for the pin to settle
    digitalWrite(pinReset, HIGH);	// pull reset pin back high
  }
  else
  {
    // Software reset command sequence:
    writeByte(REG_RESET, 0x12);
    writeByte(REG_RESET, 0x34);
  }
}

//void sx1509Class::enableInterrupt(uint8_t pin, uint8_t riseFall)
//{
//  // Set REG_INTERRUPT_MASK
//  unsigned int tempWord = readWord(REG_INTERRUPT_MASK_B);
//  tempWord &= ~(1 << pin);	// 0 = event on IO will trigger interrupt
//  writeWord(REG_INTERRUPT_MASK_B, tempWord);
//
//  uint8_t sensitivity = 0;
//  switch (riseFall)
//  {
//    case CHANGE:
//      sensitivity = 0b11;
//      break;
//    case FALLING:
//      sensitivity = 0b10;
//      break;
//    case RISING:
//      sensitivity = 0b01;
//      break;
//  }
//
//  // Set REG_SENSE_XXX
//  // Sensitivity is set as follows:
//  // 00: None
//  // 01: Rising
//  // 10: Falling
//  // 11: Both
//  uint8_t pinMask = (pin & 0x07) * 2;
//  uint8_t senseRegister;
//
//  // Need to select between two words. One for bank A, one for B.
//  if (pin >= 8)	senseRegister = REG_SENSE_HIGH_B;
//  else			senseRegister = REG_SENSE_HIGH_A;
//
//  tempWord = readWord(senseRegister);
//  tempWord &= ~(0b11 << pinMask);	// Mask out the bits we want to write
//  tempWord |= (sensitivity << pinMask);	// Add our new bits
//  writeWord(senseRegister, tempWord);
//}
//
//unsigned int sx1509Class::interruptSource(void)
//{
//  unsigned int intSource = readWord(REG_INTERRUPT_SOURCE_B);
//  writeWord(REG_INTERRUPT_SOURCE_B, 0xFFFF);	// Clear interrupts
//  return intSource;
//}
//
//void sx1509Class::configClock(uint8_t oscSource, uint8_t oscPinFunction, uint8_t oscFreqOut, uint8_t oscDivider)
//{
//  // RegClock constructed as follows:
//  //	6:5 - Oscillator frequency souce
//  //		00: off, 01: external input, 10: internal 2MHz, 1: reserved
//  //	4 - OSCIO pin function
//  //		0: input, 1 ouptut
//  //	3:0 - Frequency of oscout pin
//  //		0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
//  oscSource = (oscSource & 0b11) << 5;		// 2-bit value, bits 6:5
//  oscPinFunction = (oscPinFunction & 1) << 4;	// 1-bit value bit 4
//  oscFreqOut = (oscFreqOut & 0b1111);	// 4-bit value, bits 3:0
//  uint8_t regClock = oscSource | oscPinFunction | oscFreqOut;
//  writeByte(REG_CLOCK, regClock);
//
//  // Config RegMisc[6:4] with oscDivider
//  // 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
//  oscDivider = (oscDivider & 0b111) << 4;	// 3-bit value, bits 6:4
//  uint8_t regMisc = readByte(REG_MISC);
//  regMisc &= ~(0b111 << 4);
//  regMisc |= oscDivider;
//  writeByte(REG_MISC, regMisc);
//}

//****************************************************************************//
//
//  Bar functions
//
//****************************************************************************//

uint8_t SensorBar::getRaw( void )
{
  scan();
  return lastBarRawValue;
}


int8_t SensorBar::getPosition( void )
{
  //Assign values to each bit, -127 to 127, sum, and divide
  int16_t accumulator = 0;
  uint8_t bitsCounted = 0;
  int16_t i;

  //get input from the I2C machine
  scan();

  //count bits
  for ( i = 0; i < 8; i++ )
  {
    if ( ((lastBarRawValue >> i) & 0x01) == 1 )
    {
      bitsCounted++;
    }
  }

  //Find the vector value of each positive bit and sum
  for ( i = 7; i > 3; i-- )
  {
    if ( ((lastBarRawValue >> i) & 0x01) == 1 )
    {
      accumulator += ((32 * (i - 3)) - 1);
    }
  }
  for ( i = 0; i < 4; i++ )
  {
    if ( ((lastBarRawValue >> i) & 0x01) == 1 )
    {
      accumulator += ((-32 * (4 - i)) + 1);
    }
  }

  if ( bitsCounted > 0 )
  {
    lastBarRawValue = accumulator / bitsCounted;
  }
  else
  {
    lastBarRawValue = 0;
  }

  return lastBarRawValue;
}

uint8_t SensorBar::getDensity( void )
{
  uint8_t bitsCounted = 0;
  uint8_t i;

  //get input from the I2C machine
  scan();

  //count bits
  for ( i = 0; i < 8; i++ )
  {
    if ( ((lastBarRawValue >> i) & 0x01) == 1 )
    {
      bitsCounted++;
    }
  }
  return bitsCounted;
}

//****************************************************************************//
//
//  Utilities
//
//****************************************************************************//
void SensorBar::scan( void )
{
  //Operate the I2C machine
  lastBarRawValue = readByte( REG_DATA_A );  //Peel the data off port A

}

//uint8_t SensorBar::readPin(uint8_t pin)
//{
//  unsigned int tempRegDir = readWord(REG_DIR_B);
//
//  if (tempRegDir & (1 << pin))	// If the pin is an input
//  {
//    unsigned int tempRegData = readWord(REG_DATA_B);
//    if (tempRegData & (1 << pin))
//      return 1;
//  }
//
//  return 0;
//}

// readByte(uint8_t registerAddress)
//	This function reads a single uint8_t located at the registerAddress register.
//	- deviceAddress should already be set by the constructor.
//	- Return value is the uint8_t read from registerAddress
//		- Currently returns 0 if communication has timed out
uint8_t SensorBar::readByte(uint8_t registerAddress)
{
  uint8_t readValue;
  unsigned int timeout = RECEIVE_TIMEOUT_VALUE;

  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t) 1);

  while ((Wire.available() < 1) && (timeout != 0))
    timeout--;

  if (timeout == 0)
    return 0;

  readValue = Wire.read();

  return readValue;
}

// readWord(uint8_t registerAddress)
//	This function will read a two-uint8_t word beginning at registerAddress
//	- A 16-bit unsigned int will be returned.
//		- The msb of the return value will contain the value read from registerAddress
//		- The lsb of the return value will contain the value read from registerAddress + 1
unsigned int SensorBar::readWord(uint8_t registerAddress)
{
  unsigned int readValue;
  unsigned int msb, lsb;
  unsigned int timeout = RECEIVE_TIMEOUT_VALUE * 2;

  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t) 2);

  while ((Wire.available() < 2) && (timeout != 0))
    timeout--;

  if (timeout == 0)
    return 0;

  msb = (Wire.read() & 0x00FF) << 8;
  lsb = (Wire.read() & 0x00FF);
  readValue = msb | lsb;

  return readValue;
}

// readBytes(uint8_t firstRegisterAddress, uint8_t * destination, uint8_t length)
//	This function reads a series of uint8_ts incrementing from a given address
//	- firstRegsiterAddress is the first address to be read
//	- destination is an array of uint8_ts where the read values will be stored into
//	- length is the number of uint8_ts to be read
//	- No return value.
void SensorBar::readBytes(uint8_t firstRegisterAddress, uint8_t * destination, uint8_t length)
{
  uint8_t readValue;

  Wire.beginTransmission(deviceAddress);
  Wire.write(firstRegisterAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, length);

  while (Wire.available() < length)
    ;

  for (int i = 0; i < length; i++)
  {
    destination[i] = Wire.read();
  }
}

// writeByte(uint8_t registerAddress, uint8_t writeValue)
//	This function writes a single uint8_t to a single register on the SX509.
//	- writeValue is written to registerAddress
//	- deviceAddres should already be set from the constructor
//	- No return value.
void SensorBar::writeByte(uint8_t registerAddress, uint8_t writeValue)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(writeValue);
  Wire.endTransmission();
}

// writeWord(uint8_t registerAddress, ungisnged int writeValue)
//	This function writes a two-uint8_t word to registerAddress and registerAddress + 1
//	- the upper uint8_t of writeValue is written to registerAddress
//		- the lower uint8_t of writeValue is written to registerAddress + 1
//	- No return value.
void SensorBar::writeWord(uint8_t registerAddress, unsigned int writeValue)
{
  uint8_t msb, lsb;
  msb = ((writeValue & 0xFF00) >> 8);
  lsb = (writeValue & 0x00FF);
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(msb);
  Wire.write(lsb);
  Wire.endTransmission();
}

// writeBytes(uint8_t firstRegisterAddress, uint8_t * writeArray, uint8_t length)
//	This function writes an array of uint8_ts, beggining at a specific adddress
//	- firstRegisterAddress is the initial register to be written.
//		- All writes following will be at incremental register addresses.
//	- writeArray should be an array of uint8_t values to be written.
//	- length should be the number of uint8_ts to be written.
//	- no return value.
void SensorBar::writeBytes(uint8_t firstRegisterAddress, uint8_t * writeArray, uint8_t length)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(firstRegisterAddress);
  for (int i = 0; i < length; i++)
  {
    Wire.write(writeArray[i]);
  }
  Wire.endTransmission();
}


//****************************************************************************//
//
//  Circular stack
//
//****************************************************************************//

CircularStack::CircularStack()
{
  cStackLastPtr = 0;
  cStackElementsUsed = 0;  
}

int16_t CircularStack::getElement( uint8_t elementNum ) //zero is the push location.  Max is CSTACK_MAX_LENGTH - 1
{
  //Translate elementNum into terms of cStackLastPtr.
  int16_t virtualElementNum;
  virtualElementNum = cStackLastPtr - elementNum;
  if( virtualElementNum < 0 )
  {
    virtualElementNum += CSTACK_MAX_LENGTH;
  }
  
  //Output the value
  return cStackData[virtualElementNum];
}

void CircularStack::pushElement( int16_t elementVal )
{
  //inc. the pointer
  cStackLastPtr++;

  //deal with roll
  if( cStackLastPtr >= CSTACK_MAX_LENGTH )
  {
    cStackLastPtr = 0;
  }

  //write data
  cStackData[cStackLastPtr] = elementVal;

  //increase length up to CSTACK_MAX_LENGTH
  if( cStackElementsUsed < CSTACK_MAX_LENGTH )
  {
    cStackElementsUsed++;
  }
}

int16_t CircularStack::averageLast( uint8_t numElements )
{
  //Add up all the elements
  int32_t accumulator = 0;
  int8_t i;
  for( i = 0; i < numElements; i++ )
  {
    accumulator += getElement( i );
  }
  //Divide by number of elements
  accumulator /= numElements;
  return accumulator;
}

uint8_t CircularStack::recordLength( void )
{
  return cStackElementsUsed;
}

