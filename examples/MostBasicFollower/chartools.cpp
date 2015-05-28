#include "chartools.h"

//This function changes an integer to a string, null terminated
//
//  You better not provide a string of length shorter than length + 1....
//
void hexString(char* opString, int input, int length)
{
  int i;
  for(i = 0; i < length; i++)
  {
    opString[i] = hex2char((input >> (4 * (length - 1 -i))) & 0xF);
  }
  opString[length] = 0x00;
  
  return;
}

// This takes a int input and converts to char
//
// The output will be an char if the int
// is between zero and 0xF
//
// Otherwise, the output is $.
//
char hex2char(int hexin)
{
  int charout;
  charout = 0x24; // default $ state
  if(hexin >= 0x00)
  {
    if(hexin <= 0x09)
    {
      charout = hexin + 0x30;
    }
  }
  if(hexin >= 0x0A)
  {
    if(hexin <= 0x0F)
    {
      charout = hexin -10 + 0x41;
    }
  }
  return charout;
}

