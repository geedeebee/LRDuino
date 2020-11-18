///*
//* Utility functions to help debugging running code.
//*/
//
//#ifndef DEBUGUTILS_H
//#define DEBUGUTILS_H
//
////#include <WProgram.h>
//
//#define DEBUG_PRINT(str) \
//   Serial.print(millis()); \
//   Serial.print(": "); \
//   Serial.print(__FUNCTION__); \
//   Serial.print("() at line  "); \
//   Serial.print(__FILE__); \
//   Serial.print(':'); \
//   Serial.print(__LINE__); \
//   Serial.print(' '); \
//   Serial.println(str);
//
//#endif


/*
DebugUtils.h - Simple debugging utilities.
*/

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

#endif
