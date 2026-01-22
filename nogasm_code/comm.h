#ifndef COMM_H_INCLUDED
#define COMM_H_INCLUDED

#include <Arduino.h>        // uint8_t, Stream
#include <SerialCommands.h> // SerialCommands

enum class ParamError : uint8_t {
    Ok = 0,
    Missing,
    NotANumber,
    OutOfRange
};

ParamError validateRangeParam(const char* valueStr, long minVal, long maxVal, long& outValue);
const char* paramErrorToString(ParamError err);

void serial_printf(Stream *serial, const char* format, ...);
void commandUnrecognized(SerialCommands* sender, const char* cmd);

#endif // COMM_H_INCLUDED
