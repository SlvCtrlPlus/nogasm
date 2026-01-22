#include "comm.h"

void serial_printf(Stream *serial, const char* format, ...) {
  va_list args;
  va_start(args, format);

  int bufferSize = vsnprintf(NULL, 0, format, args);
  bufferSize++;  // safe byte for \0

  char buffer[bufferSize];

  vsnprintf(buffer, bufferSize, format, args);

  va_end(args);

  serial->print(buffer);
}

void commandUnrecognized(SerialCommands* sender, const char* cmd)
{
    serial_printf(sender->GetSerial(), "Unrecognized command [%s]\n", cmd);
}

#include <stdlib.h>

ParamError validateRangeParam(const char* valueStr, long minVal, long maxVal, long& outValue)
{
  if (valueStr == nullptr) return ParamError::Missing;

  char* end = nullptr;
  long value = strtol(valueStr, &end, 10);

  if (end == valueStr || *end != '\0') return ParamError::NotANumber;
  if (value < minVal || value > maxVal) return ParamError::OutOfRange;

  outValue = value;
  return ParamError::Ok;
}

const char* paramErrorToString(ParamError err)
{
  switch (err) {
    case ParamError::Ok:         return "ok";
    case ParamError::Missing:    return "param_missing";
    case ParamError::NotANumber: return "not_a_number";
    case ParamError::OutOfRange: return "out_of_range";
    default:                     return "unknown_error";
  }
}
