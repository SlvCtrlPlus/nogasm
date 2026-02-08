#include "nogasm_slvctrl.h"
#include "nogasm_code.h"

#include <SlvCtrlProtocol.h>
#include <SlvCtrlArduinoSerialCommandsTransport.h>

// ----- Getters -----
static int32_t getTimestampMs(void*) { return (int32_t)millis(); }
static int32_t getMode(void*) { return (int32_t)state; }

static int32_t getCurrentSpeed(void*) {
  if (motSpeed < 0) return 0;
  return (int32_t)motSpeed;
}

static int32_t getMaxSpeed(void*) { return (int32_t)maxSpeed; }
static int32_t getCurrentPressure(void*) { return (int32_t)pressure; }
static int32_t getAvgPressure(void*) { return (int32_t)avgPressure; }
static int32_t getSensitivityHuman(void*) { return (int32_t)(sensitivity + 1); }
static int32_t getMaxPressureDelta(void*) { return (int32_t)pLimit; }
static int32_t getRampUpTime(void*) { return (int32_t)rampTimeS; }

static int32_t getRemainingCoolDownTime(void*) {
  if (motSpeed >= 0) return 0;
  return (int32_t)((-motSpeed * (float)rampTimeS) / (float)maxSpeed);
}

// ----- Setters (must match old behavior) -----
static SlvCtrlParseError setMode(void*, int32_t v) {
  if (v < (int32_t)MANUAL || v > (int32_t)AUTO) return SlvCtrlParseError::OutOfRange;
  transition_to((uint8_t)v);
  return SlvCtrlParseError::Ok;
}

static SlvCtrlParseError setMaxSpeedAttr(void*, int32_t v) {
  // DIP-dependent upper bound
  if (v < (int32_t)MOT_MIN || v > (int32_t)MOT_MAX) return SlvCtrlParseError::OutOfRange;

  maxSpeed = (uint8_t)v;

  // Clamp current motor speed
  if (motSpeed > maxSpeed) {
    motSpeed = (float)maxSpeed;
  }
  return SlvCtrlParseError::Ok;
}

static SlvCtrlParseError setSensitivityAttr(void*, int32_t v) {
  if (v < 1 || v > (int32_t)SENSITIVITY_MAX + 1) return SlvCtrlParseError::OutOfRange;

  sensitivity = (int)v - 1;
  pLimit = map(sensitivity, 0, SENSITIVITY_MAX, 600, 1);

  if (state == AUTO) {
    myEnc.write(sensitivity * 4);
  }
  return SlvCtrlParseError::Ok;
}

static SlvCtrlParseError setRampUpTimeAttr(void*, int32_t v) {
  if (v < 10 || v > 60) return SlvCtrlParseError::OutOfRange;
  rampTimeS = (int)v;
  return SlvCtrlParseError::Ok;
}

static SlvCtrlParseError setCurrentSpeedAttr(void*, int32_t v) {
  if (state != MANUAL) return SlvCtrlParseError::InvalidValue;
  if (v < 0 || v > (int32_t)MOT_MAX) return SlvCtrlParseError::OutOfRange;

  int knob = map((int)v, 0, MOT_MAX, 0, NUM_LEDS - 1);
  myEnc.write(knob * 4);

  return SlvCtrlParseError::Ok;
}

// ----- Attributes -----
static IntAttribute aTimestampMs("timestampMs", &getTimestampMs, nullptr);
static RangeAttribute<int32_t> aMode("mode", &getMode, &setMode, (int32_t)MANUAL, (int32_t)AUTO);

// NOTE: Protocol access is static; cannot reflect "currentSpeed is RO in AUTO" without a custom attribute wrapper.
static RangeAttribute<int32_t> aCurrentSpeed("currentSpeed", &getCurrentSpeed, &setCurrentSpeedAttr, 0, 255);
static RangeAttribute<int32_t> aMaxSpeed("maxSpeed", &getMaxSpeed, &setMaxSpeedAttr, (int32_t)MOT_MIN, 255);

static RangeAttribute<int32_t> aCurrentPressure("currentPressure", &getCurrentPressure, nullptr, 0, 4095);
static RangeAttribute<int32_t> aAvgPressure("avgPressure", &getAvgPressure, nullptr, 0, 4095);
static RangeAttribute<int32_t> aSensitivity("sensitivity", &getSensitivityHuman, &setSensitivityAttr, 1, (int32_t)SENSITIVITY_MAX + 1);
static RangeAttribute<int32_t> aMaxPressureDelta("maxPressureDelta", &getMaxPressureDelta, nullptr, 1, 600);
static RangeAttribute<int32_t> aRampUpTime("rampUpTime", &getRampUpTime, &setRampUpTimeAttr, 10, 60);
static RangeAttribute<int32_t> aRemainingCoolDownTime("remainingCoolDownTime", &getRemainingCoolDownTime, nullptr, 0, 15);

static IAttribute* ATTRS[] = {
  &aTimestampMs,
  &aMode,
  &aCurrentSpeed,
  &aMaxSpeed,
  &aCurrentPressure,
  &aAvgPressure,
  &aSensitivity,
  &aMaxPressureDelta,
  &aRampUpTime,
  &aRemainingCoolDownTime
};

static SlvCtrlProtocol gProto(DEVICE_TYPE, FW_VERSION, ATTRS);

// Transport storage
static char gCmdBuf[64];
static SlvCtrlSerialCommandsTransport* gTransport = nullptr;

void nogasmSlvCtrlInit(Stream& serial) {
  static SlvCtrlSerialCommandsTransport transport(serial, gProto, gCmdBuf, sizeof(gCmdBuf));
  gTransport = &transport;
}

void nogasmSlvCtrlReadCommand() {
  if (gTransport) gTransport->readCommand();
}
