#pragma once

#include <Arduino.h>
#include <Encoder.h>

// ---------- Hardware / constants (shared) ----------
static constexpr int NUM_LEDS = 13;

static constexpr uint8_t MANUAL      = 1;
static constexpr uint8_t AUTO        = 2;
static constexpr uint8_t OPT_SPEED   = 3;
static constexpr uint8_t OPT_RAMPSPD = 4;
static constexpr uint8_t OPT_BEEP    = 5;
static constexpr uint8_t OPT_PRES    = 6;

static constexpr uint8_t SENSITIVITY_MAX = 3 * (NUM_LEDS - 1);
static constexpr uint8_t MOT_MIN = 20;

// ---------- Button states (shared) ----------
static constexpr uint8_t BTN_NONE   = 0;
static constexpr uint8_t BTN_SHORT  = 1;
static constexpr uint8_t BTN_LONG   = 2;
static constexpr uint8_t BTN_V_LONG = 3;

static constexpr uint16_t LONG_PRESS_MS   = 600;
static constexpr uint16_t V_LONG_PRESS_MS = 2500;

// ---------- Identity ----------
extern const char* DEVICE_TYPE;
extern const uint32_t FW_VERSION;

// ---------- App state / globals (defined in nogasm_code.cpp) ----------
extern Encoder myEnc;

extern uint8_t state;

extern uint8_t MOT_MAX;

extern int pressure;
extern int avgPressure;
extern int rampTimeS;
extern int sensitivity;
extern int pLimit;
extern uint8_t maxSpeed;
extern float motSpeed;

// transition function (defined in nogasm_code.cpp)
void transition_to(uint8_t newState);
