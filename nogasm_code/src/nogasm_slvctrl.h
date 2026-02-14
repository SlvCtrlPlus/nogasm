#pragma once

#include <Arduino.h>
#include <SlvCtrlProtocol.h>
#include <SlvCtrlArduinoSerialCommandsTransport.h>

// Create/initialize the protocol + transport. Call once in setup() AFTER Serial.begin().
void nogasmSlvCtrlInit(Stream& serial);

// Pump serial input. Call each loop().
void nogasmSlvCtrlReadCommand();
