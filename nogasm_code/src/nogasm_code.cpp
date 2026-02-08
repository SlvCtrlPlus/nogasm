// Jul 2016 - Nogasm Code Rev. 3
/* Drives a vibrator and uses changes in pressure of an inflatable buttplug
 * to estimate a user's closeness to orgasm, and turn off the vibrator
 * before that point.
 * A state machine updating at 60Hz creates different modes and option menus
 * that can be identified by the color of the LEDs, especially the RGB LED
 * in the central button/encoder knob.
 * 
 * [Red]    Manual Vibrator Control
 * [Blue]   Automatic vibrator edging, knob adjusts orgasm detection sensitivity
 * [Green]  Setting menu for maximum vibrator speed in automatic mode
 * [White]  Debubbing menu to show data from the pressure sensor ADC
 * [Off]    While still plugged in, holding the button down for >3 seconds turns
 *          the whole device off, until the button is pressed again.
 * 
 * Settings like edging sensitivity, or maximum motor speed are stored in EEPROM,
 * so they are saved through power-cycling.
 * 
 * In the automatic edging mode, the vibrator speed will linearly ramp up to full
 * speed (set in the green menu) over 30 seconds. If a near-orgasm is detected,
 * the vibrator abruptly turns off for 15 seconds, then begins ramping up again.
 * 
 * The motor will beep during power on/off, and if the plug pressure rises above
 * the maximum the board can read - this condition could lead to a missed orgasm 
 * if unchecked. The analog gain for the sensor is adjustable via a trimpot to
 * accomidate different types of plugs that have higher/lower resting pressures.
 * 
 * Motor speed, current pressure, and average pressure are reported via USB serial
 * at 115200 baud. Timestamps can also be enabled, from the main loop.
 * 
 * There is some framework for more features like an adjustable "cool off" time 
 * other than the default 15 seconds, and options for LED brightness and enabling/
 * disabling beeps. Four DIP switches are included on the board to allow users to
 * change other software settings without reflashing code. One example use would 
 * be switch 1 toggling between the defaul ramping motor behavior, and a strict
 * ON/OFF output to the motor that could instead be used to toggle a relay for 
 * driving other toys.
 * 
 * Note - Do not set all 13 LEDs to white at full brightness at once 
 * (RGB 255,255,255) It may overheat the voltage regulator and cause the board 
 * to reset.
 */
#include <Arduino.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <RunningAverage.h>

#include "nogasm_code.h"
#include "nogasm_slvctrl.h"

//================ Hardware Setup ===============================
// LEDs
#define LED_PIN 17
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 255

// Encoder LED pins / button
#define REDPIN   5
#define GREENPIN 4
#define BLUEPIN  3
#define ENC_SW   6

// DIP Switches
#define SW1PIN 12
#define SW2PIN 11
#define SW3PIN 10
#define SW4PIN 9

// Motor
#define MOTPIN 23

// Pressure Sensor Analog In
#define BUTTPIN 15

//================ Software / Timing ============================
#define FREQUENCY 60

#define period (1000 / FREQUENCY)

// Running pressure average
#define RA_HIST_SECONDS 25
#define RA_FREQUENCY 6
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)
RunningAverage raPressure(RA_FREQUENCY * RA_HIST_SECONDS);

//================ SlvCtrl Identity (shared) =====================
const char* DEVICE_TYPE = "nogasm";
const uint32_t FW_VERSION = 10000; // 1.00.00

//================ Globals required by nogasm_app.h ===============
Encoder myEnc(8, 7);

uint8_t state = MANUAL;

uint8_t MOT_MAX = 179;

int pressure = 0;
int avgPressure = 0;
int rampTimeS = 30;

#define DEFAULT_PLIMIT 600
int sensitivity = 0;
int pLimit = DEFAULT_PLIMIT;

uint8_t maxSpeed = 255;
float motSpeed = 0;

//================ EEPROM Addresses ==============================
#define BEEP_ADDR         1
#define MAX_SPEED_ADDR    2
#define SENSITIVITY_ADDR  3

CRGB leds[NUM_LEDS];

//================ Helpers =======================================
void beep_motor(int f1, int f2, int f3) {
  if (motSpeed > 245) analogWrite(MOTPIN, 245);
  else if (motSpeed < 10) analogWrite(MOTPIN, 10);
  analogWriteFrequency(MOTPIN, f1);
  delay(250);
  analogWriteFrequency(MOTPIN, f2);
  delay(250);
  analogWriteFrequency(MOTPIN, f3);
  delay(250);
  analogWriteFrequency(MOTPIN, 440);
  analogWrite(MOTPIN, (int)motSpeed);
}

static void showKnobRGB(const CRGB& rgb) {
  analogWrite(REDPIN, rgb.r);
  analogWrite(GREENPIN, rgb.g);
  analogWrite(BLUEPIN, rgb.b);
}

static void draw_cursor_3(int pos, CRGB C1, CRGB C2, CRGB C3) {
  pos = constrain(pos, 0, NUM_LEDS * 3 - 1);
  int colorNum = pos / NUM_LEDS;
  int cursorPos = pos % NUM_LEDS;
  switch (colorNum) {
    case 0: leds[cursorPos] = C1; break;
    case 1: leds[cursorPos] = C2; break;
    case 2: leds[cursorPos] = C3; break;
  }
}

static void draw_cursor(int pos, CRGB C1) {
  pos = constrain(pos, 0, NUM_LEDS - 1);
  leds[pos] = C1;
}

static void draw_bars_3(int pos, CRGB C1, CRGB C2, CRGB C3) {
  pos = constrain(pos, 0, NUM_LEDS * 3 - 1);
  int colorNum = pos / NUM_LEDS;
  int barPos = pos % NUM_LEDS;
  switch (colorNum) {
    case 0: fill_gradient_RGB(leds, 0, C1, barPos, C1); break;
    case 1: fill_gradient_RGB(leds, 0, C1, barPos, C2); break;
    case 2: fill_gradient_RGB(leds, 0, C2, barPos, C3); break;
  }
}

static int encLimitRead(int minVal, int maxVal) {
  if (myEnc.read() > maxVal * 4) myEnc.write(maxVal * 4);
  else if (myEnc.read() < minVal * 4) myEnc.write(minVal * 4);
  return constrain(myEnc.read() / 4, minVal, maxVal);
}

//================ Modes =========================================
static void run_manual() {
  int knob = encLimitRead(0, NUM_LEDS - 1);
  motSpeed = map(knob, 0, NUM_LEDS - 1, 0., (float)MOT_MAX);
  analogWrite(MOTPIN, (int)motSpeed);

  int presDraw = map(constrain(pressure - avgPressure, 0, pLimit), 0, pLimit, 0, NUM_LEDS * 3);
  draw_bars_3(presDraw, CRGB::Green, CRGB::Yellow, CRGB::Red);
  draw_cursor(knob, CRGB::Red);
}

static void run_auto() {
  static float motIncrement = 0.0;
  motIncrement = ((float)maxSpeed / ((float)FREQUENCY * (float)rampTimeS));

  sensitivity = encLimitRead(0, SENSITIVITY_MAX);
  pLimit = map(sensitivity, 0, SENSITIVITY_MAX, 600, 1);

  if (pressure - avgPressure > pLimit) {
    motSpeed = -.5 * (float)rampTimeS * ((float)FREQUENCY * motIncrement);
  } else if (motSpeed < (float)maxSpeed) {
    motSpeed += motIncrement;
  }

  if (motSpeed > MOT_MIN) analogWrite(MOTPIN, (int)motSpeed);
  else analogWrite(MOTPIN, 0);

  int presDraw = map(constrain(pressure - avgPressure, 0, pLimit), 0, pLimit, 0, NUM_LEDS * 3);
  draw_bars_3(presDraw, CRGB::Green, CRGB::Yellow, CRGB::Red);
  draw_cursor_3(sensitivity, CRGB(50, 50, 200), CRGB::Blue, CRGB::Purple);
}

static void run_opt_speed() {
  int knob = encLimitRead(0, 12);
  motSpeed = map(knob, 0, 12, (float)MOT_MIN, (float)MOT_MAX);
  analogWrite(MOTPIN, (int)motSpeed);
  maxSpeed = constrain((uint8_t)motSpeed, MOT_MIN, MOT_MAX);

  static int visRamp = 0;
  if (visRamp <= FREQUENCY * NUM_LEDS - 1) visRamp += 16;
  else visRamp = 0;
  draw_bars_3(map(visRamp, 0, (NUM_LEDS - 1) * FREQUENCY, 0, knob), CRGB::Green, CRGB::Green, CRGB::Green);
}

static void run_opt_rampspd() {}
static void run_opt_beep() {}

static void run_opt_pres() {
  int p = map(analogRead(BUTTPIN), 0, 4095, 0, NUM_LEDS - 1);
  draw_cursor(p, CRGB::White);
}

//================ Button / State machine ========================
static uint8_t check_button() {
  static bool lastBtn = LOW;
  static unsigned long keyDownTime = 0;
  uint8_t btnState = BTN_NONE;
  bool thisBtn = digitalRead(ENC_SW);

  if (thisBtn == HIGH && lastBtn == LOW) {
    keyDownTime = millis();
  }

  if (thisBtn == LOW && lastBtn == HIGH) {
    if ((millis() - keyDownTime) >= V_LONG_PRESS_MS) btnState = BTN_V_LONG;
    else if ((millis() - keyDownTime) >= LONG_PRESS_MS) btnState = BTN_LONG;
    else btnState = BTN_SHORT;
  }

  lastBtn = thisBtn;
  return btnState;
}

static void run_state_machine(uint8_t s) {
  switch (s) {
    case MANUAL:
      showKnobRGB(CRGB::Red);
      run_manual();
      break;
    case AUTO:
      showKnobRGB(CRGB::Blue);
      run_auto();
      break;
    case OPT_SPEED:
      showKnobRGB(CRGB::Green);
      run_opt_speed();
      break;
    case OPT_RAMPSPD:
      showKnobRGB(CRGB::Yellow);
      run_opt_rampspd();
      break;
    case OPT_BEEP:
      showKnobRGB(CRGB::Purple);
      run_opt_beep();
      break;
    case OPT_PRES:
      showKnobRGB(CRGB::White);
      run_opt_pres();
      break;
    default:
      run_manual();
      break;
  }
}

static uint8_t next_state_from_button(uint8_t btnState, uint8_t currentState) {
  if (btnState == BTN_NONE) return currentState;

  if (btnState == BTN_V_LONG) return MANUAL;

  if (btnState == BTN_SHORT) {
    switch (currentState) {
      case MANUAL:      return AUTO;
      case AUTO:        return MANUAL;
      case OPT_SPEED:   return OPT_PRES;
      case OPT_PRES:    return OPT_SPEED;
      case OPT_RAMPSPD: return OPT_BEEP;
      case OPT_BEEP:    return OPT_PRES;
      default:          return MANUAL;
    }
  }

  if (btnState == BTN_LONG) {
    switch (currentState) {
      case MANUAL: return OPT_SPEED;
      case AUTO:   return OPT_SPEED;
      default:     return MANUAL;
    }
  }

  return currentState;
}

// transition_to is declared in nogasm_app.h and used by SlvCtrl setters
void transition_to(uint8_t newState) {
  if (newState == state) return;

  switch (state) {
    case AUTO:      EEPROM.update(SENSITIVITY_ADDR, sensitivity); break;
    case OPT_SPEED: EEPROM.update(MAX_SPEED_ADDR, maxSpeed);      break;
  }

  switch (newState) {
    case MANUAL:
      myEnc.write(0);
      motSpeed = 0;
      analogWrite(MOTPIN, 0);
      break;

    case AUTO:
      myEnc.write(sensitivity * 4);
      motSpeed = 0;
      analogWrite(MOTPIN, 0);
      break;

    case OPT_SPEED:
      myEnc.write(map(maxSpeed, MOT_MIN, MOT_MAX, 0, 4 * (NUM_LEDS - 1)));
      motSpeed = 0;
      analogWrite(MOTPIN, 0);
      break;

    case OPT_PRES:
      myEnc.write(0);
      motSpeed = 0;
      analogWrite(MOTPIN, 0);
      break;
  }

  state = newState;
}

static void power_off_sequence() {
  Serial.println("power off");
  fill_gradient_RGB(leds, 0, CRGB::Black, NUM_LEDS - 1, CRGB::Black);
  FastLED.show();
  showKnobRGB(CRGB::Black);
  analogWrite(MOTPIN, 0);
  beep_motor(2093, 1396, 1047);
  analogWrite(MOTPIN, 0);
  while (!digitalRead(ENC_SW)) delay(1);
  beep_motor(1047, 1396, 2093);
}

//================ Arduino setup/loop ============================
void setup() {
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  pinMode(ENC_SW, INPUT);

  pinMode(SW1PIN, INPUT);
  pinMode(SW2PIN, INPUT);
  pinMode(SW3PIN, INPUT);
  pinMode(SW4PIN, INPUT);

  digitalWrite(SW1PIN, HIGH);
  digitalWrite(SW2PIN, HIGH);
  digitalWrite(SW3PIN, HIGH);
  digitalWrite(SW4PIN, HIGH);

  pinMode(MOTPIN, OUTPUT);

  pinMode(BUTTPIN, INPUT);
  analogReadRes(12);
  analogReadAveraging(32);

  raPressure.clear();

  digitalWrite(MOTPIN, LOW);

  if (digitalRead(SW1PIN)) MOT_MAX = 179;
  else MOT_MAX = 255;

  Serial.begin(9600);

  nogasmSlvCtrlInit(Serial);
  Serial.write(0x07);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  sensitivity = constrain(EEPROM.read(SENSITIVITY_ADDR), 0, SENSITIVITY_MAX);
  pLimit = map(sensitivity, 0, SENSITIVITY_MAX, 600, 1);

  maxSpeed = constrain(EEPROM.read(MAX_SPEED_ADDR), MOT_MIN, MOT_MAX);
  beep_motor(1047, 1396, 2093);
}

void loop() {
  static int sampleTick = 0;
  static unsigned long lastTick = 0;

  nogasmSlvCtrlReadCommand();

  unsigned long now = millis();
  if (now - lastTick < period) return;
  lastTick = now;

  pressure = analogRead(BUTTPIN);

  sampleTick++;
  if (sampleTick % RA_TICK_PERIOD == 0) {
    raPressure.addValue(pressure);
    avgPressure = raPressure.getAverage();
  }

  fadeToBlackBy(leds, NUM_LEDS, 20);

  uint8_t btnState = check_button();

  if (btnState == BTN_V_LONG) {
    power_off_sequence();
    transition_to(MANUAL);
    return;
  }

  uint8_t next = next_state_from_button(btnState, state);
  transition_to(next);

  run_state_machine(state);
  FastLED.show();

  if (pressure > 4030) beep_motor(2093, 2093, 2093);
}
