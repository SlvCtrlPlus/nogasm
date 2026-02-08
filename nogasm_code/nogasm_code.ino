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
//=======Libraries===============================
#include <Encoder.h>
#include <EEPROM.h>
#include "FastLED.h"
#include "RunningAverage.h"
#include <SerialCommands.h>
#include "comm.h"

//=======Hardware Setup===============================
//LEDs
#define NUM_LEDS 13
#define LED_PIN 17 //5V buffered pin on Teensy LC, single wire data out to WS8212Bs
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 255 //Subject to change, limits current that the LEDs draw

//Encoder
#define REDPIN   5 //RGB pins of the encoder LED
#define GREENPIN 4
#define BLUEPIN  3
#define ENC_SW   6 //Pushbutton on the encoder
Encoder myEnc(8, 7); //Quadrature inputs on pins 7,8

//DIP Switches
#define SW1PIN 12 //Dip switch pins, for setting software options without reflashing code
#define SW2PIN 11
#define SW3PIN 10
#define SW4PIN 9

//Motor
#define MOTPIN 23

//Pressure Sensor Analog In
#define BUTTPIN 15

//=======Software/Timing options=====================
#define FREQUENCY 60 //Update frequency in Hz
#define LONG_PRESS_MS 600 //ms requirements for a long press, to move to option menus
#define V_LONG_PRESS_MS 2500 //ms for a very long press, which turns the device off

//Update/render period
#define period (1000/FREQUENCY)
#define longBtnCount (LONG_PRESS_MS / period)

//Running pressure average array length and update frequency
#define RA_HIST_SECONDS 25
#define RA_FREQUENCY 6
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)
RunningAverage raPressure(RA_FREQUENCY*RA_HIST_SECONDS);

//=======State Machine Modes=========================
#define MANUAL      1
#define AUTO        2
#define OPT_SPEED   3
#define OPT_RAMPSPD 4
#define OPT_BEEP    5
#define OPT_PRES    6


//Button states - no press, short press, long press
#define BTN_NONE   0
#define BTN_SHORT  1
#define BTN_LONG   2
#define BTN_V_LONG 3


uint8_t state = MANUAL;

//=======SlvCtrl+====================================
const char* DEVICE_TYPE = "nogasm";
const int FW_VERSION = 10000; // 1.00.00
const int PROTOCOL_VERSION = 10000; // 1.00.00

char serial_command_buffer[32];
SerialCommands serialCommands(&Serial, serial_command_buffer, sizeof(serial_command_buffer), "\n", " ");

//=======Global Settings=============================
const uint8_t SENSITIVITY_MAX = 3 * (NUM_LEDS - 1);
const uint8_t MOT_MIN = 20; // Motor PWM minimum.  It needs a little more than this to start.

//DIP switch options:
uint8_t MOT_MAX =   179; //By default, motor speed only ever reaches 179. Alternative is 255
bool SW2 =        false;
bool SW3 =        false;
bool SW4 =        false;

CRGB leds[NUM_LEDS];

int pressure = 0;
int avgPressure = 0; //Running 25 second average pressure
//int bri =100; //Brightness setting
int rampTimeS = 30; //Ramp-up time, in seconds
#define DEFAULT_PLIMIT 600
int sensitivity = 0;
int pLimit = DEFAULT_PLIMIT; //Limit in change of pressure before the vibrator turns off
uint8_t maxSpeed = 255; //maximum speed the motor will ramp up to in automatic mode
float motSpeed = 0; //Motor speed, 0-255 (float to maintain smooth ramping to low speeds)

//=======EEPROM Addresses============================
//128b available on teensy LC
#define BEEP_ADDR         1
#define MAX_SPEED_ADDR    2
#define SENSITIVITY_ADDR  3
//#define RAMPSPEED_ADDR    4 //For now, ramp speed adjustments aren't implemented

//=======Setup=======================================
//Beep out tones over the motor by frequency (1047,1396,2093) may work well
void beep_motor(int f1, int f2, int f3){
  if(motSpeed > 245) analogWrite(MOTPIN, 245); //make sure the frequency is audible
  else if(motSpeed < 10) analogWrite(MOTPIN, 10);
  analogWriteFrequency(MOTPIN, f1);
  delay(250);
  analogWriteFrequency(MOTPIN, f2);
  delay(250);
  analogWriteFrequency(MOTPIN, f3);
  delay(250);
  analogWriteFrequency(MOTPIN, 440);
  analogWrite(MOTPIN,motSpeed);
}

void setup() {
  pinMode(REDPIN,   OUTPUT); //Connected to RGB LED in the encoder
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN,  OUTPUT);
  pinMode(ENC_SW,   INPUT); //Pin to read quadrature pulses from encoder

  pinMode(SW1PIN,   INPUT); //Set DIP switch pins as inputs
  pinMode(SW2PIN,   INPUT);
  pinMode(SW3PIN,   INPUT);
  pinMode(SW4PIN,   INPUT);

  digitalWrite(SW1PIN, HIGH); //Enable pullup resistors on DIP switch pins.
  digitalWrite(SW2PIN, HIGH); //They are tied to GND when switched on.
  digitalWrite(SW3PIN, HIGH);
  digitalWrite(SW4PIN, HIGH);

  pinMode(MOTPIN,OUTPUT); //Enable "analog" out (PWM)
  
  pinMode(BUTTPIN,INPUT); //default is 10 bit resolution (1024), 0-3.3
  analogReadRes(12); //changing ADC resolution to 12 bits (4095)
  analogReadAveraging(32); //To reduce noise, average 32 samples each read.
  
  raPressure.clear(); //Initialize a running pressure average

  digitalWrite(MOTPIN, LOW);//Make sure the motor is off

  //If a pin reads low, the switch is enabled. Here, we read in the DIP settings
  //Right now, only SW1 is used, for enabling higher maximum motor speed.
  if(digitalRead(SW1PIN)){
    MOT_MAX = 179; //At the default low position, limit the motor speed
  } else{
    MOT_MAX = 255; //When SW1 is flipped high, allow higher motor speeds
  }
  SW2 = (digitalRead(SW2PIN) == LOW);
  SW3 = (digitalRead(SW3PIN) == LOW);
  SW4 = (digitalRead(SW4PIN) == LOW);

  Serial.begin(9600);

  // Add commands
  serialCommands.SetDefaultHandler(commandUnrecognized);
  serialCommands.AddCommand(new SerialCommand("introduce", commandIntroduce));
  serialCommands.AddCommand(new SerialCommand("attributes", commandAttributes));
  serialCommands.AddCommand(new SerialCommand("status", commandStatus));
  serialCommands.AddCommand(new SerialCommand("set-mode", commandSetMode));
  serialCommands.AddCommand(new SerialCommand("set-maxSpeed", commandSetMaxSpeed));
  serialCommands.AddCommand(new SerialCommand("set-sensitivity", commandSetSensitivity));
  serialCommands.AddCommand(new SerialCommand("set-rampUpTime", commandSetRampUpTime));
  serialCommands.AddCommand(new SerialCommand("set-currentSpeed", commandSetCurrentSpeed));

  serialCommands.GetSerial()->write(0x07);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // limit power draw to .6A at 5v... Didn't seem to work in my FastLED version though
  //FastLED.setMaxPowerInVoltsAndMilliamps(5,DEFAULT_PLIMIT);
  FastLED.setBrightness(BRIGHTNESS);
  
  sensitivity = constrain(EEPROM.read(SENSITIVITY_ADDR), 0, SENSITIVITY_MAX);
  pLimit = map(sensitivity, 0, SENSITIVITY_MAX, 600, 1);

  maxSpeed = constrain(EEPROM.read(MAX_SPEED_ADDR), MOT_MIN, MOT_MAX); //Obey the MOT_MAX the first power  cycle after chaning it.
  beep_motor(1047,1396,2093); //Power on beep
}

//=======LED Drawing Functions=================

void showKnobRGB(const CRGB& rgb)
{
  analogWrite(REDPIN,   rgb.r );
  analogWrite(GREENPIN, rgb.g );
  analogWrite(BLUEPIN,  rgb.b );
}

//Draw a "cursor", one pixel representing either a pressure or encoder position value
//C1,C2,C3 are colors for each of 3 revolutions over the 13 LEDs (39 values)
void draw_cursor_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,NUM_LEDS*3-1);
  int colorNum = pos/NUM_LEDS; //revolution number
  int cursorPos = pos % NUM_LEDS; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      leds[cursorPos] = C1;
      break;
    case 1:
      leds[cursorPos] = C2;
      break;
    case 2:
      leds[cursorPos] = C3;
      break;
  }
}

//Draw a "cursor", one pixel representing either a pressure or encoder position value
void draw_cursor(int pos,CRGB C1){
  pos = constrain(pos,0,NUM_LEDS-1);
  leds[pos] = C1;
}

//Draw 3 revolutions of bars around the LEDs. From 0-39, 3 colors
void draw_bars_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,NUM_LEDS*3-1);
  int colorNum = pos/NUM_LEDS; //revolution number
  int barPos = pos % NUM_LEDS; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      fill_gradient_RGB(leds,0,C1,barPos,C1);
      //leds[barPos] = C1;
      break;
    case 1:
      fill_gradient_RGB(leds,0,C1,barPos,C2);
      break;
    case 2:
      fill_gradient_RGB(leds,0,C2,barPos,C3);
      break;
  }
}

//Provide a limited encoder reading corresponting to tacticle clicks on the knob.
//Each click passes through 4 encoder pulses. This reduces it to 1 pulse per click
int encLimitRead(int minVal, int maxVal){
  if(myEnc.read()>maxVal*4)myEnc.write(maxVal*4);
  else if(myEnc.read()<minVal*4) myEnc.write(minVal*4);
  return constrain(myEnc.read()/4,minVal,maxVal);
}

//=======Program Modes/States==================

// Manual vibrator control mode (red), still shows orgasm closeness in background
void run_manual() {
  //In manual mode, only allow for 13 cursor positions, for adjusting motor speed.
  int knob = encLimitRead(0, NUM_LEDS - 1);
  motSpeed = map(knob, 0, NUM_LEDS - 1, 0., (float)MOT_MAX);
  analogWrite(MOTPIN, (int)motSpeed);

  // @todo, needs to be fixed that already on boot up the sensitivity from EEPROM is used, if available
  int presDraw = map(constrain(pressure - avgPressure, 0, pLimit),0,pLimit,0,NUM_LEDS*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor(knob, CRGB::Red);
}

// Automatic edging mode, knob adjust sensitivity.
void run_auto() {
  static float motIncrement = 0.0;
  motIncrement = ((float)maxSpeed / ((float)FREQUENCY * (float)rampTimeS));

  sensitivity = encLimitRead(0, SENSITIVITY_MAX);
  //Reverse "Knob" to map it onto a pressure limit, so that it effectively adjusts sensitivity
  pLimit = map(sensitivity, 0, SENSITIVITY_MAX, 600, 1); //set the limit of delta pressure before the vibrator turns off
  //When someone clenches harder than the pressure limit
  if (pressure - avgPressure > pLimit) {
    motSpeed = -.5*(float)rampTimeS*((float)FREQUENCY*motIncrement);//Stay off for a while (half the ramp up time)
  }
  else if (motSpeed < (float)maxSpeed) {
    motSpeed += motIncrement;
  }
  if (motSpeed > MOT_MIN) {
    analogWrite(MOTPIN, (int) motSpeed);
  } else {
    analogWrite(MOTPIN, 0);
  }

  int presDraw = map(constrain(pressure - avgPressure, 0, pLimit),0,pLimit,0,NUM_LEDS*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor_3(sensitivity, CRGB(50,50,200),CRGB::Blue,CRGB::Purple);
}

//Setting menu for adjusting the maximum vibrator speed automatic mode will ramp up to
void run_opt_speed() {
  //Serial.println("speed settings");
  int knob = encLimitRead(0,12);
  motSpeed = map(knob, 0, 12, (float)MOT_MIN, (float)MOT_MAX);
  analogWrite(MOTPIN, (int)motSpeed);
  maxSpeed = constrain((uint8_t)motSpeed, MOT_MIN, MOT_MAX); //Set the maximum ramp-up speed in automatic mode
  //Little animation to show ramping up on the LEDs
  static int visRamp = 0;
  if(visRamp <= FREQUENCY*NUM_LEDS-1) visRamp += 16;
  else visRamp = 0;
  draw_bars_3(map(visRamp,0,(NUM_LEDS-1)*FREQUENCY,0,knob),CRGB::Green,CRGB::Green,CRGB::Green);
}

//Not yet added, but adjusts how quickly the vibrator turns back on after being triggered off
void run_opt_rampspd() {
  //Serial.println("rampSpeed");
}

//Also not completed, option for enabling/disabling beeps
void run_opt_beep() {
  //Serial.println("Brightness Settings");
}

//Simply display the pressure analog voltage. Useful for debugging sensitivity issues.
void run_opt_pres() {
  int p = map(analogRead(BUTTPIN),0,4095,0,NUM_LEDS-1);
  draw_cursor(p,CRGB::White);
}

//Poll the knob click button, and check for long/very long presses as well
uint8_t check_button(){
  static bool lastBtn = LOW;
  static unsigned long keyDownTime = 0;
  uint8_t btnState = BTN_NONE;
  bool thisBtn = digitalRead(ENC_SW);

  //Detect single presses, no repeating, on keyup
  if(thisBtn == HIGH && lastBtn == LOW){
    keyDownTime = millis();
  }
  
  if (thisBtn == LOW && lastBtn == HIGH) { //there was a keyup
    if((millis()-keyDownTime) >= V_LONG_PRESS_MS){
      btnState = BTN_V_LONG;
    }
    else if((millis()-keyDownTime) >= LONG_PRESS_MS){
      btnState = BTN_LONG;
      }
    else{
      btnState = BTN_SHORT;
      }
    }

  lastBtn = thisBtn;
  return btnState;
}

//run the important/unique parts of each state. Also, set button LED color.
void run_state_machine(uint8_t state){
  switch (state) {
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

//Switch between state machine states, and reset the encoder position as necessary
//Returns the next state to run. Very long presses will turn the system off (sort of)
uint8_t next_state_from_button(uint8_t btnState, uint8_t currentState) {
  if (btnState == BTN_NONE) return currentState;

  if (btnState == BTN_V_LONG) {
    return MANUAL; // “power off” is a separate behavior; keep as-is or handle elsewhere
  }

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

void transition_to(uint8_t newState)
{
  if (newState == state) return;

  // --- exit actions (based on OLD state) ---
  switch (state) {
    case AUTO:
      EEPROM.update(SENSITIVITY_ADDR, sensitivity);
      break;
    case OPT_SPEED:
      EEPROM.update(MAX_SPEED_ADDR, maxSpeed);
      break;
  }

  // --- entry actions (based on NEW state) ---
  switch (newState) {
    case MANUAL:
      myEnc.write(0);
      motSpeed = 0;
      analogWrite(MOTPIN, 0);
      break;

    case AUTO:
      myEnc.write(sensitivity * 4);   // sensitivity is led counts (0..36, 3x 12)
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

void power_off_sequence() {
  //Turn the device off until woken up by the button
  Serial.println("power off");
  fill_gradient_RGB(leds,0,CRGB::Black,NUM_LEDS-1,CRGB::Black);//Turn off LEDS
  FastLED.show();
  showKnobRGB(CRGB::Black);
  analogWrite(MOTPIN, 0);
  beep_motor(2093,1396,1047);
  analogWrite(MOTPIN, 0); //Turn Motor off
  while(!digitalRead(ENC_SW))delay(1);
  beep_motor(1047,1396,2093);
}

//=======Main Loop=============================
void loop() {
  static int sampleTick = 0;
  static unsigned long lastTick = 0;

  serialCommands.ReadSerial();

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
  FastLED.show(); // Update the physical LEDs to match the buffer in software

  //Alert that the Pressure voltage amplifier is railing, and the trim pot needs to be adjusted
  if (pressure > 4030) beep_motor(2093, 2093, 2093);
}

void commandIntroduce(SerialCommands* sender) {
  serial_printf(sender->GetSerial(), "introduce;%s,%d,%d\n", DEVICE_TYPE, FW_VERSION, PROTOCOL_VERSION);
}

void commandAttributes(SerialCommands* sender)
{
  serial_printf(
    sender->GetSerial(), 
    "attributes;timestampMs:ro[int],mode:rw[%d|%d],currentSpeed:%s[0-%d],maxSpeed:rw[%d-%d],currentPressure:ro[0-4095],avgPressure:ro[0-4095],sensitivity:rw[1-%d],maxPressureDelta:ro[1-600],rampUpTime:rw[10-60],remainingCoolDownTime:ro[0-15]\n", 
    MANUAL, AUTO, (state == AUTO ? "ro" : "rw"), MOT_MAX, MOT_MIN, MOT_MAX, SENSITIVITY_MAX + 1
  );
}

void commandStatus(SerialCommands* sender) {
  int statusMotSpeed = motSpeed;
  int remainingCooldown = 0;

  if (motSpeed < 0) {
    statusMotSpeed = 0; 
    remainingCooldown = (-motSpeed * rampTimeS) / maxSpeed;
  }

  serial_printf(
    sender->GetSerial(), 
    "status;timestampMs:%d,mode:%d,currentSpeed:%d,maxSpeed:%d,currentPressure:%d,avgPressure:%d,maxPressureDelta:%d,sensitivity:%d,rampUpTime:%d,remainingCoolDownTime:%d\n", 
    millis(), state, statusMotSpeed, maxSpeed, pressure, avgPressure, pLimit, sensitivity+1, rampTimeS, remainingCooldown
  );
}

void commandSetMode(SerialCommands* sender)
{
  long mode;
  ParamError err = validateRangeParam(sender->Next(), MANUAL, AUTO, mode);

  if (err != ParamError::Ok) {
    serial_printf(sender->GetSerial(), "set-mode;;status:failed,reason:%s\n", paramErrorToString(err));
    return;
  }

  transition_to((uint8_t)mode);

  serial_printf(sender->GetSerial(), "set-mode;%d;status:successful\n", state);
}

void commandSetMaxSpeed(SerialCommands* sender)
{
  long inputMaxSpeed;
  ParamError err = validateRangeParam(sender->Next(), MOT_MIN, MOT_MAX, inputMaxSpeed);

  if (err != ParamError::Ok) {
    serial_printf(sender->GetSerial(), "set-maxSpeed;;status:failed,reason:%s\n", paramErrorToString(err));
    return;
  }

  maxSpeed = inputMaxSpeed;

  // Clamp current motor speed to the new max
  if (motSpeed > maxSpeed) {
    motSpeed = maxSpeed;
  }

  serial_printf(sender->GetSerial(), "set-maxSpeed;%d;status:successful\n", maxSpeed);
}

void commandSetSensitivity(SerialCommands* sender)
{
  long inputSensitivity;
  ParamError err = validateRangeParam(sender->Next(), 1, SENSITIVITY_MAX + 1, inputSensitivity);

  if (err != ParamError::Ok) {
    serial_printf(sender->GetSerial(), "set-maxPressureDelta;;status:failed,reason:%s\n", paramErrorToString(err));
    return;
  }

  sensitivity = inputSensitivity - 1;
  pLimit = map(sensitivity, 0, SENSITIVITY_MAX, 600, 1);

  // set the thing
  if (state == AUTO) {
    myEnc.write(sensitivity * 4);
  }

  serial_printf(sender->GetSerial(), "set-maxPressureDelta;%d;status:successful\n", inputSensitivity);
}

void commandSetRampUpTime(SerialCommands* sender)
{
  long inputRampUpTime;
  ParamError err = validateRangeParam(sender->Next(), 10, 60, inputRampUpTime);

  if (err != ParamError::Ok) {
    serial_printf(sender->GetSerial(), "set-rampUpTime;;status:failed,reason:%s\n", paramErrorToString(err));
    return;
  }

  // set the thing
  rampTimeS = inputRampUpTime;

  serial_printf(sender->GetSerial(), "set-rampUpTime;%d;status:successful\n", rampTimeS);
}

void commandSetCurrentSpeed(SerialCommands* sender)
{
  if (state != MANUAL) {
    serial_printf(sender->GetSerial(), "set-currentSpeed;;status:failed,reason:not_in_manual_mode\n");
    return;
  }

  long inputCurrentSpeed;
  ParamError err = validateRangeParam(sender->Next(), 0, MOT_MAX, inputCurrentSpeed);

  if (err != ParamError::Ok) {
    serial_printf(sender->GetSerial(), "set-currentSpeed;;status:failed,reason:%s\n", paramErrorToString(err));
    return;
  }

  int knob = map(inputCurrentSpeed, 0, MOT_MAX, 0, NUM_LEDS - 1);
  myEnc.write(knob * 4);

  serial_printf(sender->GetSerial(), "set-currentSpeed;%d;status:successful\n", motSpeed);
}
