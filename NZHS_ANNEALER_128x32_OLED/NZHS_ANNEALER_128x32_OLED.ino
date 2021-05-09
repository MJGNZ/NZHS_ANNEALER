/*---------------------------------------------------------------------------*/
/*! @brief      Brass Cartridge Case Annealer.
  @details      None.
  @author       Justin Spence, Mark Griffith. 2020
  @note         circuitworksnz@gmail.com
*//*-------------------------------------------------------------------------*/

//--Includes-------------------------------------------------------------------
#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <OneWire.h> 
#include <DallasTemperature.h>

//-- macros---------------------------------------------------------------
//#define DEBUG //defining DEBUG will remove the splash screen and enable serial debug info
#define SERVO
//                        Major Version
//                        | Minor Version
//                        | | LCD Type
//                        | | | 
//                        | | | 
//                        | | | 
#define SOFTWARE_VERSION "3.0.0"
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define PSU_OVERCURRENT 12300 //12.3A
#define TEMP_RESOLUTION 9 //ADC resolution on temp sensor
#define TEMP_LIMIT 55 //capacitor temperature limit degC
#define TEMP_CONVERSION_TIME 120 //measurement time for DS18B20 9,10,11,12 bit = 95ms, 190ms, 375ms, 750ms
#define TEMP_HYSTERESIS 15 //define how much temperature needs to drop to resume
#define DROP_TIME 500 //time to drop the case in ms
#define RELOAD_TIME 5000 //time for user to load a new case in free run mode (ms)
#define RELOAD_TIME_AUTO__FEED 2000 //time to feed case in auto feed mode (ms) - recommend leaving at 2000
#define MIN_ANNEAL_TIME 2000 //min anneal time in ms
#define MAX_ANNEAL_TIME 8000 //max anneal time in ms
#define LONG_PRESS_HOLD_TIME 15 //loop iterations for long button press e.g. 15 x 100ms = 1.5s press and hold
#define LOOP_TIME 120  //ms per main loop iteration
#define COOLDOWN_PERIOD 300000 //Cooling period in milliseconds
#define DISPLAY_ADDRESS 0x3C
#define SERVO_OPEN_POSITION 5  //timer load value for servo pulse. 128us per timer count. 7 => 0.89ms pulse
#define SERVO_CLOSE_POSITION 15 // 15 => 1.92ms pulse
#define STEPPER_STEPS_PER_TURN 200*STEPPER_MICROSTEPS // stepper motor steps per revolution (e.g. 200 step motor) * microsteps.
#define STEPPER_MICROSTEPS 16 // number of microsteps. set to 1 if no microstepping
#define CASE_FEEDER_STEPS_DROP_TO_PRELOAD 185*STEPPER_MICROSTEPS
#define CASE_FEEDER_STEPS_PRELOAD_TO_DROP (STEPPER_STEPS_PER_TURN - CASE_FEEDER_STEPS_DROP_TO_PRELOAD)
#define CASE_FEEDER_HOPPER_START 70*STEPPER_MICROSTEPS
#define CASE_FEEDER_HOPPER_END 130*STEPPER_MICROSTEPS
#define MODE_KEY_USED  //defines the use of the mode key input. comment out this #define to disable mode selection and reassign the mode key input to force case drop in the event of a stuck case

// temp sensor pin asignment DS1820
#define ONE_WIRE_BUS 8

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 100000, 100000);
// Setup a oneWire instance to communicate with any OneWire devices  
OneWire oneWire(ONE_WIRE_BUS); 
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Global variables :(
DeviceAddress tempDeviceAddress;
static uint8_t NumberDallasTempDevices = 0;
static bool CurrentSensorPresent = 0;
static uint16_t psuCurrentZeroOffset = 0;
static uint16_t StepsToGo = 0;
static uint16_t StepsFromHome = 0;
static bool StepToggle = 0;

//--define state machine states-----------------------------------------------------------
typedef enum tStateMachineStates
{
  STATE_STOPPED = 0, 
  STATE_ANNEALING,
  STATE_DROPPING,
  STATE_RELOADING,
  STATE_COOLDOWN,
  STATE_JUST_BOOTED,
  STATE_SHOW_WARNING,
  STATE_SHOW_SOFTWARE_VER,
  STATE_OVERCURRENT_WARNING,
  STATE_UNKNOWN,
} tStateMachineStates;

typedef enum ModeList
{
  MODE_SINGLE_SHOT = 0, 
  MODE_FREE_RUN,
  MODE_AUTOMATIC,
} ModeList;

//--global constant declarations-----------------------------------------
static const uint8_t g_StartStopButtonPin   = 2;
static const uint8_t g_ModeButtonPin        = 3;
static const uint8_t g_AnnealerPin          = 6;
static const uint8_t g_DropServoPin         = 9;
static const uint8_t g_FeederStepPin       = 12;
static const uint8_t g_StartStopLedPin      = 4;
static const uint8_t g_CoolingFanPin        = 7;
static const uint8_t g_ModeLedPin           = 11;
static const uint8_t g_PsuCurrentAdcPin     = 0;
static const uint8_t g_DropSolenoidPin      = 10;
static const uint8_t g_TimeSetButtonPin     = 16;
static const uint8_t g_FeederDirPin         = 13;
static const uint8_t g_FeederStepperEnPin   = 5;


 // custom startup image, 128x32px
const unsigned char anneallogo [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00,
0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0x00,
0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00,
0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xC3, 0x3F, 0x30, 0xC7, 0x80, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xE3, 0x3F, 0x30, 0xCF, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xE3, 0x03, 0x30, 0xCC, 0x40, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xB3, 0x06, 0x30, 0xCC, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xB3, 0x0C, 0x3F, 0xCF, 0x80, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x9B, 0x1C, 0x3F, 0xC3, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x8B, 0x18, 0x30, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x8F, 0x30, 0x30, 0xC8, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x87, 0x3F, 0x30, 0xCF, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x87, 0x3F, 0x30, 0xCF, 0x80, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00,
0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char anneallogo2 [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00,
0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0x00,
0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00,
0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x60, 0xE1, 0x9C, 0x33, 0xF0, 0x60, 0xC1, 0xF9, 0xF8, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0xF0, 0xF1, 0x9E, 0x33, 0xF0, 0xF0, 0xC1, 0xF9, 0xFC, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0xF0, 0xF1, 0x9E, 0x33, 0x00, 0xF0, 0xC1, 0x81, 0x8C, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x01, 0x98, 0xD9, 0x9B, 0x33, 0x01, 0x98, 0xC1, 0x81, 0x8C, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x01, 0x98, 0xD9, 0x9B, 0x33, 0xE1, 0x98, 0xC1, 0xF1, 0xF8, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x03, 0x0C, 0xCD, 0x99, 0xB3, 0xE3, 0x0C, 0xC1, 0xF1, 0xF0, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x03, 0xFC, 0xCD, 0x99, 0xB3, 0x03, 0xFC, 0xC1, 0x81, 0x98, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x03, 0xFC, 0xC7, 0x98, 0xF3, 0x03, 0xFC, 0xC1, 0x81, 0x98, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x06, 0x06, 0xC7, 0x98, 0xF3, 0xF6, 0x06, 0xFD, 0xF9, 0x8C, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x06, 0x06, 0xC3, 0x98, 0x73, 0xF6, 0x06, 0xFD, 0xF9, 0x8C, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00,
0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char projectile [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xFE, 0x00, 0x07, 0xC0, 0x01, 0xF0, 0x7F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x3C, 0x1F, 0x00, 0x03, 0xF0, 0x00, 0xFC, 0x00, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x0F, 0xC0, 0x01, 0xFC, 0x00, 0x7F, 0x80, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x03, 0xF0, 0x00, 0x7F, 0x00, 0x0F, 0xC0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0xFC, 0x00, 0x1F, 0x80, 0x03, 0xE0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x7F, 0x00, 0x07, 0xE0, 0x00, 0xC0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x1F, 0xC0, 0x01, 0xF8, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x7E, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x00, 0x1F, 0x80, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x07, 0xE0, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0x00, 0x00, 0x30, 0x00, 0x1F, 0x80, 0x03, 0xF8, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xE0, 0x00, 0x7C, 0x00, 0x0F, 0xE0, 0x00, 0xFE, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x3E, 0x00, 0x03, 0xF0, 0x00, 0x3F, 0x80, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x1F, 0x80, 0x00, 0xFC, 0x00, 0x0F, 0xC0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x07, 0xE0, 0x00, 0x3F, 0x00, 0x03, 0xE0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x3C, 0x01, 0xF0, 0x00, 0x0F, 0x80, 0x00, 0xE0, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0xF8, 0x00, 0x07, 0xC0, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char projectile2 [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x0F, 0x80, 0x01, 0xF0, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x07, 0xC0, 0x00, 0xFC, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x03, 0xF0, 0x00, 0x7F, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x1F, 0x80, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x07, 0xE0, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xE0, 0x00, 0x18, 0x00, 0x1F, 0xC0, 0x01, 0xF8, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0x00, 0x00, 0x3E, 0x00, 0x07, 0xF0, 0x00, 0x7E, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x1F, 0x80, 0x01, 0xF8, 0x00, 0x3F, 0x80, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x30, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x7E, 0x00, 0x0F, 0xC0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x1F, 0x80, 0x03, 0xE0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x07, 0xE0, 0x00, 0xE0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x3F, 0x00, 0x03, 0xF8, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0xFC, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x07, 0xF0, 0x00, 0x3F, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x01, 0xFC, 0x00, 0x0F, 0xC0, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x7E, 0x00, 0x03, 0xE0, 0x00, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x1F, 0x00, 0x01, 0xF0, 0x7F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


//-- global variables declarations----------------------------------------
static tStateMachineStates g_SystemState = STATE_JUST_BOOTED;
static tStateMachineStates g_SystemStatePrev = STATE_UNKNOWN;

#ifdef MODE_KEY_USED
  static ModeList CurrentMode = MODE_SINGLE_SHOT; //mode key is used so set default mode to single shot
#else
  static ModeList CurrentMode = MODE_FREE_RUN; //mode key is not used so set default mode to free run
#endif

//-- function declarations------------------------------------------------
static tStateMachineStates updateSystemState(tStateMachineStates const state);
static bool hasSystemStateChanged(void);
static bool readStartButton(void);
static bool readModeButton(void);
static bool readUpButton(void);
static void turnAnnealerOn(void);
static void turnAnnealerOff(void);
static void openDropGate(void);
static void closeDropGate(void);
static void turnStartStopLedOn(void);
static void turnStartStopLedOff(void);
static void turnModeLedOn(void);
static void turnModeLedOff(void);
static void turnCoolingFanOn(void);
static void turnCoolingFanOff(void);
static uint16_t readPsuVoltage_mv(void);
static uint16_t readPsuCurrent_ma(void);
static uint16_t readAnnealingTime_ms(void);
static void preloadCase(void);
static void loadCase(void);
static void returnCaseFeederHome(void);
static bool caseFeederStillMoving(void);

/*---------------------------------------------------------------------------*/
/*! @brief      Initialize the Case Annealer.
  @details      None.
  @param        None.
  @return       None.
*//*-------------------------------------------------------------------------*/
void setup()
{
  //TCCR0B = TCCR0B & B11111000 | B00000101; //PWM on D5 & D6 set to 61.04Hz Timer 0 -- Timer Used for system ms tick
 // TCCR2B = TCCR2B & B11111000 | B00000110; //PWM on D3 & D11 set to 122.55Hz Timer 2  <--- tiner 2
  TCCR1B = TCCR1B & B11111000 | B00000101; //PWM on D9 & D10 of 30.64 Hz Timer 1   <---- USE IO9 PWM for drop gate Servo

//set timer2 interrupt
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register - divide by microsteps to shorten step period
  OCR2A = 170 / STEPPER_MICROSTEPS; //250
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS20-22 bit for prescaler
  TCCR2B |= (1 << CS22);   
  TCCR2B |= (1 << CS21); 

  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  // Setup IO.
  pinMode(g_StartStopButtonPin, INPUT_PULLUP);
  pinMode(g_ModeButtonPin, INPUT_PULLUP);
  pinMode(g_TimeSetButtonPin, INPUT_PULLUP);
  pinMode(g_AnnealerPin, OUTPUT);
  pinMode(g_StartStopLedPin, OUTPUT);
  pinMode(g_ModeLedPin, OUTPUT);
  pinMode(g_CoolingFanPin, OUTPUT);
  pinMode(g_DropSolenoidPin, OUTPUT);
  pinMode(g_DropServoPin,OUTPUT);
  pinMode(g_FeederStepPin,OUTPUT);
  pinMode(g_FeederDirPin,OUTPUT);
  pinMode(g_FeederStepperEnPin,OUTPUT);
  digitalWrite(g_FeederDirPin,HIGH);
  digitalWrite(g_FeederStepperEnPin,HIGH); //disable stepper driver
  closeDropGate();
  turnStartStopLedOff();  
  turnModeLedOff();
  turnAnnealerOff();     
  turnCoolingFanOff();
  closeDropGate(); 
  #ifdef DEBUG
  Serial.begin(9600);
  delay(20);
  Serial.println("Debug active.");
  #endif

  delay(200);
  
  display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS);
  
  display.clearDisplay();
  // Setup text and draw splash screen
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  
  #ifndef DEBUG //dont do the splash startup in debug
    display.drawBitmap(0, 0,  anneallogo, 128, 32, 1);
    display.display();
    delay(2000);
    display.clearDisplay(); 
    display.drawBitmap(0, 0,  anneallogo2, 128, 32, 1);
    display.display();
    delay(2000);

    for(uint8_t i = 0; i <= 20; i++)
    {
      display.clearDisplay(); 
      display.drawBitmap(0, 0,  projectile, 128, 32, 1);
      display.display();
      delay(100);
      display.clearDisplay(); 
      display.drawBitmap(0, 0,  projectile2, 128, 32, 1);
      display.display();
      delay(100);
    }
  #else
    delay(2000);
  #endif
  display.clearDisplay(); 
  //Setup temp sensor and read 1-wire address. initiate first temp reading
  sensors.begin();

  NumberDallasTempDevices = sensors.getDeviceCount(); //see how many temp sensors are on the 1-wire
  for(uint8_t i=0; i<16; i++)
  {
    psuCurrentZeroOffset += analogRead(g_PsuCurrentAdcPin);
    delay(10);
  }
  psuCurrentZeroOffset = psuCurrentZeroOffset >> 4; //divide by 16
  if( psuCurrentZeroOffset > 200) //see if there is a sensor on the ADC pin. should be mid-rail with no current
  {
    CurrentSensorPresent = 1;
  }

  #ifdef DEBUG
  Serial.print("Software Version : ");
  Serial.println(SOFTWARE_VERSION);
  Serial.print("Number of Dallas temp sensors found : ");
  Serial.println(sensors.getDeviceCount());
  Serial.print("Current sensors found : ");
  Serial.println(CurrentSensorPresent);
  Serial.print("Drop gate control : ");
  #ifdef SERVO
    Serial.println("Servo");
  #else
    Serial.println("Solenoid");
  #endif
  Serial.print("PSU Current zero offset : ");
  Serial.println(psuCurrentZeroOffset);
  Serial.print("\n\n\n");
  #endif

  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, TEMP_RESOLUTION);
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false);
  //setup the watchdog timer. it needs a boot every 500ms.
  wdt_enable(WDTO_500MS);
}
/*---------------------------------------------------------------------------*/
/*! @brief      Timer2 ISR
  @details      None.
  @param        None.
  @return       Never.
*//*-------------------------------------------------------------------------*/

ISR(TIMER2_COMPA_vect){//timer2 interrupt 
  if(StepsToGo)
	  {
	  if (StepToggle)
	  {
	    digitalWrite(g_FeederStepPin,HIGH);
	    StepToggle = 0;
	    if(StepsFromHome + 1 >= STEPPER_STEPS_PER_TURN)
		  {
		  	StepsFromHome = 0;
		  }
		  else
		  {
		  	StepsFromHome = StepsFromHome + 1;
		  }
      if(StepsFromHome < CASE_FEEDER_HOPPER_START) //move feed wheel quickly to pick the next case
      {
          // set compare match register - divide by microsteps to shorten step period

          OCR2A = 120 / STEPPER_MICROSTEPS;

      }
      else if(StepsFromHome < CASE_FEEDER_HOPPER_END) //slow down the feed wheel while picking the case for more reliable pickups
      {
          // set compare match register - divide by microsteps to shorten step period
          #ifdef STEPPER_MICROSTEPS >= 4 //check we arent going to overflow the 8 bit timer register
            OCR2A = 800 / STEPPER_MICROSTEPS;
          #else
            OCR2A = 254;
          #endif
      }
      else //speed up again once new case is picked
      {
        // set compare match register - divide by microsteps to shorten step period
          OCR2A = 170 / STEPPER_MICROSTEPS;
      }
		StepsToGo = StepsToGo - 1;
	  }
	  else{
	    digitalWrite(g_FeederStepPin,LOW);
	    StepToggle = 1;
	  }
	  
	  
  }
  else
  {
  	digitalWrite(g_FeederStepPin,LOW);
  	StepToggle = 1;
  }

}

/*---------------------------------------------------------------------------*/
/*! @brief      Main Loop.
  @details      None.
  @param        None.
  @return       Never.
*//*-------------------------------------------------------------------------*/
void loop()
{
  static bool isSerialInterface = false;
  static bool start;
  static bool startPrev;
  
  static bool modeKey;
  static bool modeKeyPrev;
  static bool upKey=0;
  static bool upKeyPrev=0;
  static uint8_t upKeyDuration = 0x00;
  static uint8_t modeKeyDuration = 0x00;
  static bool FanIsOn = false;
  static bool annealTimeChanged = false;
  static uint16_t psuVoltage_mv;
  static uint16_t psuCurrent_ma;
  static uint16_t AnnealTime_ms = EEPROM.read(0)*100; //reload last used anneal time
  static uint32_t cooling_timer = 0;
  static uint32_t SystemTimeTarget;
  static uint32_t LoopStartTime;
  static float temperature = 0;
  static float temperaturePrev = 0;
  static bool Just_Booted = 1;

  //boot the watchdog
  wdt_reset();
  //read keys
  LoopStartTime = millis(); // capture time when loop starts
  start = readStartButton();
  modeKey = readModeButton();
  upKey = readUpButton();
   
  if (start && !startPrev) //Start key pressed?
  {
    if (g_SystemState == STATE_STOPPED)
    { 
      if(Just_Booted) //Show the warning screen to set the right case heigth and time 1st time
      {
        updateSystemState(STATE_SHOW_WARNING);
        Just_Booted = 0;
      }
      else
      {
        updateSystemState(STATE_ANNEALING);
      }
      
    }
    else if (g_SystemState != STATE_COOLDOWN) //confirm it's not in cooldown mode
    { 
      closeDropGate();
      if(CurrentMode == MODE_AUTOMATIC)
      {
      	returnCaseFeederHome();
      }
      updateSystemState(STATE_STOPPED);
    }
  }

  if(modeKey == 0)
  {
    #ifdef MODE_KEY_USED
    	modeKeyDuration = 0;
    #else
	    if(modeKeyPrev)
	    {
	    	closeDropGate();
	    }
    #endif
  }
  else
  {
  	#ifdef MODE_KEY_USED
	    if (!modeKeyPrev) //mode key just pressed?
	    {
	      if (g_SystemState == STATE_SHOW_SOFTWARE_VER | g_SystemState == STATE_OVERCURRENT_WARNING)
	      {
	        updateSystemState(STATE_STOPPED);
	      }
        else if (g_SystemState == STATE_STOPPED | g_SystemState == STATE_JUST_BOOTED)
        {
  	      if(CurrentMode == MODE_SINGLE_SHOT)
  	      {
  	        CurrentMode = MODE_FREE_RUN;
            digitalWrite(g_FeederStepperEnPin,HIGH); //disable stepper driver in free run mode
            turnModeLedOn();
  	      }
  	      else if(CurrentMode == MODE_FREE_RUN)
  	      {
            CurrentMode = MODE_AUTOMATIC;
            digitalWrite(g_FeederStepperEnPin,LOW); //enable stepper driver in auto mode
  	        turnModeLedOn();
  	      }
          else
          {
            CurrentMode = MODE_SINGLE_SHOT;
            digitalWrite(g_FeederStepperEnPin,HIGH); //disable stepper driver in single shot mode
            turnModeLedOff();
          }
        }
      }
	    if(g_SystemState == STATE_STOPPED | g_SystemState == STATE_JUST_BOOTED)
        {
          modeKeyDuration = modeKeyDuration + 1;
          if (modeKeyDuration >= LONG_PRESS_HOLD_TIME) //long press
            {
              updateSystemState(STATE_SHOW_SOFTWARE_VER);
              modeKeyDuration = 0;
            }
        }

    #else
      	openDropGate();
    #endif
  }


  switch (g_SystemState) //State machine. spend as little time in here as possible.
  {
    case STATE_STOPPED:
    {
      updateSystemState(g_SystemState);
      if(upKey == 0)
      {
        upKeyDuration = 0;
      }
      else
      {
        upKeyDuration = upKeyDuration + 1;
      }
      if(AnnealTime_ms > MAX_ANNEAL_TIME) // too long
      {
        AnnealTime_ms = MIN_ANNEAL_TIME;
        annealTimeChanged = true;
      }
      if (upKey && !upKeyPrev) //up key pressed?
      {
        AnnealTime_ms = AnnealTime_ms + 100;
        annealTimeChanged = true;
      }
      if (upKeyDuration >= LONG_PRESS_HOLD_TIME) //long press resets time to 2s
      {
        AnnealTime_ms = MIN_ANNEAL_TIME;
        upKeyDuration = 0;
        annealTimeChanged = true;
      }

      if (millis() > SystemTimeTarget) //wait for conversion time
      {
        SystemTimeTarget = millis() + 3000;
        temperature = sensors.getTempCByIndex(0);
        sensors.requestTemperatures();
      }
      
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("TIME");
      
      if(FanIsOn)
      {
         display.setCursor(70,0);
         display.setTextSize(1);
         display.println("FAN ON");
         display.setTextSize(2);
         /*display.setCursor(108,8);
         display.println("ON");
         display.setTextSize(2);*/
      }
      else
      {
        display.println(" ");
      }
      display.setCursor(0,16);
      display.print(AnnealTime_ms/1000, DEC); 
      display.print("."); 
      display.print((AnnealTime_ms%1000)/100, DEC);
      display.print("s"); 
      display.setCursor(70,24);
      if(NumberDallasTempDevices != 0)
      {
        display.setTextSize(1);
        display.print(temperature, 1); 
        display.print((char)248);
        display.print("C");
        display.setTextSize(2);
      }
      
      if(CurrentMode == MODE_FREE_RUN)
      {
         display.setCursor(70,8);
         display.setTextSize(1);
         display.println("FREE RUN");
         display.setTextSize(2);
      }
      else if(CurrentMode == MODE_AUTOMATIC)
      {
         display.setCursor(70,8);
         display.setTextSize(1);
         display.println("AUTO FEED");
         display.setTextSize(2);
      }
      else if(CurrentMode == MODE_SINGLE_SHOT)
      {
         display.setCursor(70,8);
         display.setTextSize(1);
         display.println("ONE SHOT");
         display.setTextSize(2);
      }
      display.drawLine(64,0,64,32,WHITE);
      display.display();
      turnStartStopLedOff();  
      turnAnnealerOff();     
      //closeDropGate();
      psuCurrent_ma = readPsuCurrent_ma(); //--------- added this
      updateSystemState(STATE_STOPPED);
    }
    break;
    
    case STATE_ANNEALING:
    {
      if (hasSystemStateChanged())
      {
        SystemTimeTarget = millis() + AnnealTime_ms;
          if(CurrentMode == MODE_AUTOMATIC)
          {
            preloadCase();
          }
      }
      updateSystemState(g_SystemState);

      psuCurrent_ma = readPsuCurrent_ma();
      if(CurrentSensorPresent)
      {
        if(psuCurrent_ma >= PSU_OVERCURRENT) //overloaded the PSU - may damage the ZVS converter 
        {
          turnAnnealerOff();
          turnStartStopLedOff();
          updateSystemState(STATE_OVERCURRENT_WARNING);
          break;
        }
      }
      
      if((SystemTimeTarget - millis()) < 100000)
      {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("ANNEALING");
        if(CurrentSensorPresent)
        {
          display.print(psuCurrent_ma/1000,DEC); 
          display.print("."); 
          display.print((psuCurrent_ma%1000)/100, DEC);
          display.print("A  "); 
        }
        display.print((SystemTimeTarget - millis())/1000, DEC); 
        display.print("."); 
        display.print(((SystemTimeTarget - millis())%1000)/100, DEC); 
        display.print("s");
        display.display();
      }

      turnStartStopLedOn();
      turnAnnealerOn();
      cooling_timer = COOLDOWN_PERIOD + millis(); // 5 minute cooldown after last anneal
      if (millis() < SystemTimeTarget)
      {
        break;
      }
      
      turnAnnealerOff();
      openDropGate();
      updateSystemState(STATE_DROPPING);
    }
    break;    

    case STATE_DROPPING:
    {
      if (hasSystemStateChanged())
      {
        if(caseFeederStillMoving()) //case feeder is still moving so wait until it's finished moving before starting the drop sequence
        {
          break;
        }
        SystemTimeTarget = millis() + DROP_TIME;
        sensors.requestTemperatures(); //this will take time. stick to 9 & 10 bit conversions.
      }      
      updateSystemState(g_SystemState);

      if (millis() < SystemTimeTarget) // wait time is not up, break.
      {
        display.clearDisplay();
        display.setCursor(15, 8);
        display.println("DROPPING");
        display.display();
        break;
      }
      temperaturePrev = temperature;
      temperature = sensors.getTempCByIndex(0);
      closeDropGate();

      if((temperature > TEMP_LIMIT) && (temperaturePrev > TEMP_LIMIT))
      {
        updateSystemState(STATE_COOLDOWN); //Too hot, go to cooldown state
      }
      else if(CurrentMode == MODE_SINGLE_SHOT) //modestate bit will determine if we free run or go to stopped state
      {
        updateSystemState(STATE_STOPPED);
      }
      else
      {
        updateSystemState(STATE_RELOADING);
      }
    }
    break;

    case STATE_RELOADING:
    {
      if (hasSystemStateChanged())
      {
        SystemTimeTarget = millis() + RELOAD_TIME; //load time to fit new case
        if(CurrentMode == MODE_AUTOMATIC)
        	{
        		loadCase();
            SystemTimeTarget = millis() + RELOAD_TIME_AUTO__FEED; //load time when in auto feed mode
        	}
      }      
      updateSystemState(g_SystemState);
      

      if (millis() < SystemTimeTarget)
      {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("LOADING");
        display.print((SystemTimeTarget - millis())/1000, DEC); 
        display.print("."); 
        display.print(((SystemTimeTarget - millis())%1000)/100, DEC); 
        display.print("s"); 
        display.display();
        break;
      }
      updateSystemState(STATE_ANNEALING);
    }
    break;

    case STATE_SHOW_WARNING:
    {   
      updateSystemState(g_SystemState);

      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("TIME"); 
      display.setTextSize(1);
      display.print(" & ");
      display.setTextSize(2);
      display.println("CASE");
      display.println("HEIGHT OK?"); 
      display.display();

    }
    break;

    case STATE_OVERCURRENT_WARNING:
    {   
      updateSystemState(g_SystemState);

      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("! FAULT !"); 
      display.println("CHECK COIL"); 
      display.display();

    }
    break;

    case STATE_SHOW_SOFTWARE_VER:
    {   
      updateSystemState(g_SystemState);
      display.setTextSize(1);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("SW VER : "); 
      display.println(SOFTWARE_VERSION); 
      display.print("Temp sensor : ");
      display.println(sensors.getDeviceCount());
      display.print("Current sensor : ");
      display.println(CurrentSensorPresent); 
      display.setTextSize(2);
      display.display();
    }
    break;

    case STATE_COOLDOWN:
    {
      if (hasSystemStateChanged())
      {
        SystemTimeTarget = millis() + TEMP_CONVERSION_TIME; //time for temp conversion
        turnStartStopLedOff(); 
      }      
      updateSystemState(g_SystemState);
      
      cooling_timer = COOLDOWN_PERIOD + millis(); //keep resetting fan timer while in cooldown mode
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("COOLDOWN: "); 
      display.print(temperature, 1); 
      display.print((char)248);
      display.print("C"); 
      display.display();

      if (millis() < SystemTimeTarget) //wait for conversion time
      {
        break;
      }
      temperaturePrev = temperature;
      temperature = sensors.getTempCByIndex(0);
      sensors.requestTemperatures();
      SystemTimeTarget = millis() + TEMP_CONVERSION_TIME; //time for temp conversion
      if((temperature < (TEMP_LIMIT - TEMP_HYSTERESIS)) && (temperaturePrev < (TEMP_LIMIT - TEMP_HYSTERESIS))) //has it cooled enough to resume?
      {
        updateSystemState(STATE_STOPPED);
      }

    }
    break;
    case STATE_JUST_BOOTED:
    {
      temperature = sensors.getTempCByIndex(0);
      if(temperature> TEMP_LIMIT)
      {
        updateSystemState(STATE_COOLDOWN);
      }
      else
      {
        updateSystemState(STATE_STOPPED);
        SystemTimeTarget = millis() + TEMP_CONVERSION_TIME; //time for temp conversion
      }
    }
    break;
    default:
    {
      updateSystemState(g_SystemState);
      updateSystemState(STATE_STOPPED);
    }
  }
  startPrev = start;  
  modeKeyPrev = modeKey;
  upKeyPrev = upKey;
  
  if(cooling_timer > millis())
  {
    turnCoolingFanOn();
    FanIsOn=true;
  }
  else
  {
    turnCoolingFanOff();
    FanIsOn=false;
  }

  #ifdef DEBUG

  Serial.print("Annealer current : ");
  Serial.print(psuCurrent_ma/1000,DEC); 
  Serial.print(".");
  Serial.print((psuCurrent_ma%1000)/100, DEC);
  Serial.println("A");

  Serial.print("Anneal Time : ");
  Serial.print(AnnealTime_ms);
  Serial.println("ms");

  Serial.print("Loop Time Remaining : ");
  Serial.print(LoopStartTime + LOOP_TIME - millis());
  Serial.println("ms");
  Serial.println(" ");

  Serial.print("Step count : ");
  Serial.print(StepsToGo);
  Serial.println(" ");

  #endif
  
  while(LoopStartTime + LOOP_TIME > millis()) // wait for the loop time to expire
  {
      if((millis() & 0x00003FFF == 0x00003FFF) && (annealTimeChanged == true)) // write to EEPROM every ~16 seconds only if anneal time has changed
      {
        EEPROM.write(0,AnnealTime_ms/100);
        annealTimeChanged = false;
      }
  }

}

/*---------------------------------------------------------------------------*/
/*! @brief      Set system state.
  @param        state: System state.
*//*-------------------------------------------------------------------------*/
static tStateMachineStates updateSystemState(tStateMachineStates const state)
{
  g_SystemStatePrev = g_SystemState;
  g_SystemState = state;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Has system state changed.
  @return       false - system state has not changed. Else true.
*//*-------------------------------------------------------------------------*/
static bool hasSystemStateChanged(void)
{
  if (g_SystemStatePrev == g_SystemState)
  {
    return false;
  }

  return true;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Read the start button state.
  @return       Start button state. 0 = low, else non-zero.
*//*-------------------------------------------------------------------------*/
static bool readStartButton(void)
{
  return !digitalRead(g_StartStopButtonPin);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Read the mode button state.
  @return       Start button state. 0 = low, else non-zero.
*//*-------------------------------------------------------------------------*/
static bool readModeButton(void)
{
  return !digitalRead(g_ModeButtonPin);
}
/*---------------------------------------------------------------------------*/
/*! @brief      Read the up button state.
  @return       Start button state. 0 = low, else non-zero.
*//*-------------------------------------------------------------------------*/
static bool readUpButton(void)
{
  return !digitalRead(A2);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the annealer on.
*//*-------------------------------------------------------------------------*/
static void turnAnnealerOn(void)
{
  digitalWrite(g_AnnealerPin, HIGH);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the annealer off.
*//*-------------------------------------------------------------------------*/
static void turnAnnealerOff(void)
{
  digitalWrite(g_AnnealerPin, LOW);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Open the drop gate.
*//*-------------------------------------------------------------------------*/
static void openDropGate(void)
{

    analogWrite(g_DropServoPin,SERVO_OPEN_POSITION);
    digitalWrite(g_DropSolenoidPin, HIGH);

}

/*---------------------------------------------------------------------------*/
/*! @brief      Close the drop gate.
*//*-------------------------------------------------------------------------*/
static void closeDropGate(void)
{

    analogWrite(g_DropServoPin,SERVO_CLOSE_POSITION); //IO9 PWM output
    digitalWrite(g_DropSolenoidPin, LOW);

}
/*---------------------------------------------------------------------------*/
/*! @brief      Turn the start/stop LED on.
*//*-------------------------------------------------------------------------*/
static void turnStartStopLedOn(void)
{
  digitalWrite(g_StartStopLedPin, HIGH);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the start/stop LED off.
*//*-------------------------------------------------------------------------*/
static void turnStartStopLedOff(void)
{
  digitalWrite(g_StartStopLedPin, LOW);
}
/*---------------------------------------------------------------------------*/
/*! @brief      Turn the start/stop LED on.
*//*-------------------------------------------------------------------------*/
static void turnModeLedOn(void)
{
  digitalWrite(g_ModeLedPin, HIGH);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the start/stop LED off.
*//*-------------------------------------------------------------------------*/
static void turnModeLedOff(void)
{
  digitalWrite(g_ModeLedPin, LOW);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the cooling fan on.
*//*-------------------------------------------------------------------------*/
static void turnCoolingFanOn(void)
{
  digitalWrite(g_CoolingFanPin, HIGH);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the cooling fan off.
*//*-------------------------------------------------------------------------*/
static void turnCoolingFanOff(void)
{
  digitalWrite(g_CoolingFanPin, LOW);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Read the PSU current.
  @return       PSU current in millamps (ma).
*//*-------------------------------------------------------------------------*/
static uint16_t readPsuCurrent_ma(void)
{
  uint16_t adc = 0;

  adc = analogRead(g_PsuCurrentAdcPin);
  adc = abs(adc - psuCurrentZeroOffset)*39; //2.5V offset scale=2.5V/20A
  if(adc > 25000) // error from abs function can return large numbers if ADC measurement goes 
  {
    adc = 0;
  }
  return adc;
}


/*---------------------------------------------------------------------------*/
/*! @brief      rotate case loader to preload position from home
*//*-------------------------------------------------------------------------*/
static void preloadCase(void)
{
	StepsToGo = StepsToGo + CASE_FEEDER_STEPS_DROP_TO_PRELOAD;
	//enableStepperPulses(1);
}

/*---------------------------------------------------------------------------*/
/*! @brief      rotate case loader to drop position
*//*-------------------------------------------------------------------------*/
static void loadCase(void)
{
	StepsToGo = StepsToGo + CASE_FEEDER_STEPS_PRELOAD_TO_DROP; //multiply by 2 for the 2 half cycles counted by the timer interrupt
	//enableStepperPulses(1);
}

/*---------------------------------------------------------------------------*/
/*! @brief      rotate case loader to home/park position from anywhere
*//*-------------------------------------------------------------------------*/
static void returnCaseFeederHome(void)
{
	if(StepsFromHome)
		{
			StepsToGo = STEPPER_STEPS_PER_TURN - StepsFromHome;
		}
	
}

/*---------------------------------------------------------------------------*/
/*! @brief      is case feeder still moving?
*//*-------------------------------------------------------------------------*/
static bool caseFeederStillMoving(void)
{
  if(StepsToGo)
    {
      return true;
    }
    else
    {
      return false;
    }
  
}