/*---------------------------------------------------------------------------*/
/*! @brief      Brass Cartridge Case Annealer.
  @details      None.
  @author       Justin Spence, Mark Griffith. 2020
  @note         circuitworksnz@gmail.com
  @note         3.9.0 - 128x64 SSD1306 0.96" OLED, Arduino Nano / Pro Mini 5V 16MHz (ATmega328P).
                Requires Optiboot bootloader (burn "Arduino Uno" bootloader) for watchdog reset to work.
*//*-------------------------------------------------------------------------*/

//--Includes-------------------------------------------------------------------
#include <Wire.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//-- macros---------------------------------------------------------------
//#define DEBUG //defining DEBUG will remove the splash screen and enable serial debug info
#define SERVO
//                          Major Version
//                          | Minor Version
//                          | | LCD Type
//                          | | |
//                          | | |
//                          | | |
#define SOFTWARE_VERSION F("3.9.0")
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels (0.96" SSD1306)
#define PSU_OVERCURRENT 12300 //12.3A
#define CURRENT_SENSOR_SCALE 74  //Choose the current scaling factor to suit your sensor. ACS712-20A = 49, ACS712-30A = 74 (set for 30A device)
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
#define EEPROM_SAVE_DELAY 3000 //save anneal time to EEPROM this long (ms) after the last change
#define TEMP_SENSOR_FAULT_READS 3 //consecutive bad temp readings before a sensor fault is raised
#define DISPLAY_ADDRESS 0x3C //most 0.96" modules are 0x3C, use 0x3D if the address jumper is set to 0x7A
#define SERVO_OPEN_PULSE_US 640   //drop gate servo open pulse width in us (0.5us resolution, 50Hz)
#define SERVO_CLOSE_PULSE_US 1920 //drop gate servo closed pulse width in us
#define STEPPER_SCALING_FACTOR 1 //1.28 //Used to compensate for BIGTREETECH controllers needing 256 steps per rev
#define STEPPER_STEPS_PER_TURN 200*STEPPER_SCALING_FACTOR*STEPPER_MICROSTEPS // stepper motor steps per revolution (e.g. 200 step motor) * microsteps.
#define STEPPER_MICROSTEPS 16 // number of microsteps. set to 1 if no microstepping
#define CASE_FEEDER_STEPS_DROP_TO_PRELOAD 185*STEPPER_MICROSTEPS*STEPPER_SCALING_FACTOR
#define CASE_FEEDER_STEPS_PRELOAD_TO_DROP (STEPPER_STEPS_PER_TURN - CASE_FEEDER_STEPS_DROP_TO_PRELOAD + 1)
#define CASE_FEEDER_HOPPER_START 70*STEPPER_MICROSTEPS*STEPPER_SCALING_FACTOR
#define CASE_FEEDER_HOPPER_END 130*STEPPER_MICROSTEPS*STEPPER_SCALING_FACTOR

#define MODE_KEY_USED  //defines the use of the mode key input. comment out this #define to disable mode selection and reassign the mode key input to force case drop in the event of a stuck case
#define SHOW_CASE_COUNT //This enables the display of the total number of cases annealed since powerup. Comment this out if you don't want to see the cases annealed counter.

// temp sensor pin asignment DS1820
#define ONE_WIRE_BUS 8

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 200000, 200000);
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Global variables :(
DeviceAddress tempDeviceAddress;
static uint8_t NumberDallasTempDevices = 0;
static bool CurrentSensorPresent = 0;
static uint16_t psuCurrentZeroOffset = 0;
static bool TempSensorFault = 0;
// shared with the timer2 ISR - access from the main loop inside ATOMIC_BLOCK
static volatile uint16_t StepsToGo = 0;
static volatile uint16_t StepsFromHome = 0;
static bool StepToggle = 0;
static volatile uint32_t SystemTimeTarget;

//--define state machine states-----------------------------------------------------------
typedef enum tStateMachineStates
{
  STATE_STOPPED = 0,
  STATE_PRELOAD,
  STATE_ANNEALING,
  STATE_DROPPING,
  STATE_RELOADING,
  STATE_COOLDOWN,
  STATE_JUST_BOOTED,
  STATE_SHOW_WARNING,
  STATE_SHOW_SOFTWARE_VER,
  STATE_OVERCURRENT_WARNING,
  STATE_TEMP_SENSOR_FAULT,
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
static volatile tStateMachineStates g_SystemState = STATE_JUST_BOOTED; //read by the timer2 ISR
static tStateMachineStates g_SystemStatePrev = STATE_UNKNOWN;

#ifdef MODE_KEY_USED
  static ModeList CurrentMode = MODE_SINGLE_SHOT; //mode key is used so set default mode to single shot
#else
  static ModeList CurrentMode = MODE_FREE_RUN; //mode key is not used so set default mode to free run
#endif

//-- function declarations------------------------------------------------
static void updateSystemState(tStateMachineStates const state);
static bool hasSystemStateChanged(void);
static bool timeReached(uint32_t const target);
static void setSystemTimeTarget(uint32_t const target);
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
static uint16_t readPsuCurrent_ma(void);
static void preloadCase(void);
static void loadCase(void);
static void returnCaseFeederHome(void);
static bool caseFeederStillMoving(void);
static float readTemperature(uint8_t);

/*---------------------------------------------------------------------------*/
/*! @brief      Initialize the Case Annealer.
  @details      None.
  @param        None.
  @return       None.
*//*-------------------------------------------------------------------------*/
void setup()
{
  //after a watchdog reset the watchdog stays enabled with a 15ms timeout - turn it off before the splash delays
  MCUSR = 0;
  wdt_disable();

  //Timer1 : 16 bit fast PWM (mode 14), prescaler 8, TOP = ICR1 => 50Hz servo frame, 0.5us per count on D9 (OC1A)
  //D9 is driven by writing OCR1A directly - do not use analogWrite() on D9
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 39999;

//set timer2 interrupt
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register - divide by microsteps to shorten step period
  OCR2A = 170 / STEPPER_MICROSTEPS;
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
  digitalWrite(g_FeederStepperEnPin,LOW); //enable stepper driver (holds the feed wheel) during startup
  closeDropGate();
  turnStartStopLedOff();
  turnModeLedOff();
  turnAnnealerOff();
  turnCoolingFanOff();
  closeDropGate();
  #ifdef DEBUG
  Serial.begin(115200);
  delay(20);
  Serial.println(F("Debug active."));
  #endif

  delay(200);

  //the 128x64 frame buffer (1024 bytes) is allocated here - begin() fails if there is not enough RAM left
  if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS))
  {
    #ifdef DEBUG
    Serial.println(F("SSD1306 allocation failed"));
    #endif
    for(;;) //no display - annealer stays off. fast blink the start LED to show the fault
    {
      turnStartStopLedOn();
      delay(100);
      turnStartStopLedOff();
      delay(100);
    }
  }

  display.clearDisplay();
  // Setup text and draw splash screen
  display.cp437(true); //use the correct code page 437 character map so (char)248 is the degree symbol
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  #ifndef DEBUG //dont do the splash startup in debug
    //splash images are 128x32 - draw them in the middle of the 128x64 display
    display.drawBitmap(0, 16,  anneallogo, 128, 32, 1);
    display.display();
    delay(2000);
    display.clearDisplay();
    display.drawBitmap(0, 16,  anneallogo2, 128, 32, 1);
    display.display();
    delay(2000);

    for(uint8_t i = 0; i <= 20; i++)
    {
      display.clearDisplay();
      display.drawBitmap(0, 16,  projectile, 128, 32, 1);
      display.display();
      delay(100);
      display.clearDisplay();
      display.drawBitmap(0, 16,  projectile2, 128, 32, 1);
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
  Serial.print(F("Software Version : "));
  Serial.println(SOFTWARE_VERSION);
  Serial.print(F("Number of Dallas temp sensors found : "));
  Serial.println(sensors.getDeviceCount());
  Serial.print(F("Current sensors found : "));
  Serial.println(CurrentSensorPresent);
  Serial.print(F("Drop gate control : "));
  #ifdef SERVO
    Serial.println(F("Servo"));
  #else
    Serial.println(F("Solenoid"));
  #endif
  Serial.print(F("PSU Current zero offset : "));
  Serial.println(psuCurrentZeroOffset);
  Serial.print(F("\n\n\n"));
  #endif

  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, TEMP_RESOLUTION);
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false);
  delay(TEMP_CONVERSION_TIME); // let the first temp read happen
  //setup the watchdog timer. it needs a boot every 500ms.
  wdt_enable(WDTO_500MS);
  digitalWrite(g_FeederStepperEnPin,HIGH); //disable stepper driver
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
          #if STEPPER_MICROSTEPS >= 4 //check we arent going to overflow the 8 bit timer register
            OCR2A = 120 / STEPPER_MICROSTEPS;
          #else
            OCR2A = 170;
          #endif

      }
      else if(StepsFromHome < CASE_FEEDER_HOPPER_END) //slow down the feed wheel while picking the case for more reliable pickups
      {
          // set compare match register - divide by microsteps to shorten step period
          #if STEPPER_MICROSTEPS >= 4 //check we arent going to overflow the 8 bit timer register
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

  if ((g_SystemState == STATE_ANNEALING) && timeReached(SystemTimeTarget)) //backup annealer off, independent of the main loop
  {
    turnAnnealerOff();
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
  static bool start;
  static bool startPrev;

  static bool modeKey;
  static bool modeKeyPrev;
  static bool upKey=0;
  static bool upKeyPrev=0;
  static uint8_t upKeyDuration = 0x00;
  static uint8_t modeKeyDuration = 0x00;
  static uint8_t startKeyDuration = 0x00;
  static bool FanIsOn = false;
  static bool annealTimeChanged = false;
  static uint32_t annealTimeChangedAt = 0;
  static uint16_t psuCurrent_ma;
  static uint16_t AnnealTime_ms = EEPROM.read(0)*100; //reload last used anneal time (range checked in STATE_STOPPED)
  static uint32_t cooling_timer = 0;
  static bool coolingActive = false;
  static uint32_t LoopStartTime;
  static float temperature = 0;
  static uint8_t tempBadReads = 0;
  static bool Just_Booted = 1;
  static bool Next_Cycle_Is_STOPPED = 0;
  static uint16_t CasesAnnealed = 0;

  //boot the watchdog
  wdt_reset();
  //read keys
  LoopStartTime = millis(); // capture time when loop starts
  start = readStartButton();
  modeKey = readModeButton();
  upKey = readUpButton();

  if(NumberDallasTempDevices != 0)
  {
    float t = readTemperature(0);
    if((t == DEVICE_DISCONNECTED_C) || (t == 85.0)) // -127 = sensor lost, 85.0 = sensor power-on reset value
    {
      if(tempBadReads < TEMP_SENSOR_FAULT_READS)
      {
        tempBadReads++;
      }
      else
      {
        TempSensorFault = 1; //capacitor over temperature protection is not working
      }
    }
    else
    {
      tempBadReads = 0;
      TempSensorFault = 0;
      temperature = t;
    }
  }

  if(TempSensorFault && (g_SystemState != STATE_TEMP_SENSOR_FAULT) && (g_SystemState != STATE_DROPPING)) //let a drop in progress finish
  {
    turnAnnealerOff();
    turnStartStopLedOff();
    if(CurrentMode == MODE_AUTOMATIC)
    {
      returnCaseFeederHome();
    }
    Next_Cycle_Is_STOPPED = 0;
    updateSystemState(STATE_TEMP_SENSOR_FAULT);
  }

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
        Next_Cycle_Is_STOPPED = 0;
        updateSystemState(STATE_ANNEALING); //auto feed mode preloads the next case from STATE_ANNEALING
      }
    }
    else if(g_SystemState == STATE_SHOW_WARNING)
    {
      updateSystemState(STATE_STOPPED);
    }
    else if (g_SystemState == STATE_PRELOAD || g_SystemState == STATE_ANNEALING ||
             g_SystemState == STATE_DROPPING || g_SystemState == STATE_RELOADING) //only a running cycle can be stopped
    {
      Next_Cycle_Is_STOPPED = 1; //finish the current case then stop
    }
  }

  //press and hold start to abort the cycle immediately. the case is not dropped - remove it from the coil by hand
  if(start && (g_SystemState == STATE_PRELOAD || g_SystemState == STATE_ANNEALING || g_SystemState == STATE_RELOADING))
  {
    startKeyDuration = startKeyDuration + 1;
    if(startKeyDuration >= LONG_PRESS_HOLD_TIME)
    {
      turnAnnealerOff();
      turnStartStopLedOff();
      closeDropGate();
      if(CurrentMode == MODE_AUTOMATIC)
      {
        returnCaseFeederHome();
      }
      Next_Cycle_Is_STOPPED = 0;
      startKeyDuration = 0;
      updateSystemState(STATE_STOPPED);
    }
  }
  else
  {
    startKeyDuration = 0;
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
	      if (g_SystemState == STATE_SHOW_SOFTWARE_VER || g_SystemState == STATE_OVERCURRENT_WARNING ||
	          (g_SystemState == STATE_TEMP_SENSOR_FAULT && !TempSensorFault)) //temp fault only clears once the sensor reads again
	      {
	        updateSystemState(STATE_STOPPED);
	      }
        else if (g_SystemState == STATE_STOPPED || g_SystemState == STATE_JUST_BOOTED)
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
	    if(g_SystemState == STATE_STOPPED || g_SystemState == STATE_JUST_BOOTED)
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


  switch (g_SystemState) //State machine.
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
      if (upKey && !upKeyPrev) //up key pressed?
      {
        AnnealTime_ms = AnnealTime_ms + 100;
        annealTimeChanged = true;
        annealTimeChangedAt = millis();
      }
      if (upKeyDuration >= LONG_PRESS_HOLD_TIME) //long press resets time to 2s
      {
        AnnealTime_ms = MIN_ANNEAL_TIME;
        upKeyDuration = 0;
        annealTimeChanged = true;
        annealTimeChangedAt = millis();
      }
      if((AnnealTime_ms > MAX_ANNEAL_TIME) || (AnnealTime_ms < MIN_ANNEAL_TIME)) //out of range (or blank EEPROM) - wrap to min
      {
        AnnealTime_ms = MIN_ANNEAL_TIME;
        annealTimeChanged = true;
        annealTimeChangedAt = millis();
      }

      display.clearDisplay();

      //top left : anneal time
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.print(F("TIME"));
      display.setCursor(0, 16);
      display.print(AnnealTime_ms/1000, DEC);
      display.print(F("."));
      display.print((AnnealTime_ms%1000)/100, DEC);
      display.print(F("s"));

      //top right : mode, fan, case count
      display.setTextSize(1);
      display.setCursor(60, 0);
      if(CurrentMode == MODE_FREE_RUN)
      {
        display.print(F("FREE RUN"));
      }
      else if(CurrentMode == MODE_AUTOMATIC)
      {
        display.print(F("AUTO FEED"));
      }
      else
      {
        display.print(F("ONE SHOT"));
      }
      if(FanIsOn)
      {
        display.setCursor(60, 12);
        display.print(F("FAN ON"));
      }
      #ifdef SHOW_CASE_COUNT
      display.setCursor(60, 24);
      display.print(F("CASES: "));
      display.print(CasesAnnealed, DEC);
      #endif
      display.drawLine(54, 0, 54, 31, WHITE);
      display.drawLine(0, 35, 127, 35, WHITE);

      //bottom : capacitor temperature
      display.setCursor(0, 39);
      if(NumberDallasTempDevices != 0)
      {
        display.print(F("CAP TEMP"));
        display.setTextSize(2);
        display.setCursor(0, 48);
        display.print(temperature, 1);
        display.print((char)248);
        display.print(F("C"));
      }
      else
      {
        display.print(F("NO TEMP SENSOR"));
      }
      display.setTextSize(2);
      display.display();
      turnStartStopLedOff();
      turnAnnealerOff();
      psuCurrent_ma = readPsuCurrent_ma(); //--------- added this
    }
    break;

    case STATE_ANNEALING:
    {
      if (hasSystemStateChanged())
      {
        setSystemTimeTarget(millis() + AnnealTime_ms);
        turnStartStopLedOn();
        turnAnnealerOn();
        cooling_timer = COOLDOWN_PERIOD + millis(); // 5 minute cooldown after last anneal
        coolingActive = true;
          if(CurrentMode == MODE_AUTOMATIC)
          {
            preloadCase();
          }
      }
      updateSystemState(g_SystemState);

      if (timeReached(SystemTimeTarget))
      {
        turnAnnealerOff();
        openDropGate();
        updateSystemState(STATE_DROPPING);
      }

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

      if(!timeReached(SystemTimeTarget))
      {
        uint32_t remaining_ms = SystemTimeTarget - millis();
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print(F("ANNEALING"));
        display.setTextSize(3);
        display.setCursor(0, 20);
        display.print(remaining_ms/1000, DEC);
        display.print(F("."));
        display.print((remaining_ms%1000)/100, DEC);
        display.print(F("s"));
        display.setTextSize(2);
        if(CurrentSensorPresent)
        {
          display.setCursor(0, 48);
          display.print(psuCurrent_ma/1000,DEC);
          display.print(F("."));
          display.print((psuCurrent_ma%1000)/100, DEC);
          display.print(F("A"));
        }
        display.display();
      }


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
        setSystemTimeTarget(millis() + DROP_TIME);
      }
      updateSystemState(g_SystemState);

      if (!timeReached(SystemTimeTarget)) // wait time is not up, break.
      {
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(16, 24);
        display.print(F("DROPPING"));
        display.display();
        break;
      }
      closeDropGate();
      CasesAnnealed++;

      if(temperature > TEMP_LIMIT)
      {
        updateSystemState(STATE_COOLDOWN); //Too hot, go to cooldown state
        if(CurrentMode == MODE_AUTOMATIC)
        {
          returnCaseFeederHome();
        }
      }
      else if(Next_Cycle_Is_STOPPED)
      {
        updateSystemState(STATE_STOPPED);
        Next_Cycle_Is_STOPPED = 0;
        if(CurrentMode == MODE_AUTOMATIC)
        {
          returnCaseFeederHome();
        }
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

    case STATE_PRELOAD:
    {
      if (hasSystemStateChanged())
      {
        preloadCase(); //pick up first case after start button is pressed
      }
      updateSystemState(g_SystemState);
      if(caseFeederStillMoving()) //case feeder is still moving so wait until it's finished moving before starting the drop sequence
        {
          break;
        }
      updateSystemState(STATE_RELOADING);
    }
    break;

    case STATE_RELOADING:
    {
      if (hasSystemStateChanged())
      {
        setSystemTimeTarget(millis() + RELOAD_TIME); //load time to fit new case
        if(CurrentMode == MODE_AUTOMATIC)
        	{
        		loadCase();
            setSystemTimeTarget(millis() + RELOAD_TIME_AUTO__FEED); //load time when in auto feed mode
        	}
      }
      updateSystemState(g_SystemState);


      if (!timeReached(SystemTimeTarget))
      {
        uint32_t remaining_ms = SystemTimeTarget - millis();
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 8);
        #ifdef SHOW_CASE_COUNT
          display.print(F("LOAD"));
        #else
          display.print(F("LOADING"));
        #endif
        display.setCursor(0, 32);
        display.print(remaining_ms/1000, DEC);
        display.print(F("."));
        display.print((remaining_ms%1000)/100, DEC);
        display.print(F("s"));

        #ifdef SHOW_CASE_COUNT
          display.setCursor(65, 8);
          display.print(F("CASES"));
          display.setCursor(65, 32);
          display.print(CasesAnnealed, DEC);
          display.drawLine(57, 0, 57, 63, WHITE);
        #endif

        display.display();
        break;
      }
      if(Next_Cycle_Is_STOPPED) //stop was pressed while loading - don't anneal another case
      {
        Next_Cycle_Is_STOPPED = 0;
        updateSystemState(STATE_STOPPED);
        break;
      }
      updateSystemState(STATE_ANNEALING);
    }
    break;

    case STATE_SHOW_WARNING:
    {
      updateSystemState(g_SystemState);

      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 16);
      display.print(F("TIME"));
      display.setTextSize(1);
      display.print(F(" & "));
      display.setTextSize(2);
      display.println(F("CASE"));
      display.println(F("HEIGHT OK?"));
      display.display();

    }
    break;

    case STATE_OVERCURRENT_WARNING:
    {
      updateSystemState(g_SystemState);

      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 16);
      display.println(F("! FAULT !"));
      display.println(F("CHECK COIL"));
      display.display();

    }
    break;

    case STATE_TEMP_SENSOR_FAULT:
    {
      updateSystemState(g_SystemState);

      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 8);
      display.println(F("! FAULT !"));
      display.println(F("CHECK TEMP"));
      display.setTextSize(1);
      display.setCursor(0, 48);
      display.print(F("SENSOR, press MODE"));
      display.setTextSize(2);
      display.display();

    }
    break;

    case STATE_SHOW_SOFTWARE_VER:
    {
      updateSystemState(g_SystemState);
      display.setTextSize(1);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("SW VER : "));
      display.println(SOFTWARE_VERSION);
      display.print(F("Temp sensor : "));
      display.println(sensors.getDeviceCount());
      display.print(F("Current sensor : "));
      display.println(CurrentSensorPresent);
      display.setTextSize(2);
      display.display();
    }
    break;

    case STATE_COOLDOWN:
    {
      if (hasSystemStateChanged())
      {
        setSystemTimeTarget(millis() + TEMP_CONVERSION_TIME); //time for temp conversion
        turnStartStopLedOff();
      }
      updateSystemState(g_SystemState);

      cooling_timer = COOLDOWN_PERIOD + millis(); //keep resetting fan timer while in cooldown mode
      coolingActive = true;
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 16);
      display.println(F("COOLDOWN: "));
      display.print(temperature, 1);
      display.print((char)248);
      display.print(F("C"));
      display.display();
      if(temperature < (TEMP_LIMIT - TEMP_HYSTERESIS)) //has it cooled enough to resume?
      {
        updateSystemState(STATE_STOPPED);
      }

    }
    break;
    case STATE_JUST_BOOTED:
    {
      //temperature = sensors.getTempCByIndex(0);
      if(temperature> TEMP_LIMIT)
      {
        updateSystemState(STATE_COOLDOWN);
      }
      else
      {
        updateSystemState(STATE_STOPPED);
        setSystemTimeTarget(millis() + TEMP_CONVERSION_TIME); //time for temp conversion
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

  if(coolingActive && timeReached(cooling_timer))
  {
    coolingActive = false;
  }
  if(coolingActive)
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

  Serial.print(F("Annealer current;"));
  Serial.print(psuCurrent_ma/1000,DEC);
  Serial.print(F("."));
  Serial.print((psuCurrent_ma%1000)/100, DEC);
  Serial.print(F(";A;"));

  Serial.print(F("Anneal Time;"));
  Serial.print(AnnealTime_ms);
  Serial.print(F(";ms;"));

  uint16_t stepsToGo;
  uint16_t stepsFromHome;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    stepsToGo = StepsToGo;
    stepsFromHome = StepsFromHome;
  }
  Serial.print(F("Step count;"));
  Serial.print(stepsToGo);
  Serial.print(F(";"));

  Serial.print(F("Steps from home;"));
  Serial.print(stepsFromHome);
  Serial.print(F(";"));

  Serial.print(F("State;"));
  Serial.print(g_SystemState);
  Serial.print(F(";"));

  Serial.print(F("Loop Time Remaining;"));
  Serial.print(LoopStartTime + LOOP_TIME - millis());
  Serial.println(F(";ms;"));


  #endif


  // save the anneal time once the user has stopped changing it. update() only writes if the value differs
  if(annealTimeChanged && ((millis() - annealTimeChangedAt) >= EEPROM_SAVE_DELAY))
  {
    EEPROM.update(0, AnnealTime_ms/100);
    annealTimeChanged = false;
  }

  while(!timeReached(LoopStartTime + LOOP_TIME)) // wait for the loop time to expire
  {
  }

}

/*---------------------------------------------------------------------------*/
/*! @brief      Set system state.
  @param        state: System state.
*//*-------------------------------------------------------------------------*/
static void updateSystemState(tStateMachineStates const state)
{
  g_SystemStatePrev = g_SystemState;
  g_SystemState = state;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Has millis() reached the target time. Safe across the 49 day millis() rollover.
  @param        target: time in ms (millis() + duration).
  @return       true once the target time has been reached.
*//*-------------------------------------------------------------------------*/
static bool timeReached(uint32_t const target)
{
  return (int32_t)(millis() - target) >= 0;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Set SystemTimeTarget. It is 32 bit and read by the timer2 ISR so write it atomically.
  @param        target: time in ms (millis() + duration).
*//*-------------------------------------------------------------------------*/
static void setSystemTimeTarget(uint32_t const target)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    SystemTimeTarget = target;
  }
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
  return !digitalRead(g_TimeSetButtonPin);
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

    OCR1A = SERVO_OPEN_PULSE_US * 2; //IO9 servo pulse, 0.5us per count
    digitalWrite(g_DropSolenoidPin, HIGH);

}

/*---------------------------------------------------------------------------*/
/*! @brief      Close the drop gate.
*//*-------------------------------------------------------------------------*/
static void closeDropGate(void)
{

    OCR1A = SERVO_CLOSE_PULSE_US * 2; //IO9 servo pulse, 0.5us per count
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
  //signed difference from the 2.5V zero offset so current in either direction through the sensor is measured
  int16_t diff = (int16_t)analogRead(g_PsuCurrentAdcPin) - (int16_t)psuCurrentZeroOffset;
  if(diff < 0)
  {
    diff = -diff;
  }
  uint32_t current_ma = (uint32_t)diff * CURRENT_SENSOR_SCALE; //Scaling factor to suit sensor chosen.
  if(current_ma > 0xFFFF)
  {
    current_ma = 0xFFFF;
  }
  return (uint16_t)current_ma;
}


/*---------------------------------------------------------------------------*/
/*! @brief      rotate case loader to preload position from home
*//*-------------------------------------------------------------------------*/
static void preloadCase(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) //StepsToGo is decremented by the timer2 ISR
  {
    StepsToGo = StepsToGo + CASE_FEEDER_STEPS_DROP_TO_PRELOAD;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      rotate case loader to drop position
*//*-------------------------------------------------------------------------*/
static void loadCase(void)
{
	//StepsToGo = StepsToGo + CASE_FEEDER_STEPS_PRELOAD_TO_DROP; //multiply by 2 for the 2 half cycles counted by the timer interrupt
  returnCaseFeederHome();
	//enableStepperPulses(1);
}

/*---------------------------------------------------------------------------*/
/*! @brief      rotate case loader to home/park position from anywhere
*//*-------------------------------------------------------------------------*/
static void returnCaseFeederHome(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) //StepsToGo and StepsFromHome are updated by the timer2 ISR
  {
    if(StepsFromHome)
    {
      StepsToGo = STEPPER_STEPS_PER_TURN - StepsFromHome;
    }
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      is case feeder still moving?
*//*-------------------------------------------------------------------------*/
static bool caseFeederStillMoving(void)
{
  bool moving;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    moving = (StepsToGo != 0);
  }
  return moving;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Read the temperature from the last conversion and start the next one.
*//*-------------------------------------------------------------------------*/
static float readTemperature(uint8_t index)
{
  float t = sensors.getTempCByIndex(index);
  sensors.requestTemperatures(); // this takes quite some time to complete ~90ms or longer. read it on the next loop
  return t;
}
