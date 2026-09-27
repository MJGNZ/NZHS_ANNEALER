# NZHS_ANNEALER
Arduino Uno source code for NZHS Annealer Shield

Annealer build and use guide
http://www.mgnz-makes.com/

| Folder | Display | Board |
|---|---|---|
| `NZHS_ANNEALER_128x64_OLED` | SSD1306 0.96" 128x64 | Arduino Pro Mini 5V/16MHz or Nano (Optiboot) — see its README.md and WIRING.svg |
| `NZHS_ANNEALER_128x32_OLED` | SSD1306 128x32 | Arduino Uno (original) |

SW version 3.9.0 (NZHS_ANNEALER_128x64_OLED)
- 128x64 SSD1306 0.96" OLED support with new screen layouts
- Arduino Pro Mini 5V/16MHz / Nano support (requires Optiboot bootloader for the watchdog)
- Temperature sensor fault detection - stops heating if the DS18B20 is lost
- Press and hold Start to abort a cycle immediately
- Fixed current measurement reading zero when current flows the reverse way through the ACS712
- Fixed stale stop request after fault/info screens, EEPROM save timing, undefined return in updateSystemState
- Stepper ISR shared variables made volatile/atomic, millis() rollover safe timing
- Servo drive changed to 50Hz with 0.5us resolution (same gate positions)
- Default current sensor set to ACS712-30A (CURRENT_SENSOR_SCALE 74)

SW version 3.8.0
- Fixed bug relating to case feeder home position drift

SW version 3.7.0
- Added case counter feature. Can be enabled/disabled
- changed OLED I2C clock speed to improve main loop speed

SW version 3.6.0
- Utilise CURRENT_SENSOR_SCALE define in current measurement function

SW version 3.5.0
- bug fix for scaling error in current measurement function

SW version 3.4.0
- Merged changes from stro to reduce RAM usage for fixed strings

SW version 3.3.0
- Bug fix for stepper interrupt duration
- Improvement to Anneal timing precision and repeatabilty

SW version 3.2.0
- Resolved bugs relating to stepper motor drifting after multiple cases
- UI cleanup

SW version v3.0.0
- Support for REV C hardware with stepper motor control for automatic case feeder
- v3.0.0 software is backward compatible with REV A & REV B hardware

SW version v2.1.0
- Added provision for reassigning the mode key as a case dump button for stuck cases. Unit runs in free run mode if this option is selected.
