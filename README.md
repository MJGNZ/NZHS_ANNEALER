# NZHS_ANNEALER
Arduino Uno source code for NZHS Annealer Shield

Annealer build and use guide
http://www.mgnz-makes.com/

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
