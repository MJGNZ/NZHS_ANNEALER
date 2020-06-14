# NZHS_ANNEALER
Arduino Uno source code for NZHS Annealer Shield

Ref docs available here:
1) PCB assembly guide
https://www.dropbox.com/s/uarh0mn9ruvqi06/Cartridge%20Case%20Annealer%20-%20Electronic%20Controller%20Assembly%20Guide.pdf

2) Annealer build and use guide
https://www.dropbox.com/s/1ccgn1x5fqed7u1/Cartridge%20Case%20Annealer%20-%20Parts%2C%20wiring%20and%20use.pdf


 ADDITION FOR COIL TYPE CHANGES
 
Considerations for changing the induction coil type from a water cooled helical coil, to a ferrite rod core that will be wound with 800 strand Litz wire.

Ist consideration Change minimum start time to 1.2 seconds
2nd consideration Alter "mode" switch to a proximity switch this will "turn on" the start process when  case is detected in the coil
3rd consideration Create a timed delay of 5 seconds ( initially ) to aid with keping the project cooler for sustained use.
