# NZHS Case Annealer v3.9.0: 128x64 OLED, Arduino Pro Mini / Nano

This is firmware for the NZHS brass cartridge case annealer. The original was by Justin Spence and Mark Griffith (2020); this version is adapted for:

- **Arduino Pro Mini 5V / 16 MHz** (or Arduino Nano), both with the **Optiboot** bootloader
- **SSD1306 0.96" 128x64 I2C OLED**

| File | Contents |
|---|---|
| `NZHS_ANNEALER_128x64_OLED.ino` | Firmware |
| `WIRING.svg` | Wiring diagram. Open it in any web browser. |
| `README.md` | This file |
| `*.ino.standard.hex` | Prebuilt firmware for uploading over serial (Board = Arduino Uno / Optiboot) |
| `*.ino.with_bootloader.standard.hex` | Firmware + Optiboot in one image, for flashing with an ISP programmer |

---

## 1. Parts

| Part | Notes |
|---|---|
| Arduino Pro Mini **5V / 16 MHz** | **Do not use the 3.3V / 8 MHz version.** The servo and stepper timing, and the 5V current sensor, won't work on it. An Arduino Nano also works. |
| USB-serial adapter (FTDI / CP2102 / CH340) | Set to 5V. It must have a **DTR** pin. |
| ISP programmer (USBasp, or a spare Uno/Nano) | Needed once, to burn the bootloader |
| SSD1306 0.96" 128x64 OLED, I2C (4-pin) | Address 0x3C |
| DS18B20 temperature sensor + 4.7 kΩ resistor | Mounted on the ZVS capacitors |
| ACS712-30A current sensor | Optional. Detected automatically at power-up. |
| Drop gate servo (or solenoid + MOSFET) | |
| A4988 / DRV8825 stepper driver + stepper | Auto-feed mode only |
| ZVS induction board + relay/MOSFET driver | |
| Cooling fan + MOSFET/relay | |
| 3 push buttons, 2 LEDs + 2 × 330 Ω | |
| 3 × 10 kΩ resistors | Pull-downs on D6, D7, D10 |
| 12V PSU + 12V→5V buck converter (≥ 2 A) | |

---

## 2. Burn the Optiboot bootloader (once per board)

Pro Mini boards (and many Nano clones) ship with an old bootloader that **locks up after a watchdog reset**. The annealer uses the watchdog as a safety restart, so this step is required.

### 2.1 Check which bootloader is installed

Upload this test sketch. If the Uno setting fails, upload it with *Tools → Board → Arduino Pro or Pro Mini → ATmega328P (5V, 16 MHz)*.

```cpp
#include <avr/wdt.h>

void setup()
{
  MCUSR = 0;
  wdt_disable();
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial.println(F("BOOT"));
  for (uint8_t i = 0; i < 3; i++)   // 3 quick flashes = sketch started
  {
    digitalWrite(13, HIGH); delay(100);
    digitalWrite(13, LOW);  delay(100);
  }
  wdt_enable(WDTO_500MS);           // never reset -> watchdog fires in 0.5s
}

void loop() {}
```

| LED on pin 13 | Serial Monitor (9600) | Result |
|---|---|---|
| 3 blinks, pause, repeating forever | `BOOT` printed every cycle | **Optiboot, OK.** Go to step 3. |
| 3 blinks, pause, then **nonstop fast flicker** | `BOOT` printed once | **Old bootloader.** Do step 2.2. |

To get out of the flicker loop, unplug the power.

### 2.2 Burn Optiboot

1. Open *File → Examples → 11.ArduinoISP → ArduinoISP* and upload it to a spare Uno or Nano.
2. Put a **10 µF capacitor** between the spare board's RESET and GND, with the negative leg to GND.
3. Wire the spare board to the Pro Mini. Disconnect the USB-serial adapter from the Pro Mini first.

   | Spare Uno/Nano | Pro Mini |
   |---|---|
   | D13 | 13 |
   | D12 | 12 |
   | D11 | 11 |
   | D10 | RST |
   | 5V | VCC |
   | GND | GND |

4. *Tools → Board →* **Arduino Uno**
5. *Tools → Port →* the spare board's COM port
6. *Tools → Programmer →* **Arduino as ISP** (choose **USBasp** if you're using one)
7. *Tools → **Burn Bootloader*** and wait for "Done burning bootloader".
8. Run the test sketch from 2.1 again with **Board = Arduino Uno**. You should now see 3 blinks and a pause, repeating.

From now on, **always upload to this board with Board = Arduino Uno.** You also get 32,256 bytes of program space instead of 30,720.

---

## 3. Install the libraries and upload

1. Install these through *Sketch → Include Library → Manage Libraries*:
   - **Adafruit SSD1306**, which also asks to install Adafruit GFX and Adafruit BusIO. Accept.
   - **OneWire**
   - **DallasTemperature**
2. Open `NZHS_ANNEALER_128x64_OLED.ino`. Check that the IDE title shows **128x64**, not the old 128x32 sketch.
3. Connect the USB-serial adapter:

   | Adapter | Pro Mini |
   |---|---|
   | GND | GND |
   | VCC (5V) | VCC |
   | TX | RXI |
   | RX | TXO |
   | DTR | DTR / GRN |

   Power the board from the adapter **or** from the 5V rail, never both at once.

4. *Board = **Arduino Uno***, select the adapter's port, then Upload.
5. Expected result: **about 24,830 bytes** of flash and **475 bytes** of RAM (the display's 1 KB buffer is allocated at runtime). If it reports 24,152 bytes, you uploaded the old 128x32 sketch.

---

## 4. Wiring

See **`WIRING.svg`** for the diagram. The Nano uses the same pin numbers.

| Module | Module pin | Pro Mini pin | Notes |
|---|---|---|---|
| **OLED SSD1306** | VCC / GND | 5V / GND | |
| | SDA | **A4** | Pro Mini: separate inner pad, solder directly |
| | SCL | **A5** | Pro Mini: separate inner pad, solder directly |
| **Start/Stop button** | leg 1 / leg 2 | **D2** / GND | internal pull-up |
| **Mode button** | leg 1 / leg 2 | **D3** / GND | internal pull-up |
| **Time/Up button** | leg 1 / leg 2 | **A2** (D16) / GND | internal pull-up |
| **Start/Stop LED** | anode via 330 Ω | **D4** | cathode to GND |
| **Mode LED** | anode via 330 Ω | **D11** | cathode to GND |
| **DS18B20** | DQ | **D8** | 4.7 kΩ from DQ to 5V. VDD to 5V, GND to GND. |
| **Servo** | signal | **D9** | + to the 5V rail, − to GND |
| **Solenoid** (instead of the servo) | MOSFET gate | **D10** | 10 kΩ gate to GND, flyback diode across the coil |
| **ACS712** | OUT | **A0** | VCC to 5V, GND to GND. IP+/IP− in series with the ZVS 12V supply. |
| **Stepper driver** | STEP | **D12** | |
| | DIR | **D13** | |
| | EN | **D5** | LOW = enabled |
| | VDD / GND | 5V / GND | |
| | VMOT / GND | 12V / GND | 100 µF capacitor across VMOT |
| | MS1 / MS2 / MS3 | 5V | 1/16 microstepping (A4988), matches `STEPPER_MICROSTEPS 16` |
| | RESET ↔ SLEEP | linked | A4988 only |
| **ZVS board** (relay/MOSFET) | IN / gate | **D6** | 10 kΩ pull-down to GND |
| **Cooling fan** (MOSFET/relay) | IN / gate | **D7** | 10 kΩ pull-down to GND |

Free pins: D0/D1 (serial), A1, A3, A6, A7.

### Power

- 12V PSU feeds the ZVS board (through the ACS712), the stepper VMOT and the fan.
- A 12V→5V buck converter (≥ 2 A) feeds the **5V rail**: Pro Mini **VCC**, OLED, servo, ACS712, DS18B20 and the stepper driver's VDD.
- Leave the Pro Mini **RAW** pin unconnected. Its onboard regulator is too small for this load.
- **All grounds must be connected together.**
- Fit the **10 kΩ pull-downs on D6, D7 and D10**. They keep the heater, fan and solenoid off during the ~1 s at power-up before the firmware starts.

---

## 5. Operation

### Buttons

| Button | Action | When | Effect |
|---|---|---|---|
| **Start/Stop** | press | stopped | Start a cycle. The first press after power-up shows the "TIME & CASE HEIGHT OK?" warning; press again to confirm. |
| | press | running | Finish the current case, then stop |
| | **hold 1.5 s** | running | **Abort immediately.** Heater off, gate stays closed. **Remove the case from the coil by hand.** |
| **Mode** | press | stopped | Cycle through ONE SHOT → FREE RUN → AUTO FEED |
| | press | fault / info screen | Clear the screen |
| | hold 1.5 s | stopped | Show the software version and detected sensors |
| **Time/Up** | press | stopped | Anneal time +0.1 s. It wraps from 8.0 s back to 2.0 s. |
| | hold 1.5 s | stopped | Reset the anneal time to 2.0 s |

The anneal time is saved automatically 3 s after the last change.

### Modes

| Mode | Mode LED | Behavior |
|---|---|---|
| ONE SHOT | off | Anneal one case, drop it, stop |
| FREE RUN | on | Anneal, drop, then 5 s to load the next case by hand, and repeat |
| AUTO FEED | on | Anneal, drop, and the stepper feeds the next case, repeating |

### Protections

| Condition | What happens | To recover |
|---|---|---|
| Capacitors > 55 °C after a drop | COOLDOWN screen, fan on | Resumes automatically below 40 °C |
| Current ≥ 12.3 A (if the ACS712 is fitted) | Heater off, "FAULT – CHECK COIL" | Fix the cause, then press Mode |
| Temperature sensor lost (3 bad readings) | Heater off, "FAULT – CHECK TEMP SENSOR" | Fix the sensor, then press Mode. It won't clear while the sensor is still failing. |
| No temperature sensor at power-up | Idle screen shows "NO TEMP SENSOR". **There is no capacitor overheat protection.** | Fit the sensor |
| Display not found | Start LED blinks fast, nothing else runs | Check the OLED wiring and address |
| Firmware hang | The watchdog restarts the board within 0.5 s | Automatic (needs Optiboot) |
| After any anneal | Fan runs for 5 minutes | |

---

## 6. First power-up checklist

**With the ZVS board disconnected:**

- [ ] The splash screens appear, then the idle screen: TIME, mode, CAP TEMP (or NO TEMP SENSOR)
- [ ] Mode button cycles the modes, and the mode LED follows
- [ ] Up button changes the time, and holding it resets to 2.0 s
- [ ] Start shows the warning screen. Start again returns to idle. Start again runs a cycle: the start LED lights, the countdown shows, the servo gate opens and closes.
- [ ] **Check that the servo gate positions are correct.** Adjust `SERVO_OPEN_PULSE_US` and `SERVO_CLOSE_PULSE_US` if needed.
- [ ] AUTO FEED: the feed wheel rotates and parks correctly
- [ ] Unplug the DS18B20 while idle. "FAULT – CHECK TEMP SENSOR" appears within about 0.5 s.
- [ ] The fan runs after a cycle

Then connect the ZVS board and test with a single case.

---

## 7. Settings (`#define`s at the top of the sketch)

| Setting | Default | Meaning |
|---|---|---|
| `DISPLAY_ADDRESS` | `0x3C` | Use `0x3D` if the OLED's address jumper is set to 0x7A |
| `CURRENT_SENSOR_SCALE` | `74` | ACS712-30A = 74, ACS712-20A = 49, ACS712-5A = 27 |
| `PSU_OVERCURRENT` | `12300` | Overcurrent trip in mA |
| `TEMP_LIMIT` / `TEMP_HYSTERESIS` | `55` / `15` | Cooldown above 55 °C, resume below 40 °C |
| `MIN_ANNEAL_TIME` / `MAX_ANNEAL_TIME` | `2000` / `8000` | Anneal time range in ms |
| `RELOAD_TIME` | `5000` | Time to load a case by hand in FREE RUN (ms) |
| `DROP_TIME` | `500` | How long the gate stays open (ms) |
| `COOLDOWN_PERIOD` | `300000` | Fan run-on after the last anneal (ms) |
| `SERVO_OPEN_PULSE_US` / `SERVO_CLOSE_PULSE_US` | `640` / `1920` | Servo positions (µs, 50 Hz) |
| `STEPPER_MICROSTEPS` | `16` | Must match the driver's MS1–MS3 setting |
| `MODE_KEY_USED` | defined | Comment it out to make the Mode button a manual "open gate" button instead |
| `SHOW_CASE_COUNT` | defined | Show the number of cases annealed since power-up |
| `DEBUG` | not defined | Uncomment to skip the splash and send debug output at 115200 baud |

---

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Upload fails: `not in sync` / `programmer is not responding` | Wrong board setting. It must be Arduino Uno after Optiboot. Otherwise TX/RX are swapped or DTR isn't connected; press reset just as "Uploading…" appears. |
| Nonstop fast LED flicker after running for a while | Old bootloader. See section 2. |
| Blank OLED, start LED blinking fast | OLED not found. Check SDA=A4, SCL=A5 and the address (0x3C or 0x3D). |
| Blank OLED, no blinking | Power or ground problem, or the old 128x32 sketch was uploaded |
| Display shows only the top half, or looks stretched | The old 128x32 sketch is loaded. Upload the 128x64 one. |
| Temperature shows −127 or a sensor fault | Missing 4.7 kΩ pull-up, or a DQ wiring fault |
| No current (A) shown on the ANNEALING screen | ACS712 not detected at power-up. Its output must be about 2.5V with no load. The info screen (hold Mode) shows "Current sensor : 0". |
| Servo jitters or moves to the wrong place | Servo powered from the Pro Mini instead of the 5V rail, or the pulse widths need adjusting |
| Feed wheel skips or stalls | Stepper driver current limit (Vref) too low, or the microstep setting doesn't match `STEPPER_MICROSTEPS` |

---

## Changes from v3.8.0 (128x32 / Uno)

- **Display:** 128x64 layout for every screen. Centered splash screens. Correct ° symbol. Startup fault if the display can't start.
- **Bug fixes:**
  - Stale stop request after a fault or info screen
  - `updateSystemState()` undefined behavior
  - Broken EEPROM save timing
  - Current sensor not reading when current flows the reverse way
  - Anneal time range checking
- **Safety:**
  - Watchdog disabled at start-up
  - Temperature sensor fault detection
  - Values shared with the stepper interrupt protected from partial updates
  - Timing safe across the `millis()` rollover
  - Hold-Start abort
- **Behavior:**
  - Stopping during the load phase no longer anneals one more case
  - The servo runs at a proper 50 Hz with 0.5 µs resolution; positions are unchanged
