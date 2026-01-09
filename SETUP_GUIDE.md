# Hydroponics Controller Setup Guide

## For Non-Programmers - Everything You Need to Know

This guide will walk you through setting up your automated hydroponics controller step-by-step. No programming knowledge required!

---

## Table of Contents

1. [What This System Does](#what-this-system-does)
2. [Shopping List](#shopping-list)
3. [Assembly Instructions](#assembly-instructions)
4. [Installing the Software](#installing-the-software)
5. [Configuring Your System](#configuring-your-system)
6. [Calibrating Sensors](#calibrating-sensors)
7. [Daily Operation](#daily-operation)
8. [Troubleshooting](#troubleshooting)
9. [Plant-Specific Settings](#plant-specific-settings)

---

## What This System Does

This controller automatically maintains optimal growing conditions for your hydroponic plants:

| Parameter | What It Measures | Why It Matters |
|-----------|-----------------|----------------|
| **pH** | Acidity/alkalinity | Nutrients are only available to plants within certain pH ranges |
| **TDS/EC** | Dissolved nutrients | Tells you if plants have enough "food" |
| **Temperature** | Water temperature | Affects oxygen levels and root health |
| **Water Level** | Tank fullness | Prevents pump damage and plant stress |

The system will automatically:
- Add pH Up or pH Down solution when pH drifts
- Add nutrient solution when TDS drops
- Add fresh water when levels get low
- Alert you if anything goes wrong

---

## Shopping List

### Required Components (~$200 total)

#### The Brain
| Item | Where to Buy | Approx. Price |
|------|-------------|---------------|
| Arduino Mega 2560 | Amazon, Adafruit | $15-20 |
| MicroSD Card (8GB+) | Any electronics store | $5-10 |
| SD Card Module | Amazon, Adafruit | $3-5 |

#### Sensors
| Item | Model | Where to Buy | Approx. Price |
|------|-------|-------------|---------------|
| pH Sensor Kit | DFRobot SEN0161 | DFRobot.com, Amazon | $30 |
| TDS Sensor | DFRobot SEN0244 | DFRobot.com, Amazon | $12 |
| Temperature Probe | DS18B20 Waterproof | Amazon | $5 |
| Level Sensor | HC-SR04 Ultrasonic | Amazon | $3 |

#### Dosing Pumps
| Item | Quantity | Where to Buy | Approx. Price |
|------|----------|-------------|---------------|
| Peristaltic Pump 12V | 4 | Amazon, AliExpress | $10 each |
| Solenoid Valve 12V | 1 | Amazon | $15 |

#### Electronics
| Item | Where to Buy | Approx. Price |
|------|-------------|---------------|
| 12V 5A Power Supply | Amazon | $12 |
| IRLZ44N MOSFETs (5 pack) | Amazon | $5 |
| 20x4 LCD Display I2C | Amazon | $10 |
| Jumper Wires | Amazon | $5 |
| Breadboard or PCB | Amazon | $5 |

#### Chemicals (Not Included Above)
- pH Down solution (phosphoric acid based)
- pH Up solution (potassium hydroxide based)
- Hydroponic nutrients (A + B formula)

---

## Assembly Instructions

### Step 1: Prepare the Arduino

1. Unbox your Arduino Mega 2560
2. Connect it to your computer with the USB cable
3. The power LED should light up

### Step 2: Connect the Sensors

**pH Sensor:**
```
pH Module Pin    →    Arduino Pin
─────────────────────────────────
V+               →    5V
GND              →    GND
Po (signal)      →    A0
```

**Temperature Sensor (DS18B20):**
```
Wire Color       →    Arduino Pin
─────────────────────────────────
Red              →    5V
Black            →    GND
Yellow (data)    →    D2

IMPORTANT: Add a 4.7kΩ resistor between the Yellow and Red wires!
```

**TDS Sensor:**
```
TDS Module Pin   →    Arduino Pin
─────────────────────────────────
VCC              →    5V
GND              →    GND
A (signal)       →    A2
```

**Ultrasonic Level Sensor (HC-SR04):**
```
Sensor Pin       →    Arduino Pin
─────────────────────────────────
VCC              →    5V
GND              →    GND
Trig             →    D3
Echo             →    D4
```

### Step 3: Connect the Dosing Pumps

Each pump needs a MOSFET driver circuit:

```
Arduino Pin → 1kΩ Resistor → MOSFET Gate
                              ↓
                           MOSFET Drain → Pump Negative
                              ↓
                           MOSFET Source → GND

Pump Positive → 12V Power Supply
```

**Pump Pin Assignments:**
| Pump | Arduino Pin |
|------|-------------|
| pH Down | D5 |
| pH Up | D6 |
| Nutrient A | D7 |
| Nutrient B | D8 |
| Fresh Water Valve | D9 |
| Circulation Pump | D10 |

### Step 4: Connect the Display

**LCD Display (I2C):**
```
LCD Pin          →    Arduino Pin
─────────────────────────────────
VCC              →    5V
GND              →    GND
SDA              →    D20 (SDA)
SCL              →    D21 (SCL)
```

### Step 5: Connect the SD Card

```
SD Module Pin    →    Arduino Pin
─────────────────────────────────
VCC              →    5V
GND              →    GND
MISO             →    D50
MOSI             →    D51
SCK              →    D52
CS               →    D53
```

---

## Installing the Software

You'll install a program called PlatformIO that handles everything for you. Just follow these steps exactly.

### Step 1: Install Visual Studio Code

1. Go to https://code.visualstudio.com/
2. Click the big **Download** button
3. Run the installer
4. Open Visual Studio Code when it's done

### Step 2: Install PlatformIO Extension

1. In VS Code, click the **Extensions** icon on the left sidebar (looks like 4 squares)
2. In the search box, type `PlatformIO`
3. Click **Install** on "PlatformIO IDE"
4. Wait for it to finish (may take a few minutes)
5. **Restart VS Code** when prompted

### Step 3: Download This Project

1. Go to https://github.com/jacobpascalcoblentz/arduino-for-jcn
2. Click the green **Code** button
3. Click **Download ZIP**
4. Extract the ZIP file somewhere you'll remember (like your Desktop)

### Step 4: Open the Project

1. In VS Code, click **File → Open Folder**
2. Navigate to the extracted folder (`arduino-for-jcn-main`)
3. Click **Select Folder**
4. Wait for PlatformIO to load (you'll see activity in the bottom bar)

### Step 5: Connect Your Arduino

1. Plug in your Arduino Mega 2560 via USB cable
2. Wait a few seconds for your computer to recognize it

### Step 6: Upload the Firmware

1. Look at the bottom blue bar in VS Code
2. Click the **→** (right arrow) button - this is "Upload"
3. Wait for it to compile and upload (first time takes a few minutes)
4. You should see **SUCCESS** in the terminal

**That's it!** The firmware is now on your Arduino.

### If Something Goes Wrong

**"No device found" error:**
- Make sure Arduino is plugged in
- Try a different USB cable
- Try a different USB port

**"Access denied" error (Windows):**
- Close any other programs that might be using the Arduino (like Arduino IDE)

**Compilation errors:**
- Make sure you extracted the full ZIP file
- Try **Terminal → Run Task → PlatformIO: Clean** then upload again

### Updating the Firmware Later

If you get an updated version:
1. Download the new ZIP
2. Extract it
3. Open the folder in VS Code
4. Click the Upload button (→)

---

### Alternative: Arduino IDE (if PlatformIO doesn't work)

1. Download Arduino IDE from https://www.arduino.cc/en/software
2. Install these libraries via **Sketch → Include Library → Manage Libraries**:
   - `OneWire`
   - `DallasTemperature`
   - `LiquidCrystal I2C`
3. Open `src/main.cpp`
4. Select **Tools → Board → Arduino Mega 2560**
5. Select **Tools → Port → (your Arduino's port)**
6. Click **Upload** (→ button)

---

## Configuring Your System

This is the important part! You'll edit a simple text file to customize everything.

### Step 1: Prepare the SD Card

1. Format your SD card as FAT32
2. Copy the `config.yaml` file to the SD card
3. The file must be named exactly `config.yaml`

### Step 2: Edit config.yaml

Open `config.yaml` in any text editor (Notepad, TextEdit, etc.)

#### Basic Settings (The Most Important Part!)

Find the `setpoints:` section. These are your target values:

```yaml
setpoints:
  ph: 6.0                    # What pH do you want?
  tds: 800                   # How strong should nutrients be? (ppm)
  temperature: 22            # Ideal water temperature (Celsius)
  water_level: 80            # Keep tank this full (percent)
```

**Common pH Values by Plant Type:**
| Plant | pH Range |
|-------|----------|
| Lettuce, Greens | 5.5 - 6.5 |
| Tomatoes | 5.5 - 6.5 |
| Peppers | 5.5 - 6.0 |
| Strawberries | 5.5 - 6.2 |
| Herbs | 5.5 - 6.5 |
| Cannabis | 5.8 - 6.2 |

> **Note:** If you're growing "tomatoes" that smell like a skunk and require very specific lighting schedules, this system won't judge you. It also won't remember anything you tell it, which is fitting.

**Common TDS Values by Plant Type:**
| Plant | TDS (ppm) |
|-------|-----------|
| Seedlings | 200 - 400 |
| Lettuce | 560 - 840 |
| Tomatoes | 1400 - 3500 |
| Peppers | 1400 - 1750 |
| Strawberries | 1000 - 1400 |
| Herbs | 700 - 1100 |

#### System Settings

Tell the controller about your physical setup:

```yaml
system:
  volume_liters: 100         # How many liters in your reservoir?
  tank_height_cm: 60         # How tall is your tank?
  tank_area_cm2: 2500        # Length × Width of tank in cm²
```

**Example Calculations:**
- 20 gallon tank = ~76 liters
- Tank that is 50cm × 50cm = 2500 cm²

#### Safety Settings

These trigger alarms if something goes wrong:

```yaml
safety:
  ph_min: 5.0                # ALARM if pH drops below this
  ph_max: 7.5                # ALARM if pH rises above this
  tds_min: 300               # ALARM if nutrients too low
  tds_max: 1500              # ALARM if nutrients too high
  level_min: 20              # ALARM if water too low
```

#### Pump Calibration

You need to measure how fast your pumps flow:

1. Put the pump tube in a measuring cup
2. Run the pump for exactly 60 seconds
3. Measure how many mL came out
4. Divide by 60 = mL per second

```yaml
pumps:
  ph_down_ml_per_sec: 1.0    # Your measured value
  ph_up_ml_per_sec: 1.0      # Your measured value
  nutrient_a_ml_per_sec: 2.0 # Your measured value
```

### Step 3: Save and Insert

1. Save the `config.yaml` file
2. Safely eject the SD card
3. Insert it into the Arduino's SD module
4. Power on the Arduino
5. The LCD should show "Loading config..."

---

## Calibrating Sensors

### pH Sensor Calibration

You'll need pH calibration solutions (usually pH 4.0 and pH 7.0).

1. Connect to the Arduino via USB and open Serial Monitor (115200 baud)
2. Type `c` and press Enter to enter calibration mode
3. Rinse the pH probe with distilled water
4. Place probe in pH 7.0 solution, wait 1 minute
5. Type `1` and press Enter
6. Rinse probe, place in pH 4.0 solution, wait 1 minute
7. Type `2` and press Enter
8. Type `q` to exit calibration mode

### TDS Sensor Calibration

You'll need a TDS calibration solution (usually 1000 ppm).

1. Enter calibration mode (type `c`)
2. Place TDS probe in calibration solution
3. Type `3` and press Enter
4. Type the known TDS value (e.g., `1000`) and press Enter
5. Type `q` to exit

### Water Level Calibration

1. Enter calibration mode (type `c`)
2. With tank EMPTY, type `4` and press Enter (records empty distance)
3. Fill tank to desired MAX level
4. Type `5` and press Enter (records full distance)
5. Type `q` to exit

---

## Daily Operation

### LCD Display

The display shows:
```
Line 1: pH:6.50  T:24.5C
Line 2: TDS:800ppm EC:1.60
Line 3: Lvl:80%  NORMAL
Line 4: Stable
```

### Status Indicators

| Display | Meaning |
|---------|---------|
| NORMAL | Everything is good |
| WARNING | Values approaching limits |
| ALARM | Values outside safe range |
| E-STOP | Emergency stop activated |

### Serial Monitor Commands

Connect via USB and open Serial Monitor to use these commands:

| Key | Action |
|-----|--------|
| s | Show full status report |
| c | Enter calibration mode |
| p | Pause/resume automatic control |
| e | Emergency stop (stops all pumps) |
| x | Clear emergency stop |
| r | Reset controller memory |

### Checking the Log

The system saves data to `hydro.csv` on the SD card. You can:

1. Remove the SD card (when system is off)
2. Open the CSV file in Excel or Google Sheets
3. See graphs of pH, TDS, temperature over time

---

## Troubleshooting

### "SD card failed - using defaults"

- Make sure SD card is formatted as FAT32
- Check the SD card is fully inserted
- Try a different SD card

### "SENSOR ERROR" on display

- Check all sensor connections
- Make sure sensors are submerged in water
- Check that the 4.7kΩ resistor is connected for temperature sensor

### pH readings are wrong

- Recalibrate with fresh calibration solutions
- Clean the pH probe (soak in cleaning solution)
- pH probes wear out - may need replacement after 1-2 years

### TDS readings are wrong

- Recalibrate with known TDS solution
- Clean the TDS probe with distilled water
- Make sure probe is fully submerged

### Pumps not running

- Check 12V power supply is connected
- Check MOSFET connections
- Test pumps manually in calibration mode (press 6, 7, 8)

### Water level readings erratic

- Ultrasonic sensor needs clear line of sight to water
- Remove any foam or floating debris
- Mount sensor securely so it doesn't vibrate

### System keeps dosing too much

- Increase the `min_dose_interval_sec` in config (try 600 = 10 minutes)
- Decrease the controller gains in config
- Check that your system volume is set correctly

---

## Plant-Specific Settings

Copy these settings into your `config.yaml`:

### Leafy Greens (Lettuce, Spinach, Kale)
```yaml
setpoints:
  ph: 6.0
  tds: 560
  temperature: 20
  water_level: 80
```

### Tomatoes
```yaml
setpoints:
  ph: 6.0
  tds: 1400
  temperature: 24
  water_level: 80
```

### Strawberries
```yaml
setpoints:
  ph: 5.8
  tds: 1000
  temperature: 20
  water_level: 80
```

### Peppers
```yaml
setpoints:
  ph: 6.0
  tds: 1200
  temperature: 24
  water_level: 80
```

### Herbs (Basil, Mint, Cilantro)
```yaml
setpoints:
  ph: 6.0
  tds: 700
  temperature: 22
  water_level: 80
```

---

## Getting Help

If you're stuck:

1. Check all wire connections
2. Make sure the SD card has `config.yaml`
3. Connect via USB and look at Serial Monitor for error messages
4. Check the `SCHEMATICS.txt` file for wiring diagrams

Good luck with your grow!
