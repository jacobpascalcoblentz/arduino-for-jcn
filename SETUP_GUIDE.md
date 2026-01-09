# Hydroponics Controller Setup Guide

## For Non-Programmers - Everything You Need to Know

This guide will walk you through setting up your automated hydroponics controller step-by-step. No programming knowledge required!

> Written by someone who codes so you don't have to. You're welcome. Please don't call me at 2 AM asking why the "blinky light stopped blinking."

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

**That's it!** The firmware is now on your Arduino. You are now technically a "programmer." Update your LinkedIn accordingly.

### If Something Goes Wrong

Don't panic. The Arduino isn't going to explode. Probably.

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

Yes, you actually have to do this part. No, you can't skip it. I know you want to.

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

*"Have you tried turning it off and on again?"* - Tech Support, since 1952

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
5. Re-read this guide (yes, the whole thing)
6. Google the error message
7. Sleep on it - seriously, bugs fix themselves overnight sometimes
8. *Then* text your programmer friend, but only after trying steps 1-7

Good luck with your grow!

---

*If your plants die, it wasn't the code. It was you. But mostly it was the code. But legally, it was you.*

---

## Hardware Integration Testing Checklist

Before trusting this thing with your precious plants, run through this checklist. Think of it as a pre-flight check, except instead of a plane, it's a box of wires that squirts liquid.

### Pre-Power Checklist

- [ ] **Visual inspection** - No loose wires, exposed metal touching, or "that doesn't look right" moments
- [ ] **Power supply off** - Don't test with power connected yet
- [ ] **USB connected** - Arduino hooked up to computer for monitoring
- [ ] **Serial Monitor open** - 115200 baud, ready to see messages

### Power-On Tests

- [ ] **Arduino powers on** - Power LED lights up, no smoke (smoke is bad)
- [ ] **No magic smoke** - Seriously, if anything smells burnt, unplug immediately
- [ ] **Serial output appears** - You should see boot messages

### Sensor Verification

Run each test and record your readings:

#### Temperature Sensor (DS18B20)
- [ ] Sensor detected on boot (check Serial Monitor)
- [ ] Room temp reading: ____°C (should be 18-28°C typically)
- [ ] Hold sensor in hand - temperature rises? [ ] Yes [ ] No
- [ ] Dip in ice water - temperature drops? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

#### pH Sensor
- [ ] Voltage reading appears in Serial Monitor
- [ ] pH 7.0 buffer reading: ______ (should be 6.5-7.5 before calibration)
- [ ] pH 4.0 buffer reading: ______ (should be lower than 7.0 reading)
- [ ] Readings stable (not jumping around wildly)? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

#### TDS Sensor
- [ ] Voltage reading appears in Serial Monitor
- [ ] Tap water reading: ______ ppm (typically 100-500 ppm)
- [ ] Calibration solution reading: ______ ppm
- [ ] Readings stable? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

#### Ultrasonic Level Sensor
- [ ] Distance reading appears in Serial Monitor
- [ ] Empty tank distance: ______ cm
- [ ] Hand in front of sensor shows shorter distance? [ ] Yes [ ] No
- [ ] Readings stable (±2cm)? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

### Actuator Tests

**WARNING: Have paper towels ready. Things are about to get wet.**

Enter calibration mode by pressing 'c' in Serial Monitor.

#### pH Down Pump (Pin D5)
- [ ] Press '6' to test pump
- [ ] Pump runs for 2 seconds? [ ] Yes [ ] No
- [ ] Liquid flows through tubing? [ ] Yes [ ] No
- [ ] Pump stops automatically? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

#### pH Up Pump (Pin D6)
- [ ] Press '7' to test pump
- [ ] Pump runs and liquid flows? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

#### Nutrient A Pump (Pin D7)
- [ ] Press '8' to test pump
- [ ] Pump runs and liquid flows? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

#### Nutrient B Pump (Pin D8)
- [ ] Press '9' to test pump
- [ ] Pump runs and liquid flows? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

#### Fresh Water Valve (Pin D9)
- [ ] Press '0' to test valve
- [ ] Valve opens (you should hear a click)? [ ] Yes [ ] No
- [ ] Water flows when open? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

### SD Card Tests

- [ ] SD card inserted properly (click sound)
- [ ] "SD card initialized" message on boot? [ ] Yes [ ] No
- [ ] config.yaml loaded successfully? [ ] Yes [ ] No
- [ ] Logging to hydro.csv works? [ ] Yes [ ] No (run for 1 min, check file)
- [ ] **PASS/FAIL**: ______

### Safety System Tests

#### Emergency Stop
- [ ] Press 'e' for emergency stop
- [ ] All pumps stop immediately? [ ] Yes [ ] No
- [ ] Display shows "E-STOP"? [ ] Yes [ ] No
- [ ] Press 'x' to clear emergency stop
- [ ] Normal operation resumes? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

#### Sensor Fault Detection
- [ ] Disconnect temperature sensor
- [ ] Fault detected within 10 seconds? [ ] Yes [ ] No
- [ ] Reconnect sensor, fault clears? [ ] Yes [ ] No
- [ ] **PASS/FAIL**: ______

### Full System Integration Test

Run the system for 30 minutes with all sensors and pumps connected:

- [ ] All sensor readings remain stable
- [ ] No unexpected pump activations
- [ ] Controller responds to setpoint deviations
- [ ] Log file records data correctly
- [ ] No error messages in Serial Monitor
- [ ] Display updates correctly
- [ ] System doesn't crash or freeze

### Final Checks Before Deployment

- [ ] All connections secure (give wires a gentle tug)
- [ ] Enclosure protects electronics from splashes
- [ ] Sensors positioned correctly in tank
- [ ] Pump tubing secured and not kinked
- [ ] Power supply adequate (not getting hot)
- [ ] SD card has plenty of space
- [ ] Backup of config.yaml saved somewhere

### Sign-Off

**Tested by**: _______________________

**Date**: _______________________

**Overall Result**: [ ] PASS - Ready for deployment [ ] FAIL - Needs fixes

**Notes**:
_________________________________________
_________________________________________
_________________________________________

*Pro tip: Take a photo of your wiring before closing up the enclosure. Future you will thank present you when something inevitably needs debugging.*

---

## ESP32 WiFi Setup Guide

Want to monitor your hydroponics system from your phone while pretending to work? This section is for you.

### Why ESP32?

The ESP32 is like an Arduino that went to college. It has:
- Built-in WiFi (no shields needed)
- More memory (your plants' data needs a home)
- Faster processor (for those sweet, sweet web dashboards)
- Bluetooth too (because why not)

### Hardware Changes for ESP32

If you're switching from Arduino Mega to ESP32, here are the pin changes:

| Function | Arduino Mega | ESP32 |
|----------|-------------|-------|
| pH Sensor | A0 | GPIO 36 (VP) |
| TDS Sensor | A2 | GPIO 39 (VN) |
| Temperature | D2 | GPIO 4 |
| Ultrasonic Trig | D3 | GPIO 5 |
| Ultrasonic Echo | D4 | GPIO 18 |
| pH Down Pump | D5 | GPIO 19 |
| pH Up Pump | D6 | GPIO 21 |
| Nutrient A Pump | D7 | GPIO 22 |
| Nutrient B Pump | D8 | GPIO 23 |
| Fresh Water Valve | D9 | GPIO 25 |
| Circulation Pump | D10 | GPIO 26 |
| SD Card CS | D53 | GPIO 5 |

**Important Notes:**
- ESP32 analog pins are 12-bit (0-4095) vs Arduino's 10-bit (0-1023)
- ESP32 runs on 3.3V logic - use level shifters for 5V sensors!
- Some GPIO pins have restrictions (GPIO 6-11 are for flash, avoid them)

### Building for ESP32

1. In VS Code with PlatformIO, click the environment selector (bottom blue bar)
2. Select `env:esp32`
3. Click the Upload button (→)

Or from command line:
```bash
pio run -e esp32 -t upload
```

### WiFi Configuration

#### Method 1: Edit config.yaml (Recommended)

Add WiFi settings to your `config.yaml` on the SD card:

```yaml
wifi:
  ssid: "YourNetworkName"
  password: "YourWiFiPassword"
  hostname: "hydroponics"    # Optional - how it appears on network

  # Optional: Static IP (leave out for DHCP)
  # static_ip: "192.168.1.100"
  # gateway: "192.168.1.1"
  # subnet: "255.255.255.0"
```

#### Method 2: AP Mode (No Config Needed)

If you don't configure WiFi or it can't connect:

1. The ESP32 creates its own WiFi network called `hydroponics`
2. Password is `hydroponics123`
3. Connect your phone/laptop to this network
4. Browse to `http://192.168.4.1`

This is useful for:
- Initial setup
- When your router dies
- Growing "tomatoes" in a location without internet access

### Accessing the Dashboard

Once connected to WiFi:

1. Find the IP address:
   - Check Serial Monitor on boot
   - Or check your router's connected devices
   - Or use `http://hydroponics.local` if your network supports mDNS

2. Open a web browser and go to that IP address

3. You'll see a beautiful dashboard with:
   - Real-time pH, temperature, TDS, and water level
   - Color-coded status indicators (green = good, yellow = warning, red = bad)
   - Auto-refresh every 2 seconds

### API Endpoints

For the nerds who want to integrate with home automation:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web dashboard (HTML) |
| `/api/sensors` | GET | Current sensor readings (JSON) |
| `/api/config` | GET | System configuration (JSON) |

#### Example: Get Sensor Data

```bash
curl http://192.168.1.100/api/sensors
```

Returns:
```json
{
  "ph": 6.52,
  "temperature": 24.3,
  "tds": 823,
  "level": 78,
  "uptime": 3600,
  "valid": true
}
```

### Home Assistant Integration

Add to your `configuration.yaml`:

```yaml
sensor:
  - platform: rest
    name: "Hydroponics pH"
    resource: http://192.168.1.100/api/sensors
    value_template: "{{ value_json.ph }}"
    unit_of_measurement: "pH"
    scan_interval: 30

  - platform: rest
    name: "Hydroponics Temperature"
    resource: http://192.168.1.100/api/sensors
    value_template: "{{ value_json.temperature }}"
    unit_of_measurement: "°C"
    scan_interval: 30

  - platform: rest
    name: "Hydroponics TDS"
    resource: http://192.168.1.100/api/sensors
    value_template: "{{ value_json.tds }}"
    unit_of_measurement: "ppm"
    scan_interval: 30

  - platform: rest
    name: "Hydroponics Water Level"
    resource: http://192.168.1.100/api/sensors
    value_template: "{{ value_json.level }}"
    unit_of_measurement: "%"
    scan_interval: 30
```

### MQTT (Coming Soon)

For those who want real-time updates without polling:

```yaml
wifi:
  # ... your wifi settings ...

mqtt:
  enabled: true
  server: "192.168.1.50"
  port: 1883
  username: "hydro"          # Optional
  password: "secret"         # Optional
  topic: "hydroponics/sensors"
  publish_interval_sec: 5
```

The controller will publish JSON to your MQTT broker:
```
hydroponics/sensors → {"ph":6.5,"temp":24.3,"tds":800,"level":80}
```

### Troubleshooting WiFi

**Can't connect to WiFi:**
- Double-check SSID and password (case-sensitive!)
- Make sure your router is 2.4GHz (ESP32 doesn't support 5GHz)
- Try moving closer to the router
- Check Serial Monitor for error messages

**Dashboard not loading:**
- Verify you're on the same network
- Try the IP address directly instead of hostname
- Check if another device has the same IP (static IP conflict)

**Connection keeps dropping:**
- ESP32 might be too far from router
- WiFi interference from other devices
- Power supply might be inadequate (ESP32 needs stable 3.3V)

**AP Mode not appearing:**
- Wait 30 seconds after boot
- Make sure ESP32 is powered properly
- Check Serial Monitor - it logs the AP name and IP

### Security Considerations

**WARNING: This is a basic implementation without authentication.**

Do NOT expose this directly to the internet unless you:
1. Add password authentication (modify `wifi_logger.cpp`)
2. Use HTTPS (requires SSL certificates)
3. Put it behind a VPN or reverse proxy
4. Enable your router's guest network isolation

For home networks, it's generally fine. For commercial grows... hire a security consultant. And maybe a lawyer.
