# Hydroponics Controller for Arduino

Automated hydroponics monitoring and control system with feedforward controller. Monitors pH, TDS/EC, temperature, and water level with automatic nutrient dosing and pH adjustment.

**Built for non-programmers** - just edit a simple config file!

## Features

- **pH Control** - Automatic dosing of pH Up/Down solutions
- **Nutrient Management** - Maintains target TDS/EC levels
- **Water Level** - Ultrasonic monitoring with auto top-off
- **Temperature Monitoring** - DS18B20 waterproof probe
- **SD Card Logging** - CSV data export for analysis
- **LCD Display** - Real-time status at a glance
- **Safety System** - Alarms and emergency stop protection

## Quick Start

1. **Build the hardware** - See [SCHEMATICS.txt](SCHEMATICS.txt)
2. **Flash the firmware** - Use PlatformIO or Arduino IDE
3. **Edit config.yaml** - Set your target pH, TDS, etc.
4. **Copy to SD card** - Insert into Arduino
5. **Power on** - System runs automatically

## Configuration

All settings are in `config.yaml` - no programming required:

```yaml
setpoints:
  ph: 6.0          # Target pH
  tds: 800         # Target nutrients (ppm)
  temperature: 22  # Target temp (°C)
  water_level: 80  # Target level (%)
```

See [SETUP_GUIDE.md](SETUP_GUIDE.md) for complete instructions.

## Hardware Requirements

| Component | Model | ~Cost |
|-----------|-------|-------|
| Microcontroller | Arduino Mega 2560 | $15 |
| pH Sensor | DFRobot SEN0161 | $30 |
| TDS Sensor | DFRobot SEN0244 | $12 |
| Temp Probe | DS18B20 Waterproof | $5 |
| Level Sensor | HC-SR04 Ultrasonic | $3 |
| Dosing Pumps | 12V Peristaltic (x4) | $40 |
| Display | 20x4 LCD I2C | $10 |
| **Total** | | **~$200** |

## Plant Presets

| Plant | pH | TDS (ppm) |
|-------|-----|-----------|
| Lettuce | 6.0 | 560 |
| Tomatoes | 6.0 | 1400 |
| Peppers | 6.0 | 1200 |
| Strawberries | 5.8 | 1000 |
| Herbs | 6.0 | 700 |

## Building

```bash
# Run tests
cmake -B build && cmake --build build && ctest --test-dir build

# Upload to Arduino (PlatformIO)
pio run -e mega2560 -t upload
```

## Project Structure

```
├── config.yaml       # User configuration (edit this!)
├── SETUP_GUIDE.md    # Beginner-friendly instructions
├── SCHEMATICS.txt    # Wiring diagrams
├── src/
│   ├── main.cpp      # Main application
│   ├── sensors.*     # Sensor classes
│   ├── controller.*  # Feedforward controller
│   └── yaml_config.* # Config file parser
└── test/             # Google Test unit tests
```

## Documentation

- [SETUP_GUIDE.md](SETUP_GUIDE.md) - Complete setup instructions for non-programmers
- [SCHEMATICS.txt](SCHEMATICS.txt) - Wiring diagrams and bill of materials
- [config.yaml](config.yaml) - Example configuration with comments

## License

MIT
