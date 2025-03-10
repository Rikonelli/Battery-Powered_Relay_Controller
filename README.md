# Battery-Powered Relay Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A power-efficient relay controller system with battery monitoring and configurable switching intervals.

> [!CAUTION]
> Working with high voltage is dangerous. Always follow local laws and regulations regarding high voltage work. If you are unsure about the rules in your country, consult a licensed electrician for more information.

## ✨ Features

### Hardware
- 🔋 Single 18650 Li-ion battery operation
- ⚡ USB-C charging (TP4056 module)
- 🔌 5V/10A relay output
- 📊 Battery voltage monitoring
- 💤 Ultra-low power sleep mode (<1μA)

### Firmware
- 🕹️ Push button interface:
  - **Short press**: Toggle device ON/OFF
  - **Long press (2s)**: Cycle between 1s/10s intervals
- 🔄 Automatic relay toggling
- 🚨 Low-battery warning (LED blinking @ 5Hz)
- ⏳ Watchdog timer for system stability

## 🛠️ Hardware Components

| Component | Specification | Qty |
|-----------|---------------|-----|
| Arduino Uno | ATmega328P 16MHz | 1 |
| TP4056 Module | USB-C 1A | 1 |
| MT3608 Boost Converter | 2A Output | 1 |
| 5V Relay Module | SRD-05VDC-SL-C SONGLE 10A | 1 |
| 18650 Battery | 3.7V 2600mAh | 1 |
| Tactile Switch | 6x6mm | 1 |

## 📐 Schematic

[Connection Diagram](docs/schematic.pdf)

**Key Connections:**
BAT+ → TP4056 → MT3608 → Arduino 5V  
BUTTON → D2 (INT0)  
LED → D4 (220Ω resistor)  
RELAY → D7 (Transistor driver)  
VOLTAGE SENSOR → A0 (Direct BAT+ connection, no divider used in prototype)  *Note: For safe operation with batteries of higher voltage, consider adding a voltage divider to prevent ADC overvoltage.*

## ⚙️ Firmware Configuration

```c
// Main configuration  
const float LOW_BATTERY_THRESHOLD = 3.3;  // Li-ion cutoff  
const unsigned long TOGGLE_INTERVALS[] = {1000, 10000};  
const unsigned long BATTERY_CHECK_INTERVAL = 60000;  // 1 minute  
```

## 📥 Installation

### Dependencies
- Arduino IDE 2.0+
- avr-libc

### Upload Firmware

```
git clone https://github.com/yourusername/battery-relay-controller.git  
cd battery-relay-controller/src  
# Select Arduino Uno board and correct port  
# Compile and upload  
```

### Calibration
- Adjust `VOLTAGE_DIVIDER_RATIO` for your divider
- Set `LOW_BATTERY_THRESHOLD` for your battery

🎛️ **Operational Modes**

| Mode        | LED State | Relay State | Current Draw |
|-------------|-----------|-------------|--------------|
| Active      | Solid ON  | Cycling     | 15-20mA      |
| Sleep       | OFF       | OFF         | <1μA         |
| Low Battery | Blinking  | Normal      | 5mA          |

🌟 **Advanced Features**
- **Smart Debouncing**: Hardware-filtered button input with 50ms debounce
- **Battery Profiling**:
```c
void checkBattery() {  
  // 3-sample averaging for stable readings  
  power_adc_enable();  
  delay(5);  // ADC stabilization  
  // ... voltage calculation ...  
}  
```

- **Power Optimization**:
```c
void enterSleepMode() {  
  power_all_disable();  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();  
  sleep_cpu();  
}  
```

📊 **Performance Metrics**

| Parameter        | Value           |
|------------------|-----------------|
| Sleep Current    | 0.8μA           |
| Active Current   | 18mA            |
| Wakeup Time      | <50ms           |
| Battery Life     | ~60 days (2600mAh) |

## 🚀 Future Development Note

Due to time constraints, this prototype focuses on core functionality. Potential improvements include:

- Additional capacitor filtering for power stabilization  
- Overvoltage protection circuits (TVS diode, varistor)  
- Transistor-optimized relay energy management  
- Temperature-compensated battery monitoring  
- Enhanced EMI suppression in high-voltage sections  

Current implementation omits some engineering best practices - hardware modifications are encouraged!


📜 **License**  
MIT License - See LICENSE for details.

Maintainer: Wojciech Rykaczewski  
Documentation: Project Wiki
