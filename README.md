# 🚀 Rocket Flight Computer

A comprehensive flight computer system for model rockets, featuring real-time telemetry, autonomous flight phase detection, and recovery system deployment.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Configuration](#configuration)
- [Flight Phases](#flight-phases)
- [Data Output](#data-output)
- [File Structure](#file-structure)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## 🎯 Overview

This rocket flight computer is designed for model rocketry applications, providing:

- **Real-time sensor data collection** from IMU, barometer, and GPS
- **Autonomous flight phase detection** (Ground → Boost → Coast → Apogee → Descent)
- **Recovery system deployment** at apogee detection
- **Wireless telemetry** via LoRa radio
- **Data logging** with comprehensive sensor fusion

The system is built on Arduino-compatible hardware (tested on Teensy 4.1) and uses modular, well-documented code for easy customization and maintenance.

## ✨ Features

### Sensor Integration
- **MPU6050 IMU**: 6-axis motion tracking with DMP (Digital Motion Processor)
- **BMP280 Barometer**: High-precision altitude and temperature measurement
- **GPS Module**: Location tracking with satellite count monitoring
- **Voltage Monitoring**: voltage measurement

### Flight Control
- **Multi-phase detection**: Automatic transition through flight phases
- **Apogee detection**: Sophisticated algorithm using altitude, velocity, and acceleration
- **Recovery deployment**: Relay-controlled parachute/recovery system activation
- **Configurable thresholds**: Easily adjustable detection parameters

### Communication
- **LoRa Telemetry**: Long-range wireless data transmission
- **Real-time monitoring**: 10Hz data output via serial
- **Packet validation**: Checksums and error detection
- **Device identification**: Multi-rocket support

### Data Processing
- **Sensor fusion**: Combining multiple sensor inputs for accuracy
- **Kalman filtering**: Smooth orientation calculations
- **Moving averages**: Noise reduction through buffering
- **Vertical acceleration**: Gravity-compensated acceleration calculation

## 🔧 Hardware Requirements

### Core Components
- **Microcontroller**: Teensy 4.1 (or compatible Arduino board)
- **IMU**: MPU6050 (I2C, pins SDA/SCL)
- **Barometer**: BMP280 (I2C, pins 17/16 on Teensy)
- **GPS**: Any NMEA-compatible module (Serial2, pins 7/8)
- **LoRa Radio**: SX1276/SX1278 compatible (SPI)
- **Recovery Relay**: Connected to pin 7

### Power System
- **Primary Battery**: Connected to analog pin A1
- **Voltage Dividers**: 2.5:1.9 ratio for voltage monitoring

### Connections
```
Teensy 4.1 Pinout:
├── Pin 3:    MPU6050 Interrupt
├── Pin 7:    Recovery Relay Control
├── Pin 16:   BMP280 SCL (Wire1)
├── Pin 17:   BMP280 SDA (Wire1)
├── Pin 7:    GPS RX (Serial2)
├── Pin 8:    GPS TX (Serial2)
├── Pin A1:   Battery Voltage 2
└── SPI Pins: LoRa Module (default SPI pins)
```

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────┐    ┌─────────────────┐
│   Sensors       │    │   Flight     │    │   Outputs       │
│                 │    │  Controller  │    │                 │
│ • MPU6050 (IMU) │───▶│              │───▶│ • Serial Data   │
│ • BMP280 (Baro) │    │ • State      │    │ • LoRa Radio    │
│ • GPS Module    │    │   Machine    │    │ • Recovery      │
│ • Voltage Mon.  │    │ • Apogee     │    │   Relay         │
│                 │    │   Detection  │    │                 │
└─────────────────┘    └──────────────┘    └─────────────────┘
```

### Component Overview

| Component | Purpose | Communication |
|-----------|---------|---------------|
| **FlightController** | Main flight logic and state management | Internal |
| **MPU6050Handler** | IMU data processing and orientation | I2C (Wire) |
| **BMP280Handler** | Altitude and temperature measurement | I2C (Wire1) |
| **GPSHandler** | Location tracking and satellite data | Serial2 |
| **LoRaHandler** | Wireless telemetry transmission | SPI |


## ⚙️ Configuration

All system parameters are centralized in `src/Config.h`:

### Key Configuration Options

```cpp
// Timing
const unsigned long OUTPUT_INTERVAL = 100;         // Data rate (ms)

// Flight Detection Thresholds
const float DEFAULT_LAUNCH_ACCEL_THRESHOLD = 2.0;  // Launch detection (g)
const float DEFAULT_MIN_APOGEE_ALTITUDE = 20.0;    // Minimum apogee height (m)
const float DEFAULT_DESCENT_ALTITUDE_THRESHOLD = 2.0; // Descent confirmation (m)

// Communication
const long LORA_FREQUENCY = 915000000;             // LoRa frequency (Hz)
const int GPS_BAUD_RATE = 9600;                    // GPS baud rate

// Hardware Pins
const int RELAY_PIN = 7;                           // Recovery system pin
const int VOLTAGE_PIN_0 = A0;                      // Battery monitor 1
```

### Calibration

1. **Sea Level Pressure**: Adjust `DEFAULT_SEA_LEVEL_PRESSURE` for accurate altitude
2. **Launch Threshold**: Tune `LAUNCH_ACCEL_THRESHOLD` based on rocket acceleration
3. **Apogee Sensitivity**: Modify `DESCENT_ALTITUDE_THRESHOLD` for deployment timing

## 🛸 Flight Phases

The flight computer automatically detects and transitions through five distinct phases:

### 1. GROUND 🏠
- **State**: Rocket on launch pad
- **Detection**: Low acceleration, minimal altitude change
- **Actions**: System initialization, sensor calibration

### 2. BOOST 🔥
- **State**: Motor burning, rapid acceleration
- **Entry**: Acceleration > 2g AND altitude > 10m
- **Actions**: Data logging, trajectory monitoring

### 3. COAST 🌙
- **State**: Motor burnout, ballistic flight
- **Entry**: Acceleration drops to ~0g
- **Actions**: Apogee detection preparation

### 4. APOGEE 🎯
- **State**: Maximum altitude reached
- **Entry**: Multiple criteria:
  - Negative vertical velocity
  - Altitude decrease confirmation
  - Minimum flight time elapsed
- **Actions**: **Recovery system deployment** 🪂

### 5. DESCENT 📉
- **State**: Falling under parachute
- **Entry**: Significant altitude loss confirmed
- **Actions**: Impact preparation, continued logging

## 📊 Data Output

### Serial Output Format (10Hz)
```
State: COAST Yaw:45.23 Pitch:12.34 Roll:-5.67 VAcc:0.12 VVel:15.8 V0:8.4V V1:7.9V Temp:25.3°C Alt:245.7m Max:245.7m GPS:40.123456,-74.123456 Sats:8
```

### Data Fields
| Field | Description | Units |
|-------|-------------|-------|
| `State` | Current flight phase | Text |
| `Yaw/Pitch/Roll` | Orientation angles | Degrees |
| `VAcc` | Vertical acceleration | g |
| `VVel` | Vertical velocity | m/s |
| `V0/V1` | Battery voltages | Volts |
| `Temp` | Temperature | °C |
| `Alt` | Current altitude | Meters |
| `Max` | Maximum altitude | Meters |
| `GPS` | Latitude, Longitude | Degrees |
| `Sats` | Satellite count | Count |

### LoRa Telemetry
- **Frequency**: 915 MHz (configurable)
- **Packet Rate**: 10 Hz
- **Range**: Several kilometers (line of sight)
- **Data**: Complete sensor packet with checksum

## 📁 File Structure

```
src/
├── main.cpp                 # Main program loop and coordination
├── Config.h                 # System configuration constants
├── FlightController.h/.cpp  # Flight state machine and apogee detection
├── MPU6050Handler.h/.cpp    # IMU sensor management
├── BMP280Handler.h/.cpp     # Barometer sensor management
├── GPSHandler.h/.cpp        # GPS module management
└── LoRaHandler.h/.cpp       # LoRa radio communication
```

### Key Functions

#### Main Loop (`main.cpp`)
- **`setup()`**: Initialize all components
- **`loop()`**: 10Hz sensor reading and data transmission
- **`readSensorData()`**: Collect data from all sensors
- **`updateFlightController()`**: Process flight state
- **`outputSensorData()`**: Serial output formatting
- **`transmitLoRaData()`**: Wireless telemetry

#### Flight Controller (`FlightController.cpp`)
- **`update()`**: Main state machine processing
- **`updateFlightState()`**: Phase transition logic
- **`calculateVerticalAcceleration()`**: Gravity-compensated acceleration
- **Apogee detection algorithm**: Multi-sensor confirmation

## 🚀 Usage

### Pre-Flight Checklist
1. ✅ Power on system and verify serial output
2. ✅ Confirm GPS lock acquisition
3. ✅ Check LoRa telemetry reception
4. ✅ Verify all sensor readings are reasonable
5. ✅ Test recovery system deployment (manually)
6. ✅ Secure all connections and mount in rocket

### Launch Sequence
1. **Install in rocket** with recovery system connected
2. **Power on** and wait for GPS lock
3. **Arm recovery system** (if applicable)
4. **Launch** - system automatically detects flight phases
5. **Monitor telemetry** during flight
6. **Recovery** - parachute deploys at apogee

### Post-Flight
1. **Download data** via serial connection
2. **Analyze flight profile** using logged data
3. **Check system status** and battery levels
4. **Review configuration** for next flight


## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Arduino Community** for excellent libraries and documentation
- **Model Rocketry Community** for inspiration and testing feedback
- **Open Source Contributors** who make projects like this possible

---

**Happy Flying! 🚀**