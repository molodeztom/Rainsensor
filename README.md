| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

Mainly 3d printed rain sensor with integration to HomeAutomation procject

# Rain Sensor Project
Hardware:
- Based on a rainsensor hardware from AliExpress
- Due to much noise when the rocker hits the base I made some modifications

## Features
- **Rocker arm mechanism** for precise rain detection.
- **Modular electronics** with adjustable thresholds and low-power operation.
- **LoRa/Wi-Fi/MQTT integration** for seamless data transmission.
- **Dynamic parameter adjustments** based on rainfall intensity.

---

## Electronics Overview
- **Axle & Suspension**  
  - New axle mounting with TPU suspension.  
  - Redesigned base with TPU buffers for the rocker arm.  

- **Sensing & Signal Processing**  
  - Hall sensor SS49E (replaces reed contact).  
  - **LM393P dual-stage comparator circuit**:  
    - Adjustable threshold.  
    - Second stage with feedback to prevent oscillations.  
    - 3.3V logic-compatible digital output for ESP32-S3.  

- **Mechanical Adjustments**  
  - Dual-plate base leveling system with knurled nuts.  
  - Wedge plate to compensate for garage roof slope.  
  - Cable clamp integrated into the base plate.  

---

## Software Overview
### Core Logic
- **Ultra-Low Power (ULP) Co-Processor**:  
  - Pulse counting handled by the ESP32-S3 ULP for energy efficiency.  
  - Main processor sleeps until:  
    - A predefined pulse count (`Ti`) is reached.  
    - A time interval (`Tm`) expires for rainfall calculation.  

- **Rainfall Calculation**:  
  - Volume computed using:  
    - Bucket volume of the rocker arm.  
    - Pulse count and time interval (`Tm`).  

- **Dynamic Transmission**:  
  - Data sent at interval `Tt`, adjusted based on rainfall intensity.  
  - If no rain:  
    - Main processor wakes after interval `Tks` to report status/battery.  
    - Remains ready for parameter updates.  

### Configuration
- Time intervals (`Ti`, `Tm`, `Tt`, `Tks`) and thresholds configurable via **LoRa**.  

---

## Power Management
- **3.3V LiPo-based power supply** optimized for long-term operation.  
- ESP32-S3 ULP ensures minimal power consumption during idle states.  

---

## Data Transmission
- **LoRa** for long-range, low-power communication.  
- **LoRa-to-Wi-Fi Bridge**:  
  - Integrates with home automation systems.  
  - Converts LoRa payloads to MQTT topics.  

---

## Integration & Visualization
- **Home Automation**:  
  - MQTT integration via Node-RED for automation workflows.  
- **Data Pipeline**:  
  - Node-RED → InfluxDB → Grafana dashboards.  

---

## Setup & Parameters
Key parameters (adjust via LoRa/MQTT):
```plaintext
Ti  = Wake-up pulse count  
Tm  = Rainfall calculation interval  
Tt  = Dynamic data transmission interval  
Tks = Status check interval (no-rain condition)  








