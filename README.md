# Conrad Robotics Patient Monitoring System
## Setup & Deployment Guide

---

## System Overview

```
[Bedside Unit — ESP32]
  Sensors: MAX30102, DS18B20, AD8232, DS3231
  UI:      OLED + Rotary Encoder
  Storage: SD Card (offline CSV logs by date)
  TX:      ESP-NOW → CYD  |  MQTT → GitHub Pages
        ↓ ESP-NOW (local, fast, no router needed)
[Nurses' Station — CYD ESP32]
  Display: 320×240 ILI9341 TFT (reactive ECG + vitals)
  Audio:   DFPlayer Mini + Speaker
  Alerts:  LED + sound when thresholds breached
        ↓ (same MQTT broker)
[GitHub Pages Web Dashboard]
  Access from any browser on the hospital network
```

---

## Step 1 — Get the CYD's MAC Address

Before flashing the Bedside Unit, you need the exact MAC address of the CYD.

1. Flash `nurses_station_cyd.ino` to the CYD first
2. Open Arduino Serial Monitor at 115200 baud
3. Look for the line: `[ESP-NOW] CYD MAC: XX:XX:XX:XX:XX:XX`
4. Copy those 6 hex values into `bedside_unit.ino`:

```cpp
uint8_t CYD_MAC_ADDRESS[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF};
//                            ↑ replace with actual values
```

---

## Step 2 — TFT_eSPI Configuration for CYD

The CYD uses a non-standard SPI pinout. You MUST edit TFT_eSPI's `User_Setup.h`
(in `Arduino/libraries/TFT_eSPI/`):

```cpp
// Comment out all other drivers, then add:
#define ILI9341_DRIVER

// CYD SPI pins
#define TFT_MOSI  13
#define TFT_SCLK  14
#define TFT_CS    15
#define TFT_DC     2
#define TFT_RST   -1   // Connected to EN pin internally
#define TFT_BL    21   // Backlight — must set HIGH in setup

// Speed
#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY  20000000
```

In your CYD setup(), add: `pinMode(21, OUTPUT); digitalWrite(21, HIGH);`

---

## Step 3 — DFPlayer Mini Wiring (CYD Side)

```
DFPlayer Mini     CYD ESP32
─────────────     ─────────
VCC           →   3.3V
GND           →   GND
TX            →   GPIO 22  (UART2 RX)
RX (via 1kΩ)  →   GPIO 27  (UART2 TX)
SPK_1         →   Speaker +
SPK_2         →   Speaker -
```

**SD card for DFPlayer:** FAT32 formatted, files named:
- `0001.mp3` — "Heart rate critically low"
- `0002.mp3` — "Heart rate critically high"
- `0003.mp3` — "Oxygen level critically low"
- `0004.mp3` — "Temperature critically low, hypothermia risk"
- `0005.mp3` — "Patient fever detected"
- `0006.mp3` — "ECG leads disconnected, please check patient"

You can record these with any text-to-speech tool and save as MP3.

---

## Step 4 — Wi-Fi & MQTT Configuration (Bedside Unit)

Edit `bedside_unit.ino`:

```cpp
#define WIFI_SSID      "YourHospitalWiFi"
#define WIFI_PASSWORD  "YourPassword"
#define MQTT_BROKER    "broker.hivemq.com"   // Free, no account needed
#define MQTT_PORT      1883
```

The MQTT topic published: `conradrobotics/patient/1/vitals`

JSON payload example:
```json
{
  "patient_id": 1,
  "timestamp": 1719360000,
  "temperature": "36.8",
  "heart_rate": 75,
  "spo2": 98,
  "is_critical": false,
  "alert_type": 0
}
```

---

## Step 5 — GitHub Pages Dashboard

1. Create a GitHub repository named `patient-monitor`
2. Upload `index.html` from the `github_pages_dashboard/` folder
3. Go to Settings → Pages → Deploy from branch `main`, folder `/`
4. Your dashboard will be live at:
   `https://YOUR_USERNAME.github.io/patient-monitor/`

The dashboard connects automatically to `broker.hivemq.com` via WebSocket.
If no MQTT data arrives within 6 seconds, it activates **Demo Mode** to show
how the system looks with simulated data.

---

## Critical Alert Thresholds

| Parameter     | Low Alert    | High Alert   |
|---------------|-------------|-------------|
| Heart Rate    | < 50 bpm    | > 120 bpm   |
| SpO2          | < 90 %      | —           |
| Temperature   | < 35.0 °C   | > 38.5 °C   |

Change these in `bedside_unit.ino`:
```cpp
#define HR_MIN    50
#define HR_MAX    120
#define SPO2_MIN  90
#define TEMP_MIN  35.0f
#define TEMP_MAX  38.5f
```

---

## SD Card Log Format (Bedside Unit)

Daily files created automatically: `/20260626.csv`

```
Time,PatientID,Temp_C,HR_bpm,SpO2_%,ECG_raw,Critical,AlertType
08:00:01,1,36.5,74,98,2048,0,0
08:00:02,1,36.5,75,98,2301,0,0
...
```

---

## Required Libraries

### Bedside Unit
| Library | Install via |
|---------|------------|
| Adafruit SSD1306 | Library Manager |
| Adafruit GFX Library | Library Manager |
| RTClib by Adafruit | Library Manager |
| SparkFun MAX3010x | Library Manager |
| DallasTemperature | Library Manager |
| OneWire | Library Manager |
| PubSubClient | Library Manager |
| ArduinoJson | Library Manager |

### CYD Nurses' Station
| Library | Install via |
|---------|------------|
| TFT_eSPI by Bodmer | Library Manager |
| DFRobotDFPlayerMini | Library Manager |

---

## Rotary Encoder Menu (Bedside OLED)

- **Turn CW/CCW** → navigate menu
- **Press button** → toggle menu mode / confirm selection

Menu items and what the CYD prioritises:
| Index | Label | CYD Behaviour |
|-------|-------|--------------|
| 0 | ECG Graph | Full-width ECG sweep (default) |
| 1 | Heart Rate | HR value highlighted |
| 2 | SpO2 | SpO2 value highlighted |
| 3 | Temperature | Temp value highlighted |
| 4 | All Vitals | All sidebar values shown equally |

---

## Troubleshooting

**ESP-NOW not receiving data on CYD:**
- Confirm both devices use `WiFi.mode(WIFI_STA)` before esp_now_init()
- Both must be on the same Wi-Fi channel (channel 0 = auto)
- Double-check the MAC address in CYD_MAC_ADDRESS[]

**MAX30102 reads 0 / invalid:**
- Hold finger flat and still on the sensor window
- Red LED should be visible through fingertip
- Wait 15–20 seconds for the algorithm to stabilise

**SD Card "Mount Failed":**
- Format card as FAT32 (not exFAT)
- Cards > 32 GB may need ESP32 SD_MMC library instead

**DFPlayer not playing:**
- 1kΩ resistor on RX line is mandatory
- MP3 files must be in root, named 0001.mp3, 0002.mp3...
- Check dfPlayer.begin() returns true before calling play()

**OLED shows nothing:**
- Confirm I2C address is 0x3C (most common); try 0x3D if 0x3C fails
- Add: `Wire.begin(21, 22);` before display.begin()
