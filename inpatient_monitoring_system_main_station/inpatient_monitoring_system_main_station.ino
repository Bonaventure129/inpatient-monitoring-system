/**
 * ============================================================
 *  PATIENT MONITORING SYSTEM — BEDSIDE UNIT
 *  Device: ESP32 (Standard 30-pin or 38-pin)
 *  Author: Conrad Robotics
 * ============================================================
 *  ARCHITECTURE:
 *   Core 0 → Sensor polling, ECG sampling, OLED UI,
 *             Rotary Encoder, ESP-NOW transmission, SD logging
 *   Core 1 → Wi-Fi management, MQTT publishing (GitHub Pages dashboard)
 *
 *  SENSORS:
 *   - MAX30102  : Heart Rate (BPM) + SpO2 via I2C (GPIO 21/22)
 *   - DS18B20   : Body Temperature via 1-Wire (GPIO 4)
 *   - AD8232    : ECG analog signal (GPIO 34, 14, 32)
 *   - DS3231    : Real-Time Clock via I2C (GPIO 21/22)
 *   - SD Card   : SPI data logger (GPIO 5, 18, 19, 23)
 *   - OLED      : SSD1306 128x64 via I2C (GPIO 21/22)
 *   - KY-040    : Rotary Encoder (GPIO 25, 26, 27)
 *
 *  REQUIRED LIBRARIES (Arduino Library Manager):
 *   - Adafruit SSD1306
 *   - Adafruit GFX Library
 *   - RTClib (by Adafruit)
 *   - SparkFun MAX3010x Pulse and Proximity Sensor Library
 *   - DallasTemperature
 *   - OneWire
 *   - PubSubClient (by Nick O'Leary)
 *   - ArduinoJson (by Benoit Blanchon)
 * ============================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Sensor libraries
#include "RTClib.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// ============================================================
//  CONFIGURATION — EDIT THESE
// ============================================================
#define PATIENT_ID        1          // Unique bed/patient number
#define WIFI_SSID         "Jesus is Lord"
#define WIFI_PASSWORD     "ROBOTICS"
#define MQTT_BROKER       "broker.hivemq.com"  // Free public broker
#define MQTT_PORT         1883
#define MQTT_TOPIC_BASE   "conradrobotics/patient/"

// CYD MAC address — replace with actual MAC from CYD's setup serial output
uint8_t CYD_MAC_ADDRESS[] = {0x94, 0x51, 0xDC, 0x32, 0xA7, 0x30};

// Critical alert thresholds
#define HR_MIN            50      // bpm — below this is bradycardia alert
#define HR_MAX            120     // bpm — above this is tachycardia alert
#define SPO2_MIN          90      // % — below this is hypoxia alert
#define TEMP_MIN          35.0f   // °C — below this is hypothermia alert
#define TEMP_MAX          38.5f   // °C — above this is fever alert

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define SD_CS             5
#define ONE_WIRE_BUS      4
#define ECG_OUTPUT        34
#define ECG_LO_PLUS       14
#define ECG_LO_MINUS      32
#define ROTARY_CLK        25
#define ROTARY_DT         26
#define ROTARY_SW         27
#define I2C_SDA           21
#define I2C_SCL           22

// ============================================================
//  SHARED DATA STRUCTURE (must be identical on both devices)
// ============================================================
typedef struct PatientPayload {
    uint8_t  patient_id;       // Bed / patient number
    float    temperature;      // DS18B20 body temp in °C
    int16_t  heart_rate;       // BPM from MAX30102
    int16_t  spo2;             // SpO2 % from MAX30102
    int16_t  ecg_value;        // Raw ADC value 0–4095
    bool     ecg_leads_off;    // True if AD8232 leads are disconnected
    uint8_t  active_param;     // Selected parameter: 0=ECG,1=HR,2=SpO2,3=Temp,4=All
    bool     is_critical;      // True if any value exceeds threshold
    uint8_t  alert_type;       // 0=None 1=HR_low 2=HR_high 3=SpO2 4=Temp_low 5=Temp_high
    uint32_t timestamp;        // Unix epoch from RTC
} PatientPayload;

PatientPayload patientData;

// ============================================================
//  OBJECT DECLARATIONS
// ============================================================
Adafruit_SSD1306 display(128, 64, &Wire, -1);
RTC_DS3231       rtc;
MAX30105         particleSensor;
OneWire          oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
WiFiClient       espClient;
PubSubClient     mqttClient(espClient);

// ============================================================
//  FREERTOS HANDLES & SYNCHRONISATION
// ============================================================
TaskHandle_t TaskCore0;
TaskHandle_t TaskCore1;
SemaphoreHandle_t dataMutex;   // Protects patientData between cores

// ============================================================
//  MAX30102 — Beat detection buffers
// ============================================================
#define MAX30102_BUFFER_LEN 100
uint32_t irBuffer[MAX30102_BUFFER_LEN];
uint32_t redBuffer[MAX30102_BUFFER_LEN];
int32_t  spo2Value;
int8_t   validSPO2;
int32_t  heartRateValue;
int8_t   validHeartRate;

// ============================================================
//  ROTARY ENCODER — State machine
// ============================================================
volatile int  encoderCount  = 0;
volatile bool btnPressed    = false;
int           menuIndex     = 0;      // 0–4 matching active_param
int           lastMenuIndex = -1;
unsigned long lastBtnTime   = 0;

const char* menuItems[] = {
    "ECG Graph",
    "Heart Rate",
    "SpO2",
    "Temperature",
    "All Vitals"
};
#define MENU_COUNT 5

// ============================================================
//  SD CARD — Logging
// ============================================================
unsigned long lastSDLog   = 0;
#define SD_LOG_INTERVAL_MS 1000

// ============================================================
//  OLED — Screen states
// ============================================================
unsigned long lastOledUpdate = 0;
#define OLED_UPDATE_INTERVAL_MS 200

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void core0Task(void* pvParameters);
void core1Task(void* pvParameters);
void IRAM_ATTR encoderISR();
void IRAM_ATTR btnISR();
void onESPNowSent(const wifi_tx_info_t* txInfo, esp_now_send_status_t status);
void readMAX30102();
void readDS18B20();
void readECG();
void evaluateCritical();
void updateOLED();
void logToSD(DateTime now);
void drawMenuOLED();
void connectMQTT();
void publishMQTT();
String buildMQTTJson();

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n=== BEDSIDE UNIT — Conrad Robotics ==="));

    Wire.begin(I2C_SDA, I2C_SCL);

    // ---- OLED ----
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("[OLED] FAIL"));
    } else {
        splashScreen();
    }

    // ---- RTC ----
    if (!rtc.begin()) {
        Serial.println(F("[RTC] FAIL — check wiring"));
    } else {
        if (rtc.lostPower()) {
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
            Serial.println(F("[RTC] Power lost — time reset to compile time"));
        }
        Serial.println(F("[RTC] OK"));
    }

    // ---- MAX30102 ----
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println(F("[MAX30102] FAIL — check wiring"));
    } else {
        particleSensor.setup(60, 4, 2, 100, 411, 4096);
        // ledBrightness=60, sampleAverage=4, ledMode=2(Red+IR),
        // sampleRate=100, pulseWidth=411, adcRange=4096
        Serial.println(F("[MAX30102] OK"));
    }

    // ---- DS18B20 ----
    ds18b20.begin();
    Serial.printf("[DS18B20] Found %d sensor(s)\n", ds18b20.getDeviceCount());

    // ---- SD Card ----
    if (!SD.begin(SD_CS)) {
        Serial.println(F("[SD] FAIL — check card/wiring"));
    } else {
        Serial.printf("[SD] OK — %llu MB\n", SD.cardSize() / (1024 * 1024));
    }

    // ---- Rotary Encoder ----
    pinMode(ROTARY_CLK, INPUT);
    pinMode(ROTARY_DT,  INPUT);
    pinMode(ROTARY_SW,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW),  btnISR,     FALLING);
    Serial.println(F("[ENCODER] OK"));

    // ---- ECG ----
    pinMode(ECG_LO_PLUS,  INPUT);
    pinMode(ECG_LO_MINUS, INPUT);
    Serial.println(F("[ECG] OK"));

    // ---- Initial data defaults ----
    patientData.patient_id    = PATIENT_ID;
    patientData.active_param  = 0;   // Start on ECG
    patientData.is_critical   = false;
    patientData.alert_type    = 0;
    patientData.temperature   = 0.0f;
    patientData.heart_rate    = 0;
    patientData.spo2          = 0;
    patientData.ecg_value     = 0;
    patientData.ecg_leads_off = true;

    // ---- FreeRTOS mutex ----
    dataMutex = xSemaphoreCreateMutex();

    // ---- FreeRTOS tasks ----
    xTaskCreatePinnedToCore(core0Task, "Core0_Local", 16000, NULL, 2, &TaskCore0, 0);
    xTaskCreatePinnedToCore(core1Task, "Core1_Cloud", 16000, NULL, 1, &TaskCore1, 1);

    Serial.println(F("[BOOT] Complete — FreeRTOS running\n"));
    vTaskDelete(NULL);  // Delete the Arduino loop task
}

void loop() {}  // Never reached — FreeRTOS takes over

// ============================================================
//  CORE 0 — Sensors, ESP-NOW, OLED, SD Card
// ============================================================
void core0Task(void* pvParameters) {
    // ESP-NOW init on Core 0
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println(F("[ESP-NOW] Init FAIL"));
    } else {
        esp_now_register_send_cb(onESPNowSent);

        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, CYD_MAC_ADDRESS, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            Serial.println(F("[ESP-NOW] Peer registered OK"));
        }
    }

    // Prime the MAX30102 buffer
    for (int i = 0; i < MAX30102_BUFFER_LEN; i++) {
        while (!particleSensor.available()) {
            particleSensor.check();
        }
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i]  = particleSensor.getIR();
        particleSensor.nextSample();
    }
    maxim_heart_rate_and_oxygen_saturation(
        irBuffer, MAX30102_BUFFER_LEN, redBuffer,
        &spo2Value, &validSPO2,
        &heartRateValue, &validHeartRate);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);  // 100 Hz

    for (;;) {
        // --- Handle encoder menu ---
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            patientData.active_param = (uint8_t)menuIndex;
            xSemaphoreGive(dataMutex);
        }

        // --- ECG (every tick — 100 Hz) ---
        readECG();

        // --- Slower sensors (every 25 ticks — ~4 Hz) ---
        static uint8_t slowTick = 0;
        if (++slowTick >= 25) {
            slowTick = 0;
            readMAX30102();
            readDS18B20();
            evaluateCritical();
        }

        // --- OLED update ---
        if (millis() - lastOledUpdate >= OLED_UPDATE_INTERVAL_MS) {
            lastOledUpdate = millis();
            if (btnPressed) {
                drawMenuOLED();
            } else {
                updateOLED();
            }
        }

        // --- ESP-NOW send ---
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            esp_now_send(CYD_MAC_ADDRESS, (uint8_t*)&patientData, sizeof(patientData));
            xSemaphoreGive(dataMutex);
        }

        // --- SD Card log ---
        if (millis() - lastSDLog >= SD_LOG_INTERVAL_MS) {
            lastSDLog = millis();
            DateTime now = rtc.now();
            logToSD(now);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ============================================================
//  CORE 1 — Wi-Fi + MQTT
// ============================================================
void core1Task(void* pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(3000));  // Let Core 0 bring up ESP-NOW first

    // Wi-Fi connect
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(F("[WiFi] Connecting"));
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected — IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println(F("\n[WiFi] FAILED — running offline"));
    }

    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setBufferSize(512);

    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!mqttClient.connected()) {
                connectMQTT();
            }
            mqttClient.loop();
            publishMQTT();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Publish every second
    }
}

// ============================================================
//  SENSOR READS
// ============================================================
void readECG() {
    bool lo_plus  = digitalRead(ECG_LO_PLUS);
    bool lo_minus = digitalRead(ECG_LO_MINUS);

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        patientData.ecg_leads_off = (lo_plus || lo_minus);
        if (!patientData.ecg_leads_off) {
            patientData.ecg_value = (int16_t)analogRead(ECG_OUTPUT);
        }
        xSemaphoreGive(dataMutex);
    }
}

void readMAX30102() {
    // Shift buffer left by 25 samples and read 25 new ones
    for (int i = 25; i < MAX30102_BUFFER_LEN; i++) {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25]  = irBuffer[i];
    }
    for (int i = (MAX30102_BUFFER_LEN - 25); i < MAX30102_BUFFER_LEN; i++) {
        while (!particleSensor.available()) {
            particleSensor.check();
        }
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i]  = particleSensor.getIR();
        particleSensor.nextSample();
    }
    maxim_heart_rate_and_oxygen_saturation(
        irBuffer, MAX30102_BUFFER_LEN, redBuffer,
        &spo2Value, &validSPO2,
        &heartRateValue, &validHeartRate);

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (validHeartRate && heartRateValue > 20 && heartRateValue < 250) {
            patientData.heart_rate = (int16_t)heartRateValue;
        }
        if (validSPO2 && spo2Value > 50) {
            patientData.spo2 = (int16_t)spo2Value;
        }
        xSemaphoreGive(dataMutex);
    }
}

void readDS18B20() {
    ds18b20.requestTemperatures();
    float t = ds18b20.getTempCByIndex(0);

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (t != DEVICE_DISCONNECTED_C) {
            patientData.temperature = t;
        }
        xSemaphoreGive(dataMutex);
    }
}

void evaluateCritical() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        patientData.is_critical = false;
        patientData.alert_type  = 0;

        if (patientData.heart_rate > 0 && patientData.heart_rate < HR_MIN) {
            patientData.is_critical = true;
            patientData.alert_type  = 1;
        } else if (patientData.heart_rate > HR_MAX) {
            patientData.is_critical = true;
            patientData.alert_type  = 2;
        } else if (patientData.spo2 > 0 && patientData.spo2 < SPO2_MIN) {
            patientData.is_critical = true;
            patientData.alert_type  = 3;
        } else if (patientData.temperature > 10.0f && patientData.temperature < TEMP_MIN) {
            patientData.is_critical = true;
            patientData.alert_type  = 4;
        } else if (patientData.temperature > TEMP_MAX) {
            patientData.is_critical = true;
            patientData.alert_type  = 5;
        }

        xSemaphoreGive(dataMutex);
    }
}

// ============================================================
//  OLED UI
// ============================================================
void splashScreen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
    display.setCursor(10, 8);
    display.setTextSize(1);
    display.println(F("PATIENT MONITOR"));
    display.drawLine(0, 18, 128, 18, SSD1306_WHITE);
    display.setCursor(20, 25);
    display.setTextSize(1);
    display.println(F("Conrad Robotics"));
    display.setCursor(15, 38);
    display.println(F("Made in Nigeria"));
    display.setCursor(28, 52);
    display.println(F("Booting..."));
    display.display();
    delay(2500);
}

void updateOLED() {
    display.clearDisplay();

    // Header bar
    display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(2, 2);
    display.printf("BED %02d", PATIENT_ID);

    DateTime now = rtc.now();
    display.setCursor(60, 2);
    display.printf("%02d:%02d:%02d", now.hour(), now.minute(), now.second());

    display.setTextColor(SSD1306_WHITE);

    uint8_t param;
    float   temp;
    int16_t hr, sp;
    bool    crit;
    uint8_t alertT;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        param  = patientData.active_param;
        temp   = patientData.temperature;
        hr     = patientData.heart_rate;
        sp     = patientData.spo2;
        crit   = patientData.is_critical;
        alertT = patientData.alert_type;
        xSemaphoreGive(dataMutex);
    }

    // Critical alert banner
    if (crit) {
        display.fillRect(0, 56, 128, 8, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(2, 57);
        const char* alerts[] = {"", "HR LOW!", "HR HIGH!", "SpO2 LOW!", "HYPOTHERMIA!", "FEVER!"};
        display.print(alerts[alertT]);
        display.setTextColor(SSD1306_WHITE);
    }

    // Vitals
    display.setTextSize(1);
    display.setCursor(0, 15);
    display.printf("HR:   %3d bpm", hr);
    display.setCursor(0, 25);
    display.printf("SpO2: %3d %%", sp);
    display.setCursor(0, 35);
    display.printf("Temp: %.1f C", temp);

    // Active parameter indicator
    display.setCursor(80, 15);
    display.print(F("Param:"));
    display.setCursor(80, 25);
    const char* shortLabels[] = {"ECG", "HR", "SpO2", "Temp", "All"};
    display.print(shortLabels[param]);

    display.display();
}

void drawMenuOLED() {
    display.clearDisplay();

    display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(20, 2);
    display.print(F("SELECT PARAM"));
    display.setTextColor(SSD1306_WHITE);

    for (int i = 0; i < MENU_COUNT; i++) {
        int y = 14 + i * 10;
        if (i == menuIndex) {
            display.fillRect(0, y, 128, 10, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK);
        } else {
            display.setTextColor(SSD1306_WHITE);
        }
        display.setCursor(4, y + 1);
        display.print(menuItems[i]);
    }
    display.display();
}

// ============================================================
//  SD CARD LOGGING
// ============================================================
void logToSD(DateTime now) {
    char filename[16];
    sprintf(filename, "/%04d%02d%02d.csv", now.year(), now.month(), now.day());

    // Write CSV header if file is new
    if (!SD.exists(filename)) {
        File hdr = SD.open(filename, FILE_WRITE);
        if (hdr) {
            hdr.println(F("Time,PatientID,Temp_C,HR_bpm,SpO2_%,ECG_raw,Critical,AlertType"));
            hdr.close();
        }
    }

    float   temp;
    int16_t hr, sp, ecg;
    bool    crit;
    uint8_t alertT;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        temp   = patientData.temperature;
        hr     = patientData.heart_rate;
        sp     = patientData.spo2;
        ecg    = patientData.ecg_value;
        crit   = patientData.is_critical;
        alertT = patientData.alert_type;
        xSemaphoreGive(dataMutex);
    }

    File dataFile = SD.open(filename, FILE_APPEND);
    if (dataFile) {
        dataFile.printf("%02d:%02d:%02d,%d,%.1f,%d,%d,%d,%d,%d\n",
            now.hour(), now.minute(), now.second(),
            PATIENT_ID, temp, hr, sp, ecg, (int)crit, alertT);
        dataFile.close();
    }
}

// ============================================================
//  MQTT
// ============================================================
void connectMQTT() {
    char clientId[20];
    sprintf(clientId, "patient_%d_bedside", PATIENT_ID);

    uint8_t retries = 0;
    while (!mqttClient.connected() && retries < 5) {
        Serial.print(F("[MQTT] Connecting..."));
        if (mqttClient.connect(clientId)) {
            Serial.println(F(" OK"));
        } else {
            Serial.printf(" FAIL rc=%d, retry in 2s\n", mqttClient.state());
            vTaskDelay(pdMS_TO_TICKS(2000));
            retries++;
        }
    }
}

void publishMQTT() {
    if (!mqttClient.connected()) return;

    char topic[64];
    sprintf(topic, "%s%d/vitals", MQTT_TOPIC_BASE, PATIENT_ID);

    String payload = buildMQTTJson();
    mqttClient.publish(topic, payload.c_str());
}

String buildMQTTJson() {
    StaticJsonDocument<256> doc;

    float   temp;
    int16_t hr, sp;
    bool    crit;
    uint8_t alertT;
    uint32_t ts;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        temp   = patientData.temperature;
        hr     = patientData.heart_rate;
        sp     = patientData.spo2;
        crit   = patientData.is_critical;
        alertT = patientData.alert_type;
        ts     = patientData.timestamp;
        xSemaphoreGive(dataMutex);
    }

    DateTime now = rtc.now();

    doc["patient_id"]  = PATIENT_ID;
    doc["timestamp"]   = now.unixtime();
    doc["temperature"] = serialized(String(temp, 1));
    doc["heart_rate"]  = hr;
    doc["spo2"]        = sp;
    doc["is_critical"] = crit;
    doc["alert_type"]  = alertT;

    String output;
    serializeJson(doc, output);
    return output;
}

// ============================================================
//  INTERRUPTS
// ============================================================
void IRAM_ATTR encoderISR() {
    static int lastCLK = LOW;
    int clk = digitalRead(ROTARY_CLK);
    if (clk != lastCLK && clk == HIGH) {
        if (digitalRead(ROTARY_DT) != clk) {
            menuIndex = (menuIndex + 1) % MENU_COUNT;   // CW → next
        } else {
            menuIndex = (menuIndex - 1 + MENU_COUNT) % MENU_COUNT;  // CCW → prev
        }
    }
    lastCLK = clk;
}

void IRAM_ATTR btnISR() {
    unsigned long now = millis();
    if (now - lastBtnTime > 200) {   // 200 ms debounce
        btnPressed = !btnPressed;
        lastBtnTime = now;
    }
}

// ============================================================
//  ESP-NOW SEND CALLBACK
// ============================================================
void onESPNowSent(const wifi_tx_info_t* txInfo, esp_now_send_status_t status) {
    // ESP32 Arduino Core 3.x: MAC is now in txInfo->ra (receiver address)
    // Optional: track delivery failure count for reliability monitoring
    // if (status != ESP_NOW_SEND_SUCCESS) { /* handle failure */ }
}