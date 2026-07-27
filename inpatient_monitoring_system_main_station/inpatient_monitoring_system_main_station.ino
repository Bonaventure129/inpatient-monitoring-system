/**
 * ============================================================
 * PATIENT MONITORING SYSTEM — BEDSIDE UNIT (NON-BLOCKING WIFI)
 * FIXED: Stale-vitals zeroing on mode switch + simplified ESP-NOW
 * peer channel handling so it coexists cleanly with Wi-Fi/MQTT.
 * ============================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <PubSubClient.h>

// --- NETWORK CONFIG ---
const char* WIFI_SSID = "Jesus is Lord";
const char* WIFI_PASS = "ROBOTICS";
const char* MQTT_BROKER = "broker.hivemq.com";
const char* MQTT_TOPIC = "conradrobotics/patient/1/vitals";

// REPLACE WITH CYD MAC
uint8_t CYD_MAC_ADDRESS[] = {0x94, 0x51, 0xDC, 0x32, 0xA7, 0x30};

// --- Hardware Pins ---
#define SD_CS        5
#define ONE_WIRE_BUS 4
#define ECG_OUTPUT   34
#define ECG_LO_PLUS  14
#define ECG_LO_MINUS 32
#define ROTARY_CLK   25
#define ROTARY_DT    26
#define ROTARY_SW    27
#define I2C_SDA      21
#define I2C_SCL      22

enum UIState { STATE_HOME, STATE_MENU, STATE_MEASURE };
enum ParamType { PARAM_NONE = 0, PARAM_BPM, PARAM_SPO2, PARAM_BOTH, PARAM_TEMP, PARAM_ECG };

typedef struct __attribute__((packed)) PatientPayload {
    uint8_t  patient_id;
    float    temperature;
    int16_t  heart_rate;
    int16_t  spo2;
    int16_t  ecg_value;
    bool     ecg_leads_off;
    uint8_t  active_param; 
    uint8_t  hour;
    uint8_t  minute;
    bool     reset_flag;  
} PatientPayload;

PatientPayload patientData;
SemaphoreHandle_t dataMutex; 

portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
RTC_DS3231 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
MAX30105 particleSensor;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

volatile int menuIndex = 0;
volatile bool btnPressed = false;
volatile bool uiNeedsUpdate = true;
UIState currentState = STATE_HOME;
ParamType selectedParam = PARAM_NONE;
const int MENU_ITEMS = 7;
const char* menuStrings[] = {"Read BPM", "Read SpO2", "Read Both", "Read Temp", "Read ECG", "Reset Patient", "<- Home"};

uint32_t irBuffer[100]; 
uint32_t redBuffer[100];
int32_t spo2Value, heartRateValue;
int8_t validSPO2, validHeartRate;
float smoothedBPM = 0;  
float smoothedSpO2 = 0;
unsigned long lastTempRequest = 0;
bool tempRequested = false;

const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// --- FUNCTION PROTOTYPES ---
void handleUI();
void readSlowSensors();
void sensorUITask(void *pvParameters);
void ecgRadioTask(void *pvParameters);

void IRAM_ATTR encoderISR() {
    static uint8_t old_AB = 3; static int enc_val = 0;
    old_AB <<= 2; old_AB |= ((digitalRead(ROTARY_CLK) << 1) | digitalRead(ROTARY_DT));
    enc_val += enc_states[(old_AB & 0x0f)];
    portENTER_CRITICAL_ISR(&isrMux);
    if (enc_val > 3) { if (currentState == STATE_MENU) menuIndex = (menuIndex + 1) % MENU_ITEMS; enc_val = 0; uiNeedsUpdate = true; } 
    else if (enc_val < -3) { if (currentState == STATE_MENU) menuIndex = (menuIndex - 1 + MENU_ITEMS) % MENU_ITEMS; enc_val = 0; uiNeedsUpdate = true; }
    portEXIT_CRITICAL_ISR(&isrMux);
}

void IRAM_ATTR btnISR() {
    static unsigned long lastBtnTime = 0; unsigned long now = millis();
    if (now - lastBtnTime > 250) { if (digitalRead(ROTARY_SW) == LOW) { btnPressed = true; uiNeedsUpdate = true; } lastBtnTime = now; }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(400000); 
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    
    dataMutex = xSemaphoreCreateMutex();
    memset(&patientData, 0, sizeof(patientData));
    patientData.patient_id = 1;

    rtc.begin();
    ds18b20.begin(); ds18b20.setWaitForConversion(false); 
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) particleSensor.setup(60, 4, 2, 100, 411, 4096); 

    // --- NON-BLOCKING WI-FI INITIALIZATION ---
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // fallback channel until Wi-Fi joins
    if (esp_now_init() == ESP_OK) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, CYD_MAC_ADDRESS, 6);
        // FIX: channel = 0 tells ESP-NOW "use whatever channel the STA
        // interface is currently on" instead of a fixed number. Once both
        // this device and the CYD join the same Wi-Fi network, their
        // radios are locked to the same channel automatically, so this
        // peer entry stays valid across router channel changes with no
        // manual re-adding needed.
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }
    
    mqttClient.setServer(MQTT_BROKER, 1883);

    pinMode(ECG_LO_PLUS, INPUT); pinMode(ECG_LO_MINUS, INPUT);
    pinMode(ROTARY_CLK, INPUT_PULLUP); pinMode(ROTARY_DT, INPUT_PULLUP); pinMode(ROTARY_SW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW), btnISR, FALLING);

    xTaskCreatePinnedToCore(sensorUITask, "UI_SENSORS", 8192, NULL, 1, NULL, 0); 
    xTaskCreatePinnedToCore(ecgRadioTask, "ECG_RADIO", 8192, NULL, 2, NULL, 1);
}

// ============================================================
// CORE 1: MAIN LOOP (Non-Blocking Wi-Fi & MQTT)
// ============================================================
void loop() {
    static bool wifiWasConnected = false;
    
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiWasConnected) {
            Serial.print("Bedside Wi-Fi connected, channel = ");
            Serial.println(WiFi.channel());
            // FIX: no more delete/re-add peer dance — peerInfo.channel = 0
            // set in setup() already tracks the current channel automatically.
            wifiWasConnected = true;
        }
        
        if (!mqttClient.connected()) {
            static unsigned long lastMqttAttempt = 0;
            if (millis() - lastMqttAttempt > 5000) {
                mqttClient.connect("Bedside_Conrad"); 
                lastMqttAttempt = millis();
            }
        } else {
            mqttClient.loop();
        }
    } else {
        wifiWasConnected = false;
    }

    static unsigned long lastMqttTx = 0;
    if (millis() - lastMqttTx >= 500 && mqttClient.connected() && currentState == STATE_MEASURE) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            char payload[150];
            snprintf(payload, sizeof(payload), "{\"patient_id\":%d,\"heart_rate\":%d,\"spo2\":%d,\"temperature\":%.1f,\"ecg_value\":%d}",
                patientData.patient_id, patientData.heart_rate, patientData.spo2, patientData.temperature, patientData.ecg_value);
            mqttClient.publish(MQTT_TOPIC, payload);
            xSemaphoreGive(dataMutex);
        }
        lastMqttTx = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
}

// ============================================================
// CORE 0: SLOW SENSORS & UI TASK
// ============================================================
void sensorUITask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long lastClockUpdate = 0;

    for (;;) {
        if (millis() - lastClockUpdate > 1000) { 
            DateTime now = rtc.now();
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                patientData.hour = now.hour(); 
                patientData.minute = now.minute();
                xSemaphoreGive(dataMutex);
            }
            uiNeedsUpdate = true; 
            lastClockUpdate = millis(); 
        }
        
        handleUI();
        readSlowSensors();
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

// ============================================================
// CORE 1: LIGHTNING FAST ECG & ESP-NOW TASK
// ============================================================
void ecgRadioTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(40);

    for(;;) {
        if (currentState == STATE_MEASURE) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                if (selectedParam == PARAM_ECG || selectedParam == PARAM_BOTH) {
                    patientData.ecg_leads_off = (digitalRead(ECG_LO_PLUS) || digitalRead(ECG_LO_MINUS));
                    patientData.ecg_value = analogRead(ECG_OUTPUT);
                }
                esp_now_send(CYD_MAC_ADDRESS, (uint8_t*)&patientData, sizeof(patientData));
                xSemaphoreGive(dataMutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================================
// HELPER FUNCTIONS (Running inside Core 0)
// ============================================================
void handleUI() {
    if (btnPressed) {
        portENTER_CRITICAL(&isrMux); btnPressed = false; portEXIT_CRITICAL(&isrMux);
        if (currentState == STATE_HOME) { currentState = STATE_MENU; menuIndex = 0; } 
        else if (currentState == STATE_MENU) {
            if (menuIndex == 5) { 
                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                    patientData.reset_flag = true;
                    esp_now_send(CYD_MAC_ADDRESS, (uint8_t*)&patientData, sizeof(patientData));
                    delay(20); 
                    patientData.reset_flag = false;
                    patientData.patient_id++; 
                    xSemaphoreGive(dataMutex);
                }
                currentState = STATE_HOME; 
            } 
            else if (menuIndex == 6) { currentState = STATE_HOME; } 
            else { 
                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                    selectedParam = (ParamType)(menuIndex + 1); 
                    patientData.active_param = selectedParam;
                    // FIX: zero out whichever vitals this new mode does NOT
                    // measure, so a stale reading from the previous mode can
                    // never be transmitted/displayed/alerted on again.
                    if (selectedParam != PARAM_BPM && selectedParam != PARAM_BOTH) patientData.heart_rate = 0;
                    if (selectedParam != PARAM_SPO2 && selectedParam != PARAM_BOTH) patientData.spo2 = 0;
                    if (selectedParam != PARAM_TEMP) patientData.temperature = 0;
                    if (selectedParam != PARAM_ECG && selectedParam != PARAM_BOTH) {
                        patientData.ecg_value = 0;
                        patientData.ecg_leads_off = false;
                    }
                    xSemaphoreGive(dataMutex);
                }
                currentState = STATE_MEASURE; 
                particleSensor.clearFIFO(); smoothedBPM = 0; smoothedSpO2 = 0;
            }
        } 
        else if (currentState == STATE_MEASURE) { 
            currentState = STATE_MENU; 
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                selectedParam = PARAM_NONE; 
                patientData.active_param = PARAM_NONE;
                xSemaphoreGive(dataMutex);
            }
        }
    }
    
    if (!uiNeedsUpdate) return;
    uiNeedsUpdate = false;
    
    PatientPayload localDisplay;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        localDisplay = patientData;
        xSemaphoreGive(dataMutex);
    }

    display.clearDisplay(); display.setTextSize(1);
    display.fillRect(0, 0, 128, 12, SSD1306_WHITE); display.setTextColor(SSD1306_BLACK); display.setCursor(2, 2); 
    display.print("BED "); display.print(localDisplay.patient_id);
    display.setCursor(95, 2); display.printf("%02d:%02d", localDisplay.hour, localDisplay.minute);
    display.setTextColor(SSD1306_WHITE);
    
    if (currentState == STATE_HOME) { 
        display.setCursor(15, 25); display.print("Push knob to Menu"); 
        display.setCursor(26, 45); display.print("Stay healthy!");
    } 
    else if (currentState == STATE_MENU) {
        int startItem = (menuIndex >= 4) ? (menuIndex - 3) : 0;
        for (int i = 0; i < 4; i++) {
            int a = startItem + i; if (a >= MENU_ITEMS) break; int y = 16 + (i * 12);
            if (a == menuIndex) { display.fillRect(0, y - 1, 128, 11, SSD1306_WHITE); display.setTextColor(SSD1306_BLACK); display.setCursor(2, y); display.print(">"); } 
            else { display.setTextColor(SSD1306_WHITE); display.setCursor(8, y); }
            display.setCursor(12, y); display.print(menuStrings[a]);
        }
    } 
    else if (currentState == STATE_MEASURE) {
        display.setCursor(0, 16); display.print("Measuring..."); display.setTextSize(2); 
        String b = (localDisplay.heart_rate == 0) ? "---" : String(localDisplay.heart_rate);
        String s = (localDisplay.spo2 == 0) ? "---" : String(localDisplay.spo2);
        if (selectedParam == PARAM_BPM) { display.setCursor(10, 32); display.print(b); display.print(" bpm"); }
        else if (selectedParam == PARAM_SPO2) { display.setCursor(10, 32); display.print(s); display.print(" %"); }
        else if (selectedParam == PARAM_BOTH) { display.setTextSize(1); display.setCursor(10, 28); display.print("BPM: "); display.print(b); display.setCursor(10, 40); display.print("SpO2: "); display.print(s); }
        else if (selectedParam == PARAM_TEMP) { display.setCursor(10, 32); display.print(localDisplay.temperature, 1); display.print(" C"); }
        else if (selectedParam == PARAM_ECG) { display.setTextSize(1); display.setCursor(10, 32); display.print("ECG: "); display.print(localDisplay.ecg_value); }
        display.setTextSize(1); display.setCursor(0, 56); display.print("<- Press to stop");
    }
    display.display();
}

void readSlowSensors() {
    unsigned long currentMillis = millis();

    if (currentState == STATE_MEASURE && (selectedParam == PARAM_BPM || selectedParam == PARAM_SPO2 || selectedParam == PARAM_BOTH)) {
        if (particleSensor.getIR() < 50000) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                patientData.heart_rate = 0; patientData.spo2 = 0;
                xSemaphoreGive(dataMutex);
            }
            smoothedBPM = 0; smoothedSpO2 = 0;
            uiNeedsUpdate = true;
        } else {
            static int sampleCounter = 0;
            particleSensor.check(); 
            while (particleSensor.available()) {
                memmove(redBuffer, redBuffer + 1, 99 * sizeof(uint32_t)); memmove(irBuffer, irBuffer + 1, 99 * sizeof(uint32_t));
                redBuffer[99] = particleSensor.getRed(); irBuffer[99] = particleSensor.getIR();
                particleSensor.nextSample(); sampleCounter++;
            }
            if (sampleCounter >= 25) {
                maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2Value, &validSPO2, &heartRateValue, &validHeartRate);
                if (validHeartRate && heartRateValue > 30 && heartRateValue < 220) {
                    smoothedBPM = (smoothedBPM == 0) ? heartRateValue : (smoothedBPM * 0.9) + (heartRateValue * 0.1);
                    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) { patientData.heart_rate = (int16_t)smoothedBPM; xSemaphoreGive(dataMutex); }
                }
                if (validSPO2 && spo2Value >= 50 && spo2Value <= 100) {
                    smoothedSpO2 = (smoothedSpO2 == 0) ? spo2Value : (smoothedSpO2 * 0.9) + (spo2Value * 0.1);
                    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) { patientData.spo2 = (int16_t)smoothedSpO2; xSemaphoreGive(dataMutex); }
                }
                sampleCounter = 0; uiNeedsUpdate = true; 
            }
        }
    }

    if (selectedParam == PARAM_TEMP) {
        if (!tempRequested && currentMillis - lastTempRequest > 1000) {
            ds18b20.requestTemperatures(); lastTempRequest = currentMillis; tempRequested = true;
        } else if (tempRequested && currentMillis - lastTempRequest > 750) {
            float t = ds18b20.getTempCByIndex(0);
            if (t > 10.0 && t < 85.0) { 
                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) { patientData.temperature = t; xSemaphoreGive(dataMutex); }
                uiNeedsUpdate = true; 
            }
            tempRequested = false;
        }
    }
}