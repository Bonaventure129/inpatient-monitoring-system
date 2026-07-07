/**
 * ============================================================
 * PATIENT MONITORING SYSTEM — BEDSIDE UNIT (FINAL SENDER)
 * Features: ESP-NOW + Medical Sensors + Polling UI
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

#define ESPNOW_CHANNEL 1

// REPLACE WITH YOUR CYD'S MAC ADDRESS
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

// --- Data Structures ---
enum UIState { STATE_HOME, STATE_MENU, STATE_MEASURE };
enum ParamType { PARAM_NONE = 0, PARAM_BPM, PARAM_SPO2, PARAM_BOTH, PARAM_TEMP, PARAM_ECG };

// THE MEDICAL PAYLOAD (Must match CYD exactly)
typedef struct __attribute__((packed)) PatientPayload {
    uint8_t  patient_id;
    float    temperature;
    int16_t  heart_rate;
    int16_t  spo2;
    int16_t  ecg_value;
    bool     ecg_leads_off;
    uint8_t  active_param; 
} PatientPayload;

PatientPayload patientData;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
RTC_DS3231 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
MAX30105 particleSensor;

// --- State Variables ---
volatile int menuIndex = 0;
volatile bool btnPressed = false;
volatile bool uiNeedsUpdate = true;

UIState currentState = STATE_HOME;
ParamType selectedParam = PARAM_NONE;
const int MENU_ITEMS = 7;
const char* menuStrings[] = {"Read BPM", "Read SpO2", "Read Both", "Read Temp", "Read ECG", "Reset Patient", "<- Home"};

// --- Sensor Buffers & Filters ---
uint32_t irBuffer[100]; 
uint32_t redBuffer[100];
int32_t spo2Value, heartRateValue;
int8_t validSPO2, validHeartRate;

float smoothedBPM = 0;  
float smoothedSpO2 = 0;

unsigned long lastTempRequest = 0;
bool tempRequested = false;

// --- Timers ---
unsigned long lastSDLogMillis = 0;
unsigned long lastClockUpdate = 0; 
unsigned long lastTxMillis = 0;
unsigned long lastEcgTxMillis = 0;
const unsigned long SD_LOG_INTERVAL_MS = 5000; 
const unsigned long TX_INTERVAL_MS = 10000; 

// ==========================================
// INTERRUPTS 
// ==========================================
const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void IRAM_ATTR encoderISR() {
    static uint8_t old_AB = 3; 
    static int enc_val = 0;
    
    old_AB <<= 2;
    old_AB |= ((digitalRead(ROTARY_CLK) << 1) | digitalRead(ROTARY_DT));
    enc_val += enc_states[(old_AB & 0x0f)];
    
    portENTER_CRITICAL_ISR(&mux);
    if (enc_val > 3) {
        if (currentState == STATE_MENU) { menuIndex = (menuIndex + 1) % MENU_ITEMS; }
        enc_val = 0; uiNeedsUpdate = true;
    } else if (enc_val < -3) {
        if (currentState == STATE_MENU) { menuIndex = (menuIndex - 1 + MENU_ITEMS) % MENU_ITEMS; }
        enc_val = 0; uiNeedsUpdate = true;
    }
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR btnISR() {
    static unsigned long lastBtnTime = 0;
    unsigned long now = millis();
    if (now - lastBtnTime > 250) { 
        if (digitalRead(ROTARY_SW) == LOW) { 
            btnPressed = true; uiNeedsUpdate = true;
        }
        lastBtnTime = now;
    }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);

    // 1. DISPLAY INIT
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); 
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(15, 20);
    display.print("BOOTING");
    display.display();
    
    // 2. HARDWARE INIT
    rtc.begin();
    ds18b20.begin();
    ds18b20.setWaitForConversion(false); 
    SD.begin(SD_CS);

    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        particleSensor.setup(60, 4, 2, 100, 411, 4096); 
    }

    // 3. WIFI / ESP-NOW INIT (From Validated Test)
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); 
    WiFi.disconnect(true, true); 
    delay(200);
    
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() == ESP_OK) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, CYD_MAC_ADDRESS, 6);
        peerInfo.channel = ESPNOW_CHANNEL; 
        peerInfo.encrypt = false;
        peerInfo.ifidx = WIFI_IF_STA;
        esp_now_add_peer(&peerInfo);
        Serial.println("ESP-NOW Ready");
    }

    // 4. PINS & INTERRUPTS
    pinMode(ECG_LO_PLUS, INPUT);
    pinMode(ECG_LO_MINUS, INPUT);
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_DT,  INPUT_PULLUP);
    pinMode(ROTARY_SW,  INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_DT),  encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW),  btnISR, FALLING);

    memset(&patientData, 0, sizeof(patientData));
    patientData.patient_id = 1;
    
    delay(500); 
    display.clearDisplay();
    display.display();
}

// ==========================================
// MAIN LOOP & CORE FUNCTIONS
// ==========================================
void loop() {
    if (millis() - lastClockUpdate > 1000) {
        uiNeedsUpdate = true;
        lastClockUpdate = millis();
    }
    
    handleUI();
    readSensors();
    handleTransmission();
    
    if (millis() - lastSDLogMillis > SD_LOG_INTERVAL_MS) {
        lastSDLogMillis = millis();
        logDataToSD();
    }
}

void handleUI() {
    if (btnPressed) {
        portENTER_CRITICAL(&mux);
        btnPressed = false;
        portEXIT_CRITICAL(&mux);
        
        if (currentState == STATE_HOME) { currentState = STATE_MENU; menuIndex = 0; } 
        else if (currentState == STATE_MENU) {
            if (menuIndex == 5) { patientData.patient_id++; currentState = STATE_HOME; } 
            else if (menuIndex == 6) { currentState = STATE_HOME; } 
            else { 
                selectedParam = (ParamType)(menuIndex + 1); 
                currentState = STATE_MEASURE; 
                if (selectedParam == PARAM_BPM || selectedParam == PARAM_SPO2 || selectedParam == PARAM_BOTH) {
                    particleSensor.clearFIFO();
                    smoothedBPM = 0;
                    smoothedSpO2 = 0;
                }
            }
        } 
        else if (currentState == STATE_MEASURE) {
            currentState = STATE_MENU; 
            selectedParam = PARAM_NONE;
            patientData.active_param = 0;
            esp_now_send(CYD_MAC_ADDRESS, (uint8_t*)&patientData, sizeof(patientData));
        }
    }

    if (!uiNeedsUpdate) return;
    uiNeedsUpdate = false;
    
    display.clearDisplay();
    display.setTextSize(1);
    
    // Status Bar
    display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(2, 2); 
    display.print("BED "); display.print(patientData.patient_id);
    
    DateTime now = rtc.now();
    display.setCursor(95, 2);
    display.printf("%02d:%02d", now.hour(), now.minute());
    
    display.setTextColor(SSD1306_WHITE);
    int bodyY = 16;

    if (currentState == STATE_HOME) {
        display.setCursor(15, 25); display.print("Push knob to Menu");
        display.setCursor(26, 45); display.print("Stay healthy!");
    } 
    else if (currentState == STATE_MENU) {
        int startItem = (menuIndex >= 4) ? (menuIndex - 3) : 0;
        for (int i = 0; i < 4; i++) {
            int actualItem = startItem + i;
            if (actualItem >= MENU_ITEMS) break;
            int yPos = bodyY + (i * 12);
            
            if (actualItem == menuIndex) { 
                display.fillRect(0, yPos - 1, 128, 11, SSD1306_WHITE); 
                display.setTextColor(SSD1306_BLACK); display.setCursor(2, yPos); display.print(">");
            } else {
                display.setTextColor(SSD1306_WHITE); display.setCursor(8, yPos);
            }
            display.setCursor(12, yPos); display.print(menuStrings[actualItem]);
        }
    } 
    else if (currentState == STATE_MEASURE) {
        display.setTextSize(1); display.setCursor(0, bodyY); display.print("Measuring...");
        display.setTextSize(2); 
        
        String bpmStr = (patientData.heart_rate == 0 || smoothedBPM == 0) ? "---" : String(patientData.heart_rate);
        String spo2Str = (patientData.spo2 == 0 || smoothedSpO2 == 0) ? "---" : String(patientData.spo2);
        
        if (selectedParam == PARAM_BPM) { display.setCursor(10, bodyY + 16); display.print(bpmStr); display.print(" bpm"); }
        else if (selectedParam == PARAM_SPO2) { display.setCursor(10, bodyY + 16); display.print(spo2Str); display.print(" %"); }
        else if (selectedParam == PARAM_BOTH) {
            display.setTextSize(1);
            display.setCursor(10, bodyY + 12); display.print("BPM:  "); display.print(bpmStr);
            display.setCursor(10, bodyY + 24); display.print("SpO2: "); display.print(spo2Str); display.print(" %");
        }
        else if (selectedParam == PARAM_TEMP) { display.setCursor(10, bodyY + 16); display.print(patientData.temperature, 1); display.print(" C"); }
        else if (selectedParam == PARAM_ECG) { display.setTextSize(1); display.setCursor(10, bodyY + 16); display.print("ECG: "); display.print(patientData.ecg_value); }
        
        display.setTextSize(1); display.setCursor(0, bodyY + 40); display.print("<- Press to stop");
        patientData.active_param = selectedParam;
    }
    display.display();
}

void readSensors() {
    unsigned long currentMillis = millis();

    if (currentState == STATE_MEASURE && (selectedParam == PARAM_BPM || selectedParam == PARAM_SPO2 || selectedParam == PARAM_BOTH)) {
        static int sampleCounter = 0;
        particleSensor.check(); 
        while (particleSensor.available()) {
            memmove(redBuffer, redBuffer + 1, 99 * sizeof(uint32_t));
            memmove(irBuffer, irBuffer + 1, 99 * sizeof(uint32_t));
            redBuffer[99] = particleSensor.getRed(); 
            irBuffer[99] = particleSensor.getIR();
            particleSensor.nextSample(); 
            sampleCounter++;
        }

        if (sampleCounter >= 25) {
            maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2Value, &validSPO2, &heartRateValue, &validHeartRate);
            if (validHeartRate && heartRateValue > 30 && heartRateValue < 220) {
                smoothedBPM = (smoothedBPM == 0) ? heartRateValue : (smoothedBPM * 0.9) + (heartRateValue * 0.1);
                patientData.heart_rate = (int16_t)smoothedBPM;
            }
            if (validSPO2 && spo2Value >= 50 && spo2Value <= 100) {
                smoothedSpO2 = (smoothedSpO2 == 0) ? spo2Value : (smoothedSpO2 * 0.9) + (spo2Value * 0.1);
                patientData.spo2 = (int16_t)smoothedSpO2;
            }
            sampleCounter = 0;
            uiNeedsUpdate = true; 
        }
    }

    if (selectedParam == PARAM_ECG) {
        patientData.ecg_leads_off = (digitalRead(ECG_LO_PLUS) || digitalRead(ECG_LO_MINUS));
        patientData.ecg_value = analogRead(ECG_OUTPUT);
        uiNeedsUpdate = true; 
    }

    if (selectedParam == PARAM_TEMP) {
        if (!tempRequested && currentMillis - lastTempRequest > 1000) {
            ds18b20.requestTemperatures(); 
            lastTempRequest = currentMillis; tempRequested = true;
        } else if (tempRequested && currentMillis - lastTempRequest > 750) {
            float t = ds18b20.getTempCByIndex(0);
            if (t > 10.0 && t < 85.0) { patientData.temperature = t; uiNeedsUpdate = true; }
            tempRequested = false;
        }
    }
}

void handleTransmission() {
    if (currentState != STATE_MEASURE) return;
    unsigned long currentMillis = millis();

    if (selectedParam == PARAM_ECG) {
        if (currentMillis - lastEcgTxMillis >= 20) { 
            esp_now_send(CYD_MAC_ADDRESS, (uint8_t*)&patientData, sizeof(patientData));
            lastEcgTxMillis = currentMillis;
        }
    } else {
        if (currentMillis - lastTxMillis >= TX_INTERVAL_MS) { 
            esp_now_send(CYD_MAC_ADDRESS, (uint8_t*)&patientData, sizeof(patientData));
            lastTxMillis = currentMillis;
        }
    }
}

void logDataToSD() {
    DateTime now = rtc.now();
    File dataFile = SD.open("/log.csv", FILE_APPEND);
    if (dataFile) {
        dataFile.printf("%02d:%02d:%02d,Temp:%.1f,HR:%d,SpO2:%d,ECG:%d\n", 
            now.hour(), now.minute(), now.second(), 
            patientData.temperature, patientData.heart_rate, patientData.spo2, patientData.ecg_value);
        dataFile.close();
    }
}