/**
 * ============================================================
 * PATIENT MONITORING SYSTEM — NURSES' STATION CYD RECEIVER
 * Features: Power Stabilized WiFi + FreeRTOS UI Rendering
 * ============================================================
 */

#define LGFX_SUNTON_ESP32_2432S028
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>
#include "DFRobotDFPlayerMini.h"

#define ESPNOW_CHANNEL 1

// ============================================================
//  CONFIGURATION & THRESHOLDS
// ============================================================
#define EXPECTED_PATIENT_ID   1

#define TRACK_HR_LOW          1
#define TRACK_HR_HIGH         2
#define TRACK_SPO2_LOW        3
#define TRACK_TEMP_LOW        4
#define TRACK_TEMP_HIGH       5
#define TRACK_LEADS_OFF       6

#define ALERT_REPEAT_MS    8000
#define LEADS_ALERT_MS    12000
#define STALE_TIMEOUT_MS   4000

// ============================================================
//  CYD HARDWARE PINS
// ============================================================
#define CYD_LED_RED         17    
#define CYD_LED_GREEN       16
#define CYD_LED_BLUE         4
#define DFPLAYER_ESP_TX     27    
#define DFPLAYER_ESP_RX     22    

// ============================================================
//  SYNCHRONIZED DATA STRUCTURE (Matches Bedside EXACTLY)
// ============================================================
typedef struct __attribute__((packed)) PatientPayload {
    uint8_t  patient_id;
    float    temperature;
    int16_t  heart_rate;
    int16_t  spo2;
    int16_t  ecg_value;
    bool     ecg_leads_off;
    uint8_t  active_param;
} PatientPayload;

volatile PatientPayload sharedBuf;
PatientPayload          localCopy;
volatile bool           newDataFlag = false;
volatile uint32_t       lastRxMs    = 0;
SemaphoreHandle_t       dataMutex;

// ============================================================
//  HARDWARE OBJECTS
// ============================================================
static LGFX              lcd;
HardwareSerial           dfSerial(2);
DFRobotDFPlayerMini      dfPlayer;

// ============================================================
//  COLOUR PALETTE 
// ============================================================
#define C_BG          0x0A0F1A   
#define C_HDR         0x0D2137   
#define C_CARD        0x111827   
#define C_DIVIDER     0x1E2D45   
#define C_ECG_BG      0x020C07   
#define C_ECG_GRID    0x0D2010   
#define C_ECG_TRACE   0x00FF88   
#define C_ECG_WARN    0xFF3333   
#define C_TEXT        0xE2E8F0   
#define C_MUTED       0x64748B   
#define C_HR          0xF87171   
#define C_SPO2        0x60A5FA   
#define C_TEMP        0xFB923C   
#define C_GREEN       0x22C55E   
#define C_ALERT_BG    0x7F1D1D   
#define C_ALERT_TXT   0xFCA5A5   
#define C_YELLOW      0xF59E0B   

#define SCR_W         320
#define SCR_H         240
#define HDR_H          24

#define ECG_X          0
#define ECG_Y          HDR_H
#define ECG_W         216
#define ECG_H         (SCR_H - HDR_H)    

#define SB_X          216
#define SB_Y          HDR_H
#define SB_W          (SCR_W - SB_X)     
#define SB_H          (SCR_H - HDR_H)    

// ============================================================
//  ECG & ALERT STATES
// ============================================================
int16_t  ecgXpos    = 0;
int16_t  ecgLastY   = ECG_H / 2;   
bool     ecgFirst   = true;

static uint8_t  lastAlertType = 0xFF;
static uint32_t lastAlertMs   = 0;

int16_t prev_hr    = -999;
int16_t prev_spo2  = -999;
float   prev_temp  = -999.0f;
bool    prev_crit  = false;
uint8_t prev_alert = 0xFF;
bool    prev_conn  = false;

TaskHandle_t hUI   = nullptr;
TaskHandle_t hData = nullptr;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void uiTask(void *pv);
void dataTask(void *pv);
void drawSplash();
void drawStaticUI();
void drawHeader(bool connected);
void drawECGBackground();
void drawSidebarStatic();
void updateSidebar(bool crit, uint8_t alertT);
void updateAlertPanel(bool crit, uint8_t alertT, bool connected);
void sweepECG();
void playAlert(uint8_t type);
void setRGB(bool r, bool g, bool b);

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    pinMode(CYD_LED_RED,   OUTPUT);
    pinMode(CYD_LED_GREEN, OUTPUT);
    pinMode(CYD_LED_BLUE,  OUTPUT);
    setRGB(false, false, true);  

    // 1. WIFI INITIALIZATION WITH POWER LIMITS (Anti-Brownout)
    WiFi.mode(WIFI_STA);
    delay(200);
    esp_wifi_start();
    
    esp_wifi_set_max_tx_power(8); 
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(500);

    // 2. ESP-NOW INIT
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(onDataRecv);
        Serial.print("CYD MAC: ");
        Serial.println(WiFi.macAddress());
    }

    // 3. HARDWARE INIT
    lcd.init();
    lcd.setRotation(1);           
    lcd.setBrightness(255);
    
    drawSplash();

    dfSerial.begin(9600, SERIAL_8N1, DFPLAYER_ESP_RX, DFPLAYER_ESP_TX);
    delay(800);
    if (dfPlayer.begin(dfSerial)) dfPlayer.volume(25);

    dataMutex = xSemaphoreCreateMutex();
    memset((void *)&sharedBuf, 0, sizeof(PatientPayload));
    memset(&localCopy,         0, sizeof(PatientPayload));

    // 4. DRAW STATIC UI
    drawStaticUI();
    drawHeader(false);

    // 5. LAUNCH FREERTOS TASKS
    xTaskCreatePinnedToCore(uiTask,   "UI",   16000, nullptr, 2, &hUI,   0);
    xTaskCreatePinnedToCore(dataTask, "DATA",  4096, nullptr, 1, &hData, 1);

    setRGB(false, false, false);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); // FreeRTOS idle loop
}

// ============================================================
//  ESP-NOW RECEIVE CALLBACK 
// ============================================================
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(PatientPayload)) return;
    const PatientPayload *pkt = (const PatientPayload *)data;
    if (pkt->patient_id != EXPECTED_PATIENT_ID) return;

    BaseType_t higher = pdFALSE;
    if (xSemaphoreTakeFromISR(dataMutex, &higher) == pdTRUE) {
        memcpy((void *)&sharedBuf, data, sizeof(PatientPayload));
        newDataFlag = true;
        lastRxMs    = millis();
        xSemaphoreGiveFromISR(dataMutex, &higher);
    }
    portYIELD_FROM_ISR(higher);
}

// ============================================================
//  CORE 1 — DATA SYNC TASK
// ============================================================
void dataTask(void *pv) {
    for (;;) {
        if (newDataFlag) {
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                memcpy(&localCopy, (const void *)&sharedBuf, sizeof(PatientPayload));
                newDataFlag = false;
                xSemaphoreGive(dataMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
//  CORE 0 — UI TASK
// ============================================================
void uiTask(void *pv) {
    TickType_t xLast = xTaskGetTickCount();

    for (;;) {
        bool connected = (lastRxMs > 0) && ((millis() - lastRxMs) < STALE_TIMEOUT_MS);

        if (connected != prev_conn) {
            drawHeader(connected);
            prev_conn = connected;
        }

        sweepECG();

        // --- MEDICAL EVALUATOR ---
        bool current_crit = false;
        uint8_t current_alert = 0;
        
        if (connected) {
            if (localCopy.heart_rate > 0 && localCopy.heart_rate < 50) { current_crit = true; current_alert = TRACK_HR_LOW; }
            else if (localCopy.heart_rate > 120) { current_crit = true; current_alert = TRACK_HR_HIGH; }
            else if (localCopy.spo2 > 0 && localCopy.spo2 < 90) { current_crit = true; current_alert = TRACK_SPO2_LOW; }
            else if (localCopy.temperature > 10.0f && localCopy.temperature < 35.0f) { current_crit = true; current_alert = TRACK_TEMP_LOW; }
            else if (localCopy.temperature > 38.0f) { current_crit = true; current_alert = TRACK_TEMP_HIGH; }
        }

        if (localCopy.heart_rate  != prev_hr     ||
            localCopy.spo2        != prev_spo2   ||
            localCopy.temperature != prev_temp   ||
            current_crit          != prev_crit   ||
            current_alert         != prev_alert) {
            
            updateSidebar(current_crit, current_alert);
            updateAlertPanel(current_crit, current_alert, connected);
            
            prev_hr    = localCopy.heart_rate;
            prev_spo2  = localCopy.spo2;
            prev_temp  = localCopy.temperature;
            prev_crit  = current_crit;
            prev_alert = current_alert;
        }

        // --- ALERTS ---
        bool     loOff = localCopy.ecg_leads_off;
        uint32_t now   = millis();

        if (current_crit && current_alert > 0 && current_alert < 6) {
            if (current_alert != lastAlertType || (now - lastAlertMs) > ALERT_REPEAT_MS) {
                playAlert(current_alert);
                lastAlertType = current_alert;
                lastAlertMs   = now;
            }
            static uint32_t blinkT = 0;
            static bool blinkS = false;
            if (now - blinkT > 300) {
                blinkS = !blinkS;
                setRGB(blinkS, false, false);
                blinkT = now;
            }
        } else if (loOff && connected) {
            if (lastAlertType != TRACK_LEADS_OFF || (now - lastAlertMs) > LEADS_ALERT_MS) {
                playAlert(0xFF);
                lastAlertType = TRACK_LEADS_OFF;
                lastAlertMs   = now;
            }
            setRGB(true, false, false);
        } else {
            lastAlertType = 0xFF;
            setRGB(false, false, false);
        }

        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(10));  
    }
}

// ============================================================
//  SPLASH SCREEN
// ============================================================
void drawSplash() {
    lcd.fillScreen(C_BG);
    int cx = SCR_W / 2;
    int cy = 70;
    int pts[][2] = {
        {cx-90,cy}, {cx-60,cy}, {cx-45,cy-25}, {cx-30,cy+35},
        {cx-15,cy-45}, {cx,cy+25}, {cx+15,cy-10}, {cx+30,cy},
        {cx+90,cy}
    };
    for (int i = 1; i < 9; i++) lcd.drawLine(pts[i-1][0], pts[i-1][1], pts[i][0], pts[i][1], C_ECG_TRACE);
    
    lcd.setTextColor(C_TEXT);
    lcd.setFont(&fonts::FreeSansBold12pt7b);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString("Patient Monitor", SCR_W/2, 130);

    lcd.setTextColor(C_ECG_TRACE);
    lcd.setFont(&fonts::FreeSansBold9pt7b);
    lcd.drawString("Conrad Robotics", SCR_W/2, 158);

    lcd.setTextColor(C_MUTED);
    lcd.setFont(&fonts::FreeSans9pt7b);
    lcd.drawString("Nurses Station  |  Made in Nigeria", SCR_W/2, 182);
    delay(2000);
}

// ============================================================
//  UI DRAWING FUNCTIONS
// ============================================================
void drawStaticUI() {
    lcd.fillScreen(C_BG);
    lcd.fillRect(SB_X, SB_Y, SB_W, SB_H, C_CARD);
    lcd.drawFastVLine(SB_X, SB_Y, SB_H, C_DIVIDER);
    drawECGBackground();
    drawSidebarStatic();
}

void drawHeader(bool connected) {
    lcd.fillRect(0, 0, SCR_W, HDR_H, C_HDR);
    lcd.setFont(&fonts::FreeSansBold9pt7b);
    lcd.setTextColor(C_TEXT, C_HDR);
    lcd.setTextDatum(ML_DATUM);
    lcd.drawString("Conrad Robotics", 6, HDR_H / 2);

    uint32_t stCol = connected ? C_GREEN : C_ECG_WARN;
    lcd.fillCircle(SCR_W/2 - 22, HDR_H/2, 4, stCol);
    lcd.setTextColor(stCol, C_HDR);
    lcd.setTextDatum(ML_DATUM);
    lcd.drawString(connected ? "LIVE" : (lastRxMs == 0 ? "WAITING" : "NO SIGNAL"), SCR_W/2 - 14, HDR_H/2);

    char bedStr[10];
    sprintf(bedStr, "BED %02d", EXPECTED_PATIENT_ID);
    lcd.setTextColor(C_TEXT, C_HDR);
    lcd.setTextDatum(MR_DATUM);
    lcd.drawString(bedStr, SCR_W - 4, HDR_H/2);
}

void drawECGBackground() {
    lcd.fillRect(ECG_X, ECG_Y, ECG_W, ECG_H, C_ECG_BG);
    for (int y = 43; y < ECG_H; y += 43) { lcd.drawFastHLine(ECG_X, ECG_Y + y, ECG_W, C_ECG_GRID); }
    for (int x = 43; x < ECG_W; x += 43) { lcd.drawFastVLine(ECG_X + x, ECG_Y, ECG_H, C_ECG_GRID); }
    lcd.drawFastHLine(ECG_X, ECG_Y + ECG_H/2, ECG_W, C_DIVIDER);

    lcd.setTextColor(C_MUTED, C_ECG_BG);
    lcd.setFont(&fonts::FreeSans9pt7b);
    lcd.setTextDatum(TL_DATUM);
    lcd.drawString("ECG", ECG_X + 3, ECG_Y + 3);

    ecgXpos  = 0;
    ecgLastY = ECG_H / 2;
    ecgFirst = true;
}

void drawSidebarStatic() {
    int x0 = SB_X + 6, y = SB_Y + 6;
    lcd.setFont(&fonts::FreeSans9pt7b);
    lcd.setTextColor(C_HR, C_CARD); lcd.setTextDatum(TL_DATUM); lcd.drawString("Heart Rate", x0, y);
    y += 14; lcd.setTextColor(C_MUTED, C_CARD); lcd.drawString("bpm", SB_X + SB_W - 6 - lcd.textWidth("bpm"), y);
    y += 22; lcd.fillRect(x0, y, SB_W - 12, 5, C_DIVIDER); y += 12; lcd.drawFastHLine(x0, y, SB_W - 12, C_DIVIDER);
    y += 8; lcd.setTextColor(C_SPO2, C_CARD); lcd.drawString("SpO2", x0, y);
    y += 14; lcd.setTextColor(C_MUTED, C_CARD); lcd.drawString("%", SB_X + SB_W - 6 - lcd.textWidth("%"), y);
    y += 22; lcd.fillRect(x0, y, SB_W - 12, 5, C_DIVIDER); y += 12; lcd.drawFastHLine(x0, y, SB_W - 12, C_DIVIDER);
    y += 8; lcd.setTextColor(C_TEMP, C_CARD); lcd.drawString("Temp", x0, y);
    y += 14; lcd.setTextColor(C_MUTED, C_CARD); lcd.drawString("\xb0""C", SB_X + SB_W - 6 - lcd.textWidth("\xb0""C"), y);
    y += 22; lcd.fillRect(x0, y, SB_W - 12, 5, C_DIVIDER);
}

void updateSidebar(bool crit, uint8_t alertT) {
    int x0 = SB_X + 6, valW = SB_W - 12, y = SB_Y + 20; char s[10];
    
    // HR
    bool hrCrit = crit && (alertT == 1 || alertT == 2);
    lcd.fillRect(x0, y, valW - lcd.textWidth("bpm") - 4, 18, C_CARD);
    lcd.setFont(&fonts::FreeSansBold12pt7b); lcd.setTextColor(hrCrit ? C_ECG_WARN : C_TEXT, C_CARD); lcd.setTextDatum(ML_DATUM);
    if (localCopy.heart_rate > 0) sprintf(s, "%d", localCopy.heart_rate); else strcpy(s, "---");
    lcd.drawString(s, x0, y + 9);
    y += 22; lcd.fillRect(x0, y, valW, 5, C_DIVIDER);
    if (localCopy.heart_rate > 0) lcd.fillRect(x0, y, map(constrain((int)localCopy.heart_rate, 0, 200), 0, 200, 0, valW), 5, hrCrit ? C_ECG_WARN : C_HR);
    y += 42; 

    // SpO2
    bool spCrit = crit && alertT == 3;
    lcd.fillRect(x0, y, valW - lcd.textWidth("%") - 4, 18, C_CARD);
    lcd.setFont(&fonts::FreeSansBold12pt7b); lcd.setTextColor(spCrit ? C_ECG_WARN : C_TEXT, C_CARD); lcd.setTextDatum(ML_DATUM);
    if (localCopy.spo2 > 0) sprintf(s, "%d", localCopy.spo2); else strcpy(s, "---");
    lcd.drawString(s, x0, y + 9);
    y += 22; lcd.fillRect(x0, y, valW, 5, C_DIVIDER);
    if (localCopy.spo2 > 0) lcd.fillRect(x0, y, map(constrain((int)localCopy.spo2, 80, 100), 80, 100, 0, valW), 5, spCrit ? C_ECG_WARN : C_SPO2);
    y += 42;

    // Temp
    bool tmpCrit = crit && (alertT == 4 || alertT == 5);
    lcd.fillRect(x0, y, valW - lcd.textWidth("\xb0""C") - 4, 18, C_CARD);
    lcd.setFont(&fonts::FreeSansBold12pt7b); lcd.setTextColor(tmpCrit ? C_ECG_WARN : C_TEXT, C_CARD); lcd.setTextDatum(ML_DATUM);
    if (localCopy.temperature > 10.0f) sprintf(s, "%.1f", localCopy.temperature); else strcpy(s, "---");
    lcd.drawString(s, x0, y + 9);
    y += 22; lcd.fillRect(x0, y, valW, 5, C_DIVIDER);
    if (localCopy.temperature > 10.0f) lcd.fillRect(x0, y, map(constrain((int)(localCopy.temperature * 10), 340, 420), 340, 420, 0, valW), 5, tmpCrit ? C_ECG_WARN : C_TEMP);
}

void updateAlertPanel(bool crit, uint8_t alertT, bool connected) {
    int panelY = SB_Y + SB_H - 44, panelH = 44;
    const char *alertMsgs[] = {"STABLE", "HR LOW !", "HR HIGH !", "SpO2 LOW !", "HYPOTHERMIA", "FEVER !"};

    if (crit && alertT > 0 && alertT < 6) {
        lcd.fillRect(SB_X, panelY, SB_W, panelH, C_ALERT_BG); lcd.drawRect(SB_X, panelY, SB_W, panelH, C_ECG_WARN);
        lcd.setTextColor(C_ALERT_TXT, C_ALERT_BG); lcd.setFont(&fonts::FreeSansBold9pt7b); lcd.setTextDatum(MC_DATUM);
        lcd.drawString("! ALERT !", SB_X + SB_W/2, panelY + 14); lcd.setFont(&fonts::FreeSans9pt7b);
        lcd.drawString(alertMsgs[alertT], SB_X + SB_W/2, panelY + 32);
    } else if (localCopy.ecg_leads_off && connected) {
        lcd.fillRect(SB_X, panelY, SB_W, panelH, 0x431407); lcd.drawRect(SB_X, panelY, SB_W, panelH, C_YELLOW);
        lcd.setTextColor(C_YELLOW, 0x431407); lcd.setFont(&fonts::FreeSansBold9pt7b); lcd.setTextDatum(MC_DATUM);
        lcd.drawString("LEADS OFF", SB_X + SB_W/2, panelY + 14); lcd.setFont(&fonts::FreeSans9pt7b);
        lcd.drawString("Check patient", SB_X + SB_W/2, panelY + 32);
    } else {
        lcd.fillRect(SB_X, panelY, SB_W, panelH, C_CARD); lcd.drawRect(SB_X, panelY, SB_W, panelH, C_GREEN);
        lcd.setTextColor(C_GREEN, C_CARD); lcd.setFont(&fonts::FreeSansBold9pt7b); lcd.setTextDatum(MC_DATUM);
        lcd.drawString("STABLE", SB_X + SB_W/2, panelY + 22);
    }
}

void sweepECG() {
    int16_t newY = localCopy.ecg_leads_off ? ECG_H / 2 : (int16_t)map(localCopy.ecg_value, 0, 4095, ECG_H - 12, 8);
    uint32_t traceCol = localCopy.ecg_leads_off ? C_ECG_WARN : C_ECG_TRACE;
    
    int ew = 6; if (ecgXpos + 2 + ew > ECG_W) ew = ECG_W - (ecgXpos + 2);
    if (ew > 0) {
        int16_t ex = ECG_X + ecgXpos + 2; lcd.fillRect(ex, ECG_Y, ew, ECG_H, C_ECG_BG);
        for (int x = ex; x < ex + ew; x++) if ((x - ECG_X) % 43 == 0) lcd.drawFastVLine(x, ECG_Y, ECG_H, C_ECG_GRID);
        for (int y = 43; y < ECG_H; y += 43) lcd.drawFastHLine(ex, ECG_Y + y, ew, C_ECG_GRID);
        lcd.drawFastHLine(ex, ECG_Y + ECG_H/2, ew, C_DIVIDER);
    }

    if (!ecgFirst) lcd.drawLine(ECG_X + ecgXpos - 1, ECG_Y + ecgLastY, ECG_X + ecgXpos, ECG_Y + newY, traceCol);
    else ecgFirst = false;

    ecgLastY = newY; ecgXpos++;
    if (ecgXpos >= ECG_W) { ecgXpos = 0; ecgFirst = true; drawECGBackground(); }
}

void playAlert(uint8_t type) {
    int track = 0;
    switch (type) {
        case 1: track = TRACK_HR_LOW; break; case 2: track = TRACK_HR_HIGH; break;
        case 3: track = TRACK_SPO2_LOW; break; case 4: track = TRACK_TEMP_LOW; break;
        case 5: track = TRACK_TEMP_HIGH; break; case 0xFF: track = TRACK_LEADS_OFF; break;
        default: return;
    }
    dfPlayer.play(track);
}

// ============================================================
//  RGB LED 
// ============================================================
void setRGB(bool r, bool g, bool b) {
    digitalWrite(CYD_LED_RED,   r ? LOW : HIGH);
    digitalWrite(CYD_LED_GREEN, g ? LOW : HIGH);
    digitalWrite(CYD_LED_BLUE,  b ? LOW : HIGH);
}