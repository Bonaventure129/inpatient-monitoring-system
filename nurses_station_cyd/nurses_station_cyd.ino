/**
 * ============================================================
 * PATIENT MONITORING SYSTEM — NURSES' STATION CYD RECEIVER
 * Features: True Black UI + One-Shot Audio State Machine
 * FIXED: Alert gating by active_param + leads-off audio + WiFi/ESPNOW coexistence
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

// FIX: same network creds as bedside unit, so both devices lock to the
// same physical channel automatically (Wi-Fi + ESP-NOW coexistence).
const char* WIFI_SSID = "Jesus is Lord";
const char* WIFI_PASS = "ROBOTICS";

#define ESPNOW_CHANNEL 1   // fallback channel used only until Wi-Fi associates

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

#define STALE_TIMEOUT_MS   4000
#define ECG_X_STEP         3  

// ============================================================
//  CYD HARDWARE PINS
// ============================================================
#define CYD_LED_RED         17    
#define CYD_LED_GREEN       16
#define CYD_LED_BLUE         4
#define DFPLAYER_ESP_TX     27    
#define DFPLAYER_ESP_RX     22    

// ============================================================
//  SYNCHRONIZED DATA STRUCTURE
// ============================================================
typedef struct __attribute__((packed)) PatientPayload {
    uint8_t  patient_id;
    float    temperature;
    int16_t  heart_rate;
    int16_t  spo2;
    int16_t  ecg_value;
    bool     ecg_leads_off;
    uint8_t  active_param;   // 0=NONE 1=BPM 2=SPO2 3=BOTH 4=TEMP 5=ECG
    uint8_t  hour;        
    uint8_t  minute;      
    bool     reset_flag;  
} PatientPayload;

volatile PatientPayload sharedBuf;
PatientPayload          localCopy;
volatile bool           newDataFlag = false;
volatile uint32_t       lastRxMs    = 0;
SemaphoreHandle_t       dataMutex;

static LGFX              lcd;
HardwareSerial           dfSerial(2);
DFRobotDFPlayerMini      dfPlayer;

// ============================================================
//  TRUE BLACK COLOUR PALETTE 
// ============================================================
#define C_BG          0x000000   
#define C_HDR         0x000000   
#define C_CARD        0x000000   
#define C_ECG_BG      0x000000   
#define C_DIVIDER     0x333333   
#define C_ECG_GRID    0x003300   
#define C_ECG_TRACE   0x00FF00   
#define C_ECG_WARN    0xFF3333   
#define C_TEXT        0xFFFFFF   
#define C_MUTED       0x94A3B8   
#define C_HR          0xF87171   
#define C_SPO2        0x60A5FA   
#define C_TEMP        0xFB923C   
#define C_GREEN       0x22C55E   
#define C_ALERT_BG    0x7F1D1D   
#define C_ALERT_TXT   0xFCA5A5   
#define C_YELLOW      0xF59E0B   

#define SCR_W         320
#define SCR_H         240
#define HDR_H          26 

#define ECG_X          0
#define ECG_Y          HDR_H
#define ECG_W         216
#define ECG_H         (SCR_H - HDR_H)    

#define SB_X          216
#define SB_Y          HDR_H
#define SB_W          (SCR_W - SB_X)     
#define SB_H          (SCR_H - HDR_H)    

#define ECG_GRID_SZ    36 

// ============================================================
//  ECG & ALERT STATES
// ============================================================
int16_t  ecgXpos    = 0;
int16_t  ecgLastY   = ECG_H / 2;   
bool     ecgFirst   = true;

static uint8_t  lastAlertType = 0xFF;

int16_t prev_hr    = -999;
int16_t prev_spo2  = -999;
float   prev_temp  = -999.0f;
bool    prev_crit  = false;
uint8_t prev_alert = 0xFF;
bool    prev_conn  = false;
uint8_t prev_min   = 99; 

// FIX: track Wi-Fi connection state for non-blocking association
bool wifiWasConnected = false;

TaskHandle_t hUI   = nullptr;
TaskHandle_t hData = nullptr;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void uiTask(void *pv);
void dataTask(void *pv);
void netTask(void *pv); // FIX: new task for non-blocking Wi-Fi join
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

    WiFi.mode(WIFI_STA);
    delay(200);
    esp_wifi_start();
    
    esp_wifi_set_max_tx_power(8); 
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE); // fallback until Wi-Fi joins

    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(onDataRecv);
    }

    // FIX: kick off non-blocking Wi-Fi join so ESP-NOW keeps working meanwhile
    WiFi.begin(WIFI_SSID, WIFI_PASS);

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

    drawStaticUI();
    drawHeader(false);

    xTaskCreatePinnedToCore(uiTask,   "UI",   16000, nullptr, 2, &hUI,   0);
    xTaskCreatePinnedToCore(dataTask, "DATA",  4096, nullptr, 1, &hData, 1);
    xTaskCreatePinnedToCore(netTask,  "NET",   4096, nullptr, 1, nullptr, 1); // FIX

    setRGB(false, false, false);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); 
}

// ============================================================
// FIX: NON-BLOCKING WI-FI MONITOR
// Once Wi-Fi associates, the radio's channel is locked to the AP's
// channel automatically — matching the bedside unit as long as it
// joins the same network. No manual channel juggling needed.
// ============================================================
void netTask(void *pv) {
    for (;;) {
        bool nowConnected = (WiFi.status() == WL_CONNECTED);
        if (nowConnected && !wifiWasConnected) {
            Serial.print("CYD Wi-Fi connected, channel = ");
            Serial.println(WiFi.channel());
        } else if (!nowConnected && wifiWasConnected) {
            Serial.println("CYD Wi-Fi dropped, falling back to hardcoded channel");
            esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
        }
        wifiWasConnected = nowConnected;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================
//  ESP-NOW RECEIVE
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
//  DATA TASK
// ============================================================
void dataTask(void *pv) {
    for (;;) {
        if (newDataFlag) {
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                memcpy(&localCopy, (const void *)&sharedBuf, sizeof(PatientPayload));
                if (sharedBuf.reset_flag) sharedBuf.reset_flag = false; 
                newDataFlag = false;
                xSemaphoreGive(dataMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
//  UI TASK (WITH ONE-SHOT AUDIO LOGIC)
// ============================================================
void uiTask(void *pv) {
    TickType_t xLast = xTaskGetTickCount();

    for (;;) {
        if (localCopy.reset_flag) {
            localCopy.heart_rate = 0;
            localCopy.spo2 = 0;
            localCopy.temperature = 0;
            localCopy.ecg_value = 0;
            localCopy.ecg_leads_off = false;
            localCopy.active_param = 0; // FIX: also clear active_param on reset
            localCopy.reset_flag = false;
            
            drawStaticUI(); 
            prev_hr = -999; 
            lastAlertType = 0xFF; 
            setRGB(false, false, false);
        }

        bool connected = (lastRxMs > 0) && ((millis() - lastRxMs) < STALE_TIMEOUT_MS);

        if (connected != prev_conn || localCopy.minute != prev_min) {
            drawHeader(connected);
            prev_conn = connected;
            prev_min = localCopy.minute;
        }

        sweepECG();

        bool current_crit = false;
        uint8_t current_alert = 0;
        
        // FIX: gate every check by which parameter is actually being measured,
        // so stale HR/SpO2/Temp values left over from a previous mode can
        // never trigger a false alert.
        if (connected) {
            uint8_t ap = localCopy.active_param; // 1=BPM 2=SPO2 3=BOTH 4=TEMP 5=ECG
            bool checkHR   = (ap == 1 || ap == 3);
            bool checkSpO2 = (ap == 2 || ap == 3);
            bool checkTemp = (ap == 4);

            if (checkHR && localCopy.heart_rate > 0 && localCopy.heart_rate < 50) {
                current_crit = true; current_alert = TRACK_HR_LOW;
            } else if (checkHR && localCopy.heart_rate > 120) {
                current_crit = true; current_alert = TRACK_HR_HIGH;
            } else if (checkSpO2 && localCopy.spo2 > 0 && localCopy.spo2 < 90) {
                current_crit = true; current_alert = TRACK_SPO2_LOW;
            } else if (checkTemp && localCopy.temperature > 10.0f && localCopy.temperature < 35.0f) {
                current_crit = true; current_alert = TRACK_TEMP_LOW;
            } else if (checkTemp && localCopy.temperature > 38.0f) {
                current_crit = true; current_alert = TRACK_TEMP_HIGH;
            }
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

        bool loOff = localCopy.ecg_leads_off && (localCopy.active_param == 5); // FIX: only matters during ECG mode

        // --- REPEATING AUDIO STATE MACHINE ---
        uint8_t target_alert = 0; // 0 = Stable State
        
        if (!connected) {
            target_alert = 99; // Disconnected State
        } else if (current_crit && current_alert > 0 && current_alert < 6) {
            target_alert = current_alert;
        } else if (loOff) {
            target_alert = TRACK_LEADS_OFF;
        }

        static uint32_t lastAudioPlayMs = 0;

        // Trigger Audio when the state changes OR if 30 seconds have passed while in a critical state
        if (target_alert != lastAlertType || 
           (target_alert > 0 && target_alert <= 6 && (millis() - lastAudioPlayMs >= 30000))) {
            
            if (target_alert >= 1 && target_alert <= 6) {
                playAlert(target_alert);
                lastAudioPlayMs = millis(); // Reset the 30-second timer
            } else if (target_alert == 99 || target_alert == 0) {
                dfPlayer.stop(); // Silence if disconnected or stable
            }
            lastAlertType = target_alert;
        }

        // --- LED BLINKING LOGIC ---
        uint32_t now = millis();
        if (target_alert >= 1 && target_alert <= 5) {
            static uint32_t blinkT = 0;
            static bool blinkS = false;
            if (now - blinkT > 300) {
                blinkS = !blinkS;
                setRGB(blinkS, false, false);
                blinkT = now;
            }
        } else if (target_alert == TRACK_LEADS_OFF) {
            setRGB(true, false, false); // Solid Red
        } else {
            setRGB(false, false, false); // LED Off
        }

        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(15));  
    }
}

// ============================================================
//  UI DRAWING FUNCTIONS
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

void drawStaticUI() {
    lcd.fillScreen(C_BG);
    lcd.fillRect(SB_X, SB_Y, SB_W, SB_H, C_CARD);
    lcd.drawFastVLine(SB_X, SB_Y, SB_H, C_DIVIDER);
    drawECGBackground();
    drawSidebarStatic();
}

void drawHeader(bool connected) {
    lcd.fillRect(0, 0, SCR_W, HDR_H, C_HDR);
    lcd.drawFastHLine(0, HDR_H - 1, SCR_W, C_DIVIDER); 

    lcd.setFont(&fonts::FreeSansBold9pt7b);
    lcd.setTextColor(C_TEXT, C_HDR);
    lcd.setTextDatum(ML_DATUM);
    lcd.drawString("Conrad", 6, HDR_H / 2);

    uint32_t stCol = connected ? C_GREEN : C_ECG_WARN;
    char timeStr[32];
    if (connected) sprintf(timeStr, "LIVE  %02d:%02d", localCopy.hour, localCopy.minute);
    else strcpy(timeStr, (lastRxMs == 0 ? "WAITING" : "NO SIGNAL"));
    
    lcd.setFont(&fonts::FreeSans9pt7b);
    int cx = SCR_W / 2;
    int txtW = lcd.textWidth(timeStr);
    
    lcd.fillCircle(cx - (txtW/2) - 10, HDR_H/2, 4, stCol);
    lcd.setTextColor(stCol, C_HDR);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString(timeStr, cx, HDR_H/2);

    char bedStr[10];
    sprintf(bedStr, "BED %02d", EXPECTED_PATIENT_ID);
    lcd.setTextColor(C_TEXT, C_HDR);
    lcd.setTextDatum(MR_DATUM);
    lcd.drawString(bedStr, SCR_W - 6, HDR_H/2);
}

void drawECGBackground() {
    lcd.fillRect(ECG_X, ECG_Y, ECG_W, ECG_H, C_ECG_BG);
    
    for (int y = ECG_GRID_SZ; y < ECG_H; y += ECG_GRID_SZ) { lcd.drawFastHLine(ECG_X, ECG_Y + y, ECG_W, C_ECG_GRID); }
    for (int x = ECG_GRID_SZ; x < ECG_W; x += ECG_GRID_SZ) { lcd.drawFastVLine(ECG_X + x, ECG_Y, ECG_H, C_ECG_GRID); }
    lcd.drawFastHLine(ECG_X, ECG_Y + ECG_H/2, ECG_W, C_DIVIDER); 

    lcd.setTextColor(C_MUTED, C_ECG_BG);
    lcd.setFont(&fonts::FreeSans9pt7b);
    lcd.setTextDatum(TL_DATUM);
    lcd.drawString("ECG", ECG_X + 4, ECG_Y + 4);

    ecgXpos  = 0;
    ecgLastY = ECG_H / 2;
    ecgFirst = true;
}

void drawSidebarStatic() {
    int x_margin = SB_X + 8;
    int y = SB_Y + 10;
    
    lcd.setFont(&fonts::FreeSans9pt7b);
    lcd.setTextDatum(TL_DATUM);
    
    lcd.setTextColor(C_MUTED, C_CARD); 
    lcd.drawString("HR (bpm)", x_margin, y);
    lcd.drawFastHLine(SB_X, y + 46, SB_W, C_DIVIDER);
    
    y += 56;
    lcd.drawString("SpO2 (%)", x_margin, y);
    lcd.drawFastHLine(SB_X, y + 46, SB_W, C_DIVIDER);
    
    y += 56;
    lcd.drawString("Temp (\xb0""C)", x_margin, y);
}

void updateSidebar(bool crit, uint8_t alertT) {
    int x_center = SB_X + (SB_W / 2);
    int y = SB_Y + 36;
    char s[10];
    
    lcd.setFont(&fonts::FreeSansBold12pt7b); 
    lcd.setTextDatum(MC_DATUM);

    // HR
    bool hrCrit = crit && (alertT == 1 || alertT == 2);
    lcd.fillRect(SB_X + 2, y - 12, SB_W - 4, 22, C_CARD); 
    lcd.setTextColor(hrCrit ? C_ECG_WARN : C_HR, C_CARD); 
    if (localCopy.heart_rate > 0) sprintf(s, "%d", localCopy.heart_rate); else strcpy(s, "---");
    lcd.drawString(s, x_center, y);
    
    // SpO2
    y += 56;
    bool spCrit = crit && alertT == 3;
    lcd.fillRect(SB_X + 2, y - 12, SB_W - 4, 22, C_CARD);
    lcd.setTextColor(spCrit ? C_ECG_WARN : C_SPO2, C_CARD); 
    if (localCopy.spo2 > 0) sprintf(s, "%d", localCopy.spo2); else strcpy(s, "---");
    lcd.drawString(s, x_center, y);
    
    // Temp
    y += 56;
    bool tmpCrit = crit && (alertT == 4 || alertT == 5);
    lcd.fillRect(SB_X + 2, y - 12, SB_W - 4, 22, C_CARD);
    lcd.setTextColor(tmpCrit ? C_ECG_WARN : C_TEMP, C_CARD); 
    if (localCopy.temperature > 10.0f) sprintf(s, "%.1f", localCopy.temperature); else strcpy(s, "---");
    lcd.drawString(s, x_center, y);
}

void updateAlertPanel(bool crit, uint8_t alertT, bool connected) {
    int panelY = SB_Y + SB_H - 44, panelH = 44;
    const char *alertMsgs[] = {"STABLE", "HR LOW !", "HR HIGH !", "SpO2 LOW !", "HYPOTHERMIA", "FEVER !"};

    if (crit && alertT > 0 && alertT < 6) {
        lcd.fillRect(SB_X, panelY, SB_W, panelH, C_ALERT_BG); lcd.drawRect(SB_X, panelY, SB_W, panelH, C_ECG_WARN);
        lcd.setTextColor(C_ALERT_TXT, C_ALERT_BG); lcd.setFont(&fonts::FreeSansBold9pt7b); lcd.setTextDatum(MC_DATUM);
        lcd.drawString("! ALERT !", SB_X + SB_W/2, panelY + 14); lcd.setFont(&fonts::FreeSans9pt7b);
        lcd.drawString(alertMsgs[alertT], SB_X + SB_W/2, panelY + 32);
    } else if (localCopy.ecg_leads_off && connected && localCopy.active_param == 5) { // FIX: gate by active_param too
        lcd.fillRect(SB_X, panelY, SB_W, panelH, 0x431407); lcd.drawRect(SB_X, panelY, SB_W, panelH, C_YELLOW);
        lcd.setTextColor(C_YELLOW, 0x431407); lcd.setFont(&fonts::FreeSansBold9pt7b); lcd.setTextDatum(MC_DATUM);
        lcd.drawString("LEADS OFF", SB_X + SB_W/2, panelY + 14); lcd.setFont(&fonts::FreeSans9pt7b);
        lcd.drawString("Check patient", SB_X + SB_W/2, panelY + 32);
    } else {
        lcd.fillRect(SB_X, panelY, SB_W, panelH, C_CARD); lcd.drawRect(SB_X, panelY, SB_W, panelH, C_DIVIDER);
        lcd.setTextColor(C_GREEN, C_CARD); lcd.setFont(&fonts::FreeSansBold9pt7b); lcd.setTextDatum(MC_DATUM);
        lcd.drawString("STABLE", SB_X + SB_W/2, panelY + 22);
    }
}

void sweepECG() {
    int16_t newY = localCopy.ecg_leads_off ? ECG_H / 2 : (int16_t)map(localCopy.ecg_value, 0, 4095, ECG_H - 30, 30);
    uint32_t traceCol = localCopy.ecg_leads_off ? C_ECG_WARN : C_ECG_TRACE;
    
    int eraserWidth = 15; 
    int ex = ECG_X + ecgXpos + 2;
    if (ex + eraserWidth <= ECG_X + ECG_W) {
        lcd.fillRect(ex, ECG_Y, eraserWidth, ECG_H, C_ECG_BG);
        for (int x = ex; x < ex + eraserWidth; x++) if ((x - ECG_X) % ECG_GRID_SZ == 0) lcd.drawFastVLine(x, ECG_Y, ECG_H, C_ECG_GRID);
        for (int y = ECG_GRID_SZ; y < ECG_H; y += ECG_GRID_SZ) lcd.drawFastHLine(ex, ECG_Y + y, eraserWidth, C_ECG_GRID);
        lcd.drawFastHLine(ex, ECG_Y + ECG_H/2, eraserWidth, C_DIVIDER);
    }

    if (!ecgFirst) {
        lcd.drawLine(ECG_X + ecgXpos - ECG_X_STEP, ECG_Y + ecgLastY,     ECG_X + ecgXpos, ECG_Y + newY,     traceCol);
        lcd.drawLine(ECG_X + ecgXpos - ECG_X_STEP, ECG_Y + ecgLastY + 1, ECG_X + ecgXpos, ECG_Y + newY + 1, traceCol);
    } else {
        ecgFirst = false;
    }

    ecgLastY = newY; 
    ecgXpos += ECG_X_STEP;
    
    if (ecgXpos >= ECG_W - ECG_X_STEP) { 
        ecgXpos = 0; 
        ecgFirst = true; 
        drawECGBackground(); 
    }
}

// FIX: added case 6 so leads-off audio actually plays (previously fell to default: return)
void playAlert(uint8_t type) {
    int track = 0;
    switch (type) {
        case 1: track = TRACK_HR_LOW; break; 
        case 2: track = TRACK_HR_HIGH; break;
        case 3: track = TRACK_SPO2_LOW; break; 
        case 4: track = TRACK_TEMP_LOW; break;
        case 5: track = TRACK_TEMP_HIGH; break; 
        case 6: track = TRACK_LEADS_OFF; break; // FIX: was missing
        default: return;
    }
    dfPlayer.play(track);
}

void setRGB(bool r, bool g, bool b) {
    digitalWrite(CYD_LED_RED,   r ? LOW : HIGH);
    digitalWrite(CYD_LED_GREEN, g ? LOW : HIGH);
    digitalWrite(CYD_LED_BLUE,  b ? LOW : HIGH);
}