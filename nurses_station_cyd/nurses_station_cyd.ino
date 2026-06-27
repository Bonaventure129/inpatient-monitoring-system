/**
 * ============================================================
 * PATIENT MONITORING SYSTEM — NURSES' STATION CYD RECEIVER
 * Device  : ESP32-2432S028R (Cheap Yellow Display)
 * Display : LovyanGFX (LGFX_SUNTON_ESP32_2432S028 autodetect)
 * Author  : Conrad Robotics
 * ============================================================
 *
 * CORE SPLIT:
 * Core 0 → Display rendering + DFPlayer audio alerts
 * Core 1 → ESP-NOW receive + data sync
 *
 * REQUIRED LIBRARIES (Library Manager):
 * - LovyanGFX        (by lovyan03)
 * - DFRobotDFPlayerMini
 *
 * DFPlayer Mini wiring:
 * VCC  → 3.3V       GND → GND
 * RX   → GPIO 27    (via 1kΩ resistor)
 * TX   → GPIO 22
 * SPK+ / SPK-  → speaker
 *
 * DFPlayer SD card (FAT32, root folder):
 * 0001.mp3 — "Heart rate critically low"
 * 0002.mp3 — "Heart rate critically high"
 * 0003.mp3 — "Oxygen level critically low"
 * 0004.mp3 — "Temperature critically low"
 * 0005.mp3 — "Fever detected"
 * 0006.mp3 — "ECG leads disconnected"
 * ============================================================
 */

// ── LovyanGFX autodetect for CYD ─────────────────────────
#define LGFX_SUNTON_ESP32_2432S028
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>
#include "DFRobotDFPlayerMini.h"

// ============================================================
//  CONFIGURATION
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
#define CYD_LED_RED         17    // Onboard RGB LED (active LOW)
#define CYD_LED_GREEN       16
#define CYD_LED_BLUE         4
#define DFPLAYER_ESP_TX     27    // → DFPlayer RX (1kΩ in series)
#define DFPLAYER_ESP_RX     22    // ← DFPlayer TX

// ============================================================
//  SHARED DATA STRUCTURE  (byte-for-byte = Bedside Unit)
// ============================================================
typedef struct __attribute__((packed)) PatientPayload {
    uint8_t  patient_id;
    float    temperature;
    int16_t  heart_rate;
    int16_t  spo2;
    int16_t  ecg_value;
    bool     ecg_leads_off;
    uint8_t  active_param;
    bool     is_critical;
    uint8_t  alert_type;
    uint32_t timestamp;
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
static LGFX_Sprite       ecgSprite(&lcd);   // Off-screen ECG canvas
HardwareSerial           dfSerial(2);
DFRobotDFPlayerMini      dfPlayer;

// ============================================================
//  COLOUR PALETTE  (RGB888 packed — lgfx uses 0xRRGGBB)
// ============================================================
#define C_BG          0x0A0F1A   // Deep navy
#define C_HDR         0x0D2137   // Header dark teal
#define C_CARD        0x111827   // Card / sidebar background
#define C_DIVIDER     0x1E2D45   // Subtle divider lines
#define C_ECG_BG      0x020C07   // Very dark ECG background
#define C_ECG_GRID    0x0D2010   // Faint green grid
#define C_ECG_TRACE   0x00FF88   // Bright green trace
#define C_ECG_WARN    0xFF3333   // Red trace (leads off)
#define C_TEXT        0xE2E8F0   // Primary white text
#define C_MUTED       0x64748B   // Labels / units
#define C_HR          0xF87171   // Heart rate red
#define C_SPO2        0x60A5FA   // SpO2 blue
#define C_TEMP        0xFB923C   // Temperature orange
#define C_GREEN       0x22C55E   // Stable / live
#define C_ALERT_BG    0x7F1D1D   // Dark red alert bg
#define C_ALERT_TXT   0xFCA5A5   // Light red alert text
#define C_YELLOW      0xF59E0B   // Leads-off amber
#define C_WHITE       0xFFFFFF
#define C_BLACK       0x000000

// ============================================================
//  SCREEN LAYOUT  (landscape 320 × 240)
//
//  ┌──────────────────────────────────────────────────────┐
//  │          HEADER BAR  (320 × 24)                      │
//  ├──────────────────────────────┬───────────────────────┤
//  │  ECG OSCILLOSCOPE            │  SIDEBAR              │
//  │  (0..215, 24..239)           │  (216..319, 24..239)  │
//  │  216 px wide × 216 px tall   │  104 px wide          │
//  └──────────────────────────────┴───────────────────────┘
//
#define SCR_W         320
#define SCR_H         240
#define HDR_H          24

#define ECG_X          0
#define ECG_Y          HDR_H
#define ECG_W         216
#define ECG_H         (SCR_H - HDR_H)    // 216
#define ECG_MID_ABS   (ECG_Y + ECG_H/2)  // 132

#define SB_X          216
#define SB_Y          HDR_H
#define SB_W          (SCR_W - SB_X)     // 104
#define SB_H          (SCR_H - HDR_H)    // 216

// ============================================================
//  ECG SWEEP STATE
// ============================================================
int16_t  ecgXpos    = 0;
int16_t  ecgLastY   = ECG_H / 2;   // Sprite-relative Y
bool     ecgFirst   = true;

// ============================================================
//  ALERT STATE
// ============================================================
static uint8_t  lastAlertType = 0xFF;
static uint32_t lastAlertMs   = 0;

// ============================================================
//  SIDEBAR REPAINT GUARD  (only redraw when values change)
// ============================================================
int16_t prev_hr    = -999;
int16_t prev_spo2  = -999;
float   prev_temp  = -999.0f;
bool    prev_crit  = false;
uint8_t prev_alert = 0xFF;
bool    prev_conn  = false;
bool    staticDrawn = false;

// ============================================================
//  FREERTOS TASKS
// ============================================================
TaskHandle_t hUI   = nullptr;
TaskHandle_t hData = nullptr;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len);
void uiTask(void *pv);
void dataTask(void *pv);
void drawSplash();
void drawStaticUI();
void drawHeader(bool connected);
void drawECGBackground();
void drawSidebarStatic();
void updateSidebar();
void updateAlertPanel();
void sweepECG();
void playAlert(uint8_t type);
void setRGB(bool r, bool g, bool b);

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n=== NURSES STATION CYD — Conrad Robotics ==="));

    // ── RGB LED (active LOW) ──────────────────────────────
    pinMode(CYD_LED_RED,   OUTPUT);
    pinMode(CYD_LED_GREEN, OUTPUT);
    pinMode(CYD_LED_BLUE,  OUTPUT);
    setRGB(false, false, true);   // Blue during boot

    // ── LovyanGFX init ───────────────────────────────────
    // The LGFX_SUNTON_ESP32_2432S028 define handles ALL pin
    // configuration including backlight — no manual pinMode needed
    lcd.init();
    lcd.setRotation(1);           // Landscape: 320 × 240
    lcd.setBrightness(255);
    lcd.setTextSize(1);

    // ── Splash ───────────────────────────────────────────
    drawSplash();

    // ── DFPlayer ─────────────────────────────────────────
    dfSerial.begin(9600, SERIAL_8N1, DFPLAYER_ESP_RX, DFPLAYER_ESP_TX);
    delay(800);
    if (dfPlayer.begin(dfSerial)) {
        dfPlayer.volume(25);
        Serial.println(F("[DFPlayer] OK"));
    } else {
        Serial.println(F("[DFPlayer] FAIL — check wiring/SD"));
    }

    // ── ESP-NOW ──────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(onDataRecv);
        Serial.print(F("[ESP-NOW] Ready — CYD MAC: "));
        Serial.println(WiFi.macAddress());
        Serial.println(F("  ↑ Paste this into CYD_MAC_ADDRESS[] on the Bedside Unit!"));
    } else {
        Serial.println(F("[ESP-NOW] FAIL"));
    }

    // ── FreeRTOS mutex + defaults ─────────────────────────
    dataMutex = xSemaphoreCreateMutex();
    memset((void *)&sharedBuf, 0, sizeof(PatientPayload));
    memset(&localCopy,         0, sizeof(PatientPayload));

    // ── ECG sprite (off-screen canvas) ───────────────────
    ecgSprite.setColorDepth(16);
    ecgSprite.createSprite(ECG_W, ECG_H);
    ecgSprite.fillSprite(C_ECG_BG);

    // ── Draw initial UI ───────────────────────────────────
    drawStaticUI();
    drawHeader(false);
    staticDrawn = true;

    // ── FreeRTOS tasks ───────────────────────────────────
    xTaskCreatePinnedToCore(uiTask,   "UI",   12000, nullptr, 2, &hUI,   0);
    xTaskCreatePinnedToCore(dataTask, "DATA",  4096, nullptr, 1, &hData, 1);

    setRGB(false, false, false);
    Serial.println(F("[BOOT] Complete"));
    vTaskDelete(nullptr);
}

void loop() {}   // Never reached

// ============================================================
//  ESP-NOW RECEIVE CALLBACK  (Core 3.x API)
// ============================================================
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len) {
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
                memcpy(&localCopy, (const void *)&sharedBuf,
                       sizeof(PatientPayload));
                newDataFlag = false;
                xSemaphoreGive(dataMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
//  CORE 0 — UI TASK  (display + audio)
// ============================================================
void uiTask(void *pv) {
    TickType_t xLast = xTaskGetTickCount();

    for (;;) {
        bool connected = (lastRxMs > 0) &&
                         ((millis() - lastRxMs) < STALE_TIMEOUT_MS);

        // Header — redraw only on connection state change
        if (connected != prev_conn) {
            drawHeader(connected);
            prev_conn = connected;
        }

        // ECG — every tick (100 Hz for smooth sweep)
        sweepECG();

        // Sidebar values — only on data change
        if (localCopy.heart_rate  != prev_hr    ||
            localCopy.spo2        != prev_spo2   ||
            localCopy.temperature != prev_temp   ||
            localCopy.is_critical != prev_crit   ||
            localCopy.alert_type  != prev_alert) {
            updateSidebar();
            updateAlertPanel();
            prev_hr    = localCopy.heart_rate;
            prev_spo2  = localCopy.spo2;
            prev_temp  = localCopy.temperature;
            prev_crit  = localCopy.is_critical;
            prev_alert = localCopy.alert_type;
        }

        // Audio + LED alerts
        bool    crit   = localCopy.is_critical;
        uint8_t alertT = localCopy.alert_type;
        bool    loOff  = localCopy.ecg_leads_off;
        uint32_t now   = millis();

        if (crit && alertT > 0 && alertT < 6) {
            if (alertT != lastAlertType ||
                (now - lastAlertMs) > ALERT_REPEAT_MS) {
                playAlert(alertT);
                lastAlertType = alertT;
                lastAlertMs   = now;
            }
            // Red LED blink
            static uint32_t blinkT = 0;
            static bool blinkS = false;
            if (now - blinkT > 300) {
                blinkS = !blinkS;
                setRGB(blinkS, false, false);
                blinkT = now;
            }
        } else if (loOff && connected) {
            if (lastAlertType != TRACK_LEADS_OFF ||
                (now - lastAlertMs) > LEADS_ALERT_MS) {
                playAlert(0xFF);
                lastAlertType = TRACK_LEADS_OFF;
                lastAlertMs   = now;
            }
            setRGB(true, false, false);
        } else {
            lastAlertType = 0xFF;
            setRGB(false, false, false);
        }

        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(10));  // 100 Hz
    }
}

// ============================================================
//  SPLASH SCREEN
// ============================================================
void drawSplash() {
    lcd.fillScreen(C_BG);

    // ECG waveform illustration
    int cx = SCR_W / 2;
    int cy = 70;
    int pts[][2] = {
        {cx-90,cy}, {cx-60,cy}, {cx-45,cy-25}, {cx-30,cy+35},
        {cx-15,cy-45}, {cx,cy+25}, {cx+15,cy-10}, {cx+30,cy},
        {cx+90,cy}
    };
    for (int i = 1; i < 9; i++) {
        lcd.drawLine(pts[i-1][0], pts[i-1][1],
                     pts[i][0],   pts[i][1], C_ECG_TRACE);
    }

    // Titles
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

    delay(2500);
}

// ============================================================
//  STATIC UI SKELETON  (drawn once at boot)
// ============================================================
void drawStaticUI() {
    lcd.fillScreen(C_BG);

    // Sidebar background
    lcd.fillRect(SB_X, SB_Y, SB_W, SB_H, C_CARD);

    // Vertical divider
    lcd.drawFastVLine(SB_X, SB_Y, SB_H, C_DIVIDER);

    // ECG area background
    lcd.fillRect(ECG_X, ECG_Y, ECG_W, ECG_H, C_ECG_BG);
    drawECGBackground();

    // Sidebar static labels
    drawSidebarStatic();
}

// ============================================================
//  HEADER BAR
// ============================================================
void drawHeader(bool connected) {
    lcd.fillRect(0, 0, SCR_W, HDR_H, C_HDR);

    // Brand left
    lcd.setFont(&fonts::FreeSansBold9pt7b);
    lcd.setTextColor(C_TEXT, C_HDR);
    lcd.setTextDatum(ML_DATUM);
    lcd.drawString("Conrad Robotics", 6, HDR_H / 2);

    // Connection status centre
    uint32_t stCol = connected ? C_GREEN : C_ECG_WARN;
    lcd.fillCircle(SCR_W/2 - 22, HDR_H/2, 4, stCol);
    lcd.setTextColor(stCol, C_HDR);
    lcd.setTextDatum(ML_DATUM);
    lcd.drawString(connected ? "LIVE" : (lastRxMs == 0 ? "WAITING" : "NO SIGNAL"),
                   SCR_W/2 - 14, HDR_H/2);

    // Bed number right
    char bedStr[10];
    sprintf(bedStr, "BED %02d", EXPECTED_PATIENT_ID);
    lcd.setTextColor(C_TEXT, C_HDR);
    lcd.setTextDatum(MR_DATUM);
    lcd.drawString(bedStr, SCR_W - 4, HDR_H/2);
}

// ============================================================
//  ECG GRID BACKGROUND
// ============================================================
void drawECGBackground() {
    // Draw onto the sprite (off-screen) and push once
    ecgSprite.fillSprite(C_ECG_BG);

    // Horizontal grid every 43 px
    for (int y = 43; y < ECG_H; y += 43) {
        ecgSprite.drawFastHLine(0, y, ECG_W, C_ECG_GRID);
    }
    // Vertical grid every 43 px
    for (int x = 43; x < ECG_W; x += 43) {
        ecgSprite.drawFastVLine(x, 0, ECG_H, C_ECG_GRID);
    }
    // Centre baseline (slightly brighter)
    ecgSprite.drawFastHLine(0, ECG_H/2, ECG_W, C_DIVIDER);

    // Label
    ecgSprite.setTextColor(C_MUTED, C_ECG_BG);
    ecgSprite.setFont(&fonts::FreeSans9pt7b);
    ecgSprite.setTextDatum(TL_DATUM);
    ecgSprite.drawString("ECG", 3, 3);

    ecgSprite.pushSprite(ECG_X, ECG_Y);

    ecgXpos  = 0;
    ecgLastY = ECG_H / 2;
    ecgFirst = true;
}

// ============================================================
//  SIDEBAR STATIC LABELS  (drawn once)
// ============================================================
void drawSidebarStatic() {
    int x0 = SB_X + 6;
    int y  = SB_Y + 6;

    // ── HR ────────────────────────────────────────────────
    lcd.setFont(&fonts::FreeSans9pt7b);
    lcd.setTextColor(C_HR, C_CARD);
    lcd.setTextDatum(TL_DATUM);
    lcd.drawString("Heart Rate", x0, y);

    y += 14;
    lcd.setTextColor(C_MUTED, C_CARD);
    lcd.drawString("bpm", SB_X + SB_W - 6 - lcd.textWidth("bpm"), y);

    y += 22;
    // HR progress bar track
    lcd.fillRect(x0, y, SB_W - 12, 5, C_DIVIDER);

    y += 12;
    lcd.drawFastHLine(x0, y, SB_W - 12, C_DIVIDER);

    // ── SpO2 ──────────────────────────────────────────────
    y += 8;
    lcd.setTextColor(C_SPO2, C_CARD);
    lcd.drawString("SpO2", x0, y);

    y += 14;
    lcd.setTextColor(C_MUTED, C_CARD);
    lcd.drawString("%", SB_X + SB_W - 6 - lcd.textWidth("%"), y);

    y += 22;
    lcd.fillRect(x0, y, SB_W - 12, 5, C_DIVIDER);

    y += 12;
    lcd.drawFastHLine(x0, y, SB_W - 12, C_DIVIDER);

    // ── Temperature ───────────────────────────────────────
    y += 8;
    lcd.setTextColor(C_TEMP, C_CARD);
    lcd.drawString("Temp", x0, y);

    y += 14;
    lcd.setTextColor(C_MUTED, C_CARD);
    lcd.drawString("\xb0""C", SB_X + SB_W - 6 - lcd.textWidth("\xb0""C"), y);

    y += 22;
    lcd.fillRect(x0, y, SB_W - 12, 5, C_DIVIDER);
}

// ============================================================
//  SIDEBAR VALUES UPDATE
// ============================================================
void updateSidebar() {
    bool    crit   = localCopy.is_critical;
    uint8_t alertT = localCopy.alert_type;
    int     x0     = SB_X + 6;
    int     valW   = SB_W - 12;

    int y = SB_Y + 6;

    // ── HR value ──────────────────────────────────────────
    y += 14;   // align with "bpm" line
    bool hrCrit = crit && (alertT == 1 || alertT == 2);
    uint32_t hrCol = hrCrit ? C_ECG_WARN : C_TEXT;
    lcd.fillRect(x0, y, valW - lcd.textWidth("bpm") - 4, 18, C_CARD);
    lcd.setFont(&fonts::FreeSansBold12pt7b);
    lcd.setTextColor(hrCol, C_CARD);
    lcd.setTextDatum(ML_DATUM);
    char s[10];
    if (localCopy.heart_rate > 0)
        sprintf(s, "%d", localCopy.heart_rate);
    else
        strcpy(s, "---");
    lcd.drawString(s, x0, y + 9);

    y += 22;   // bar row
    // HR progress bar fill (0–200 bpm)
    lcd.fillRect(x0, y, valW, 5, C_DIVIDER);
    if (localCopy.heart_rate > 0) {
        int filled = map(constrain((int)localCopy.heart_rate, 0, 200),
                         0, 200, 0, valW);
        lcd.fillRect(x0, y, filled, 5, hrCrit ? C_ECG_WARN : C_HR);
    }

    y += 20;   // divider + gap accounted in static draw
    y += 8;

    // ── SpO2 value ────────────────────────────────────────
    y += 14;
    bool spCrit = crit && alertT == 3;
    uint32_t spCol = spCrit ? C_ECG_WARN : C_TEXT;
    lcd.fillRect(x0, y, valW - lcd.textWidth("%") - 4, 18, C_CARD);
    lcd.setFont(&fonts::FreeSansBold12pt7b);
    lcd.setTextColor(spCol, C_CARD);
    lcd.setTextDatum(ML_DATUM);
    if (localCopy.spo2 > 0)
        sprintf(s, "%d", localCopy.spo2);
    else
        strcpy(s, "---");
    lcd.drawString(s, x0, y + 9);

    y += 22;
    lcd.fillRect(x0, y, valW, 5, C_DIVIDER);
    if (localCopy.spo2 > 0) {
        int filled = map(constrain((int)localCopy.spo2, 80, 100),
                         80, 100, 0, valW);
        lcd.fillRect(x0, y, filled, 5, spCrit ? C_ECG_WARN : C_SPO2);
    }

    y += 20;
    y += 8;

    // ── Temp value ────────────────────────────────────────
    y += 14;
    bool tmpCrit = crit && (alertT == 4 || alertT == 5);
    uint32_t tmpCol = tmpCrit ? C_ECG_WARN : C_TEXT;
    lcd.fillRect(x0, y, valW - lcd.textWidth("\xb0""C") - 4, 18, C_CARD);
    lcd.setFont(&fonts::FreeSansBold12pt7b);
    lcd.setTextColor(tmpCol, C_CARD);
    lcd.setTextDatum(ML_DATUM);
    if (localCopy.temperature > 10.0f)
        sprintf(s, "%.1f", localCopy.temperature);
    else
        strcpy(s, "---");
    lcd.drawString(s, x0, y + 9);

    y += 22;
    lcd.fillRect(x0, y, valW, 5, C_DIVIDER);
    if (localCopy.temperature > 10.0f) {
        int filled = map(constrain((int)(localCopy.temperature * 10),
                                   340, 420), 340, 420, 0, valW);
        lcd.fillRect(x0, y, filled, 5, tmpCrit ? C_ECG_WARN : C_TEMP);
    }
}

// ============================================================
//  ALERT / STABLE PANEL  (bottom of sidebar)
// ============================================================
void updateAlertPanel() {
    bool    crit   = localCopy.is_critical;
    uint8_t alertT = localCopy.alert_type;
    bool    loOff  = localCopy.ecg_leads_off;
    bool    connected = (lastRxMs > 0) &&
                        ((millis() - lastRxMs) < STALE_TIMEOUT_MS);

    int panelY = SB_Y + SB_H - 44;
    int panelH = 44;

    const char *alertMsgs[] = {
        "STABLE",
        "HR LOW !", "HR HIGH !",
        "SpO2 LOW !",
        "HYPOTHERMIA", "FEVER !"
    };

    if (crit && alertT > 0 && alertT < 6) {
        lcd.fillRect(SB_X, panelY, SB_W, panelH, C_ALERT_BG);
        lcd.drawRect(SB_X, panelY, SB_W, panelH, C_ECG_WARN);
        lcd.setTextColor(C_ALERT_TXT, C_ALERT_BG);
        lcd.setFont(&fonts::FreeSansBold9pt7b);
        lcd.setTextDatum(MC_DATUM);
        lcd.drawString("! ALERT !", SB_X + SB_W/2, panelY + 14);
        lcd.setFont(&fonts::FreeSans9pt7b);
        lcd.drawString(alertMsgs[alertT], SB_X + SB_W/2, panelY + 32);

    } else if (loOff && connected) {
        lcd.fillRect(SB_X, panelY, SB_W, panelH, 0x431407);
        lcd.drawRect(SB_X, panelY, SB_W, panelH, C_YELLOW);
        lcd.setTextColor(C_YELLOW, 0x431407);
        lcd.setFont(&fonts::FreeSansBold9pt7b);
        lcd.setTextDatum(MC_DATUM);
        lcd.drawString("LEADS OFF", SB_X + SB_W/2, panelY + 14);
        lcd.setFont(&fonts::FreeSans9pt7b);
        lcd.drawString("Check patient", SB_X + SB_W/2, panelY + 32);

    } else {
        lcd.fillRect(SB_X, panelY, SB_W, panelH, C_CARD);
        lcd.drawRect(SB_X, panelY, SB_W, panelH, C_GREEN);
        lcd.setTextColor(C_GREEN, C_CARD);
        lcd.setFont(&fonts::FreeSansBold9pt7b);
        lcd.setTextDatum(MC_DATUM);
        lcd.drawString("STABLE", SB_X + SB_W/2, panelY + 22);
    }
}

// ============================================================
//  ECG SWEEPING OSCILLOSCOPE
//  Draws directly into the sprite, pushes only the changed column
// ============================================================
void sweepECG() {
    int16_t newY;

    if (localCopy.ecg_leads_off) {
        newY = ECG_H / 2;   // Flatline when leads disconnected
    } else {
        // Map ADC 0–4095 → sprite height (inverted: high ADC = top)
        newY = (int16_t)map(localCopy.ecg_value, 0, 4095,
                            ECG_H - 12, 8);
    }

    uint32_t traceCol = localCopy.ecg_leads_off ? C_ECG_WARN : C_ECG_TRACE;

    int16_t absX = ECG_X + ecgXpos;

    // ── Erase-ahead cursor (6 px wide dark strip) ─────────
    int16_t eraseX = ecgXpos + 2;
    if (eraseX < ECG_W) {
        int16_t ew = min(6, ECG_W - (int)eraseX);
        ecgSprite.fillRect(eraseX, 0, ew, ECG_H, C_ECG_BG);
        // Restore grid lines inside the erased strip
        for (int ex = eraseX; ex < eraseX + ew; ex++) {
            if (ex % 43 == 0)
                ecgSprite.drawFastVLine(ex, 0, ECG_H, C_ECG_GRID);
        }
        for (int gy = 43; gy < ECG_H; gy += 43)
            ecgSprite.drawFastHLine(eraseX, gy, ew, C_ECG_GRID);
        ecgSprite.drawFastHLine(eraseX, ECG_H/2, ew, C_DIVIDER);

        // Push erased strip to screen using LovyanGFX Clip Rect
        lcd.setClipRect(ECG_X + eraseX, ECG_Y, ew, ECG_H);
        ecgSprite.pushSprite(ECG_X, ECG_Y);
        lcd.clearClipRect();
    }

    // ── Draw trace line ───────────────────────────────────
    if (!ecgFirst) {
        ecgSprite.drawLine(ecgXpos - 1, ecgLastY,
                           ecgXpos,     newY, traceCol);
                           
        // Push only the 2-pixel-wide column to the screen using Clip Rect
        lcd.setClipRect(ECG_X + ecgXpos - 1, ECG_Y, 2, ECG_H);
        ecgSprite.pushSprite(ECG_X, ECG_Y);
        lcd.clearClipRect();
    } else {
        ecgFirst = false;
    }

    ecgLastY = newY;
    ecgXpos++;

    if (ecgXpos >= ECG_W) {
        // Wrap: reset and redraw full grid
        ecgXpos  = 0;
        ecgFirst = true;
        drawECGBackground();
    }
}

// ============================================================
//  AUDIO ALERT
// ============================================================
void playAlert(uint8_t type) {
    int track = 0;
    switch (type) {
        case 1:    track = TRACK_HR_LOW;    break;
        case 2:    track = TRACK_HR_HIGH;   break;
        case 3:    track = TRACK_SPO2_LOW;  break;
        case 4:    track = TRACK_TEMP_LOW;  break;
        case 5:    track = TRACK_TEMP_HIGH; break;
        case 0xFF: track = TRACK_LEADS_OFF; break;
        default:   return;
    }
    dfPlayer.play(track);
}

// ============================================================
//  RGB LED  (active LOW on CYD)
// ============================================================
void setRGB(bool r, bool g, bool b) {
    digitalWrite(CYD_LED_RED,   r ? LOW : HIGH);
    digitalWrite(CYD_LED_GREEN, g ? LOW : HIGH);
    digitalWrite(CYD_LED_BLUE,  b ? LOW : HIGH);
}