/*
 * amplificatore_70w.ino
 * =====================================================================
 * Firmware ESP32 WROOM-32 per controller amplificatore HF da 70W
 *
 * Moduli controllati:
 *   1. XF-LPF-HF  — filtro passa-basso a 4 relè (bande 15/10m, 20/17m, 40m, 80m)
 *   2. ATU-100     — accordatore automatico con relè TUNE + LCD I2C intercettato
 *   3. Minipa 70W  — finale con relè PTT e relè switch antenna a 2 posizioni
 *
 * Board target : ESP32 Dev Module
 * Librerie     : WiFi.h, WebServer.h, Preferences.h, Wire.h (tutte standard SDK)
 * Compilazione : Arduino IDE 2.x con esp32 board package ≥ 2.0
 *
 * Autore: firmware generato automaticamente — 2026
 * =====================================================================
 */

// ============================================================
// INCLUDE
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>

// ============================================================
// DEFINIZIONE PIN
// ============================================================
#define PIN_RELAY1_LPF1510   13   // Relè 1 — LPF 15/10m   (HIGH = attivo)
#define PIN_RELAY2_LPF2017   12   // Relè 2 — LPF 20/17m   (HIGH = attivo)
#define PIN_RELAY3_LPF40     14   // Relè 3 — LPF 40m      (HIGH = attivo)
#define PIN_RELAY4_LPF80     27   // Relè 4 — LPF 80m      (HIGH = attivo)
#define PIN_RELAY5_TUNE      32   // Relè 5 — ATU-100 TUNE (HIGH = attivo)
#define PIN_RELAY6_PTT       25   // Relè 6 — Minipa PTT   (HIGH = TX)
#define PIN_RELAY7_ANT       33   // Relè 7 — Switch ANT   (HIGH=ANT1, LOW=ANT2)
#define PIN_LED_STATUS        2   // LED di stato on-board

// ============================================================
// L298N — Ventole di raffreddamento
// ============================================================
#define PIN_FAN1_PWM    26   // ENA  — PWM ventola 1 (ledcAttach 20kHz)
#define PIN_FAN1_DIR     4   // IN1  — direzione ventola 1 (sempre HIGH)
#define PIN_FAN2_PWM    18   // ENB  — PWM ventola 2 (ledcAttach 20kHz)
#define PIN_FAN2_DIR    19   // IN3  — direzione ventola 2 (sempre HIGH)
#define FAN_PWM_FREQ    20000  // 20 kHz (inudibile)
#define FAN_PWM_RES         8  // 8 bit → 0-255


// ============================================================
// TFT SPI Display (es. ILI9341 — non ancora implementato, pin riservati)
// ============================================================
// ATTENZIONE: GPIO 2 condiviso con LED_STATUS; GPIO 18/19 condivisi con L298N
#define PIN_TFT_CS      15   // Chip Select display
#define PIN_TFT_DC       2   // Data/Command  ⚠ condiviso con LED_STATUS
#define PIN_TFT_RST      0   // Reset display (usa -1 se collegato a RST board)
#define PIN_TFT_MOSI    23   // SPI MOSI
#define PIN_TFT_CLK     18   // SPI CLK      ⚠ condiviso con FAN2_PWM
#define PIN_TFT_MISO    19   // SPI MISO     ⚠ condiviso con FAN2_DIR
// Touch XPT2046
#define PIN_TOUCH_CS     5   // Touch Chip Select
#define PIN_TOUCH_IRQ   34   // Touch IRQ (input only)

// ============================================================
// COSTANTI I2C / LCD
// ============================================================
#define I2C_SDA              21
#define I2C_SCL              22
#define LCD_SLAVE_ADDR     0x27   // Indirizzo slave che l'ESP32 simula (= PCF8574 ATU-100)
#define LCD_COLS             16
#define LCD_ROWS              2
#define LCD_BUF_SIZE         64   // Buffer raw I2C

// ============================================================
// COSTANTI FIRMWARE
// ============================================================
#define FIRMWARE_VERSION    "1.0.0"
#define BUILD_DATE          __DATE__
#define BUILD_TIME          __TIME__

#define DEFAULT_AP_SSID     "Minipa-70W"
#define DEFAULT_AP_PASS     "hamradio73"
#define DEFAULT_AP_CHANNEL   1
#define SERIAL_BAUD          9600
#define TUNE_PULSE_MS        100    // durata impulso accordo ATU
#define LED_SLOW_MS         2000   // periodo LED in RX
#define LED_FAST_MS          100   // periodo LED in TX
#define STA_RECONNECT_MS   30000   // tentativo riconnessione router (ms)
#define STATUS_POLL_MS       500   // polling /status lato JS

// Namespace NVS
#define NVS_NS              "minipa70w"

// ============================================================
// VARIABILI GLOBALI
// ============================================================

// --- WiFi ---
String  g_apSSID    = DEFAULT_AP_SSID;
String  g_apPass    = DEFAULT_AP_PASS;
int     g_apChannel = DEFAULT_AP_CHANNEL;
String  g_staSSID   = "";
String  g_staPass   = "";

// --- Stato relè (1=ON, 0=OFF) per i 7 relè ---
// Array a indicizzazione 1-based: g_relay[0] non utilizzato, g_relay[1..7] = i sette relè
// Questa scelta rende il codice più leggibile (relay 1 = g_relay[1], ecc.)
bool g_relay[8]   = {false, false, false, false, false, false, false, false};
// Indici: 1=LPF15/10, 2=LPF20/17, 3=LPF40, 4=LPF80, 5=TUNE, 6=PTT, 7=ANT

// --- Selezione banda e antenna ---
int  g_band = 0;   // 0=nessuna, 1=15/10m, 2=20/17m, 3=40m, 4=80m
int  g_ant  = 1;   // 1=ANT1, 2=ANT2

// --- Impulso TUNE ---
bool         g_tuneActive  = false;
unsigned long g_tuneStartMs = 0;

// --- LED ---
unsigned long g_lastLedMs  = 0;
bool          g_ledState   = false;

// --- LCD I2C slave ---
volatile uint8_t  g_i2cRaw[LCD_BUF_SIZE];
volatile int      g_i2cRawLen  = 0;
volatile bool     g_i2cNewData = false;

// Contenuto display decodificato (ASCII, null-terminated, 16 char + '\0')
char g_lcd1[LCD_COLS + 1] = "                ";
char g_lcd2[LCD_COLS + 1] = "                ";

// Dati estratti dal LCD
float g_watts = 0.0f;
float g_swr   = 1.0f;

// Stato decodifica PCF8574 → LCD HD44780
// Il PCF8574 invia nibble: P7=D7,P6=D6,P5=D5,P4=D4,P3=BL,P2=E,P1=RW,P0=RS
static uint8_t  s_lcdNibble   = 0;
static bool     s_lcdNibbleHigh = false;  // vero quando abbiamo il nibble alto
static bool     s_lcdCursorRow  = false;  // false=riga0, true=riga1
static uint8_t  s_lcdCursorCol  = 0;

// Uptime
unsigned long g_startMs = 0;

// NVS
Preferences g_prefs;

// Web Server
WebServer g_server(80);

// Buffer seriale
String g_serialBuf = "";

// STA riconnessione
unsigned long g_lastStaAttemptMs = 0;

// --- Ventole L298N ---
uint8_t g_fan1Speed = 0;   // 0-255
uint8_t g_fan2Speed = 0;   // 0-255

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void setupPins();
void setupWiFi();
void setupWebServer();
void setupI2CSlave();
void loadPreferences();
void savePreferences();

void setLPF(int band);
void setPTT(bool on);
void setAnt(int ant);
void triggerTune();
void applyRelayPin(int relay, bool state);

void setupFans();
void setFan1(uint8_t speed);
void setFan2(uint8_t speed);

void handleLED();
void handleTunePulse();
void handleSerial();
void processSerialCommand(const String& cmd);
void handleI2CData();
void lcdProcessByte(uint8_t rs, uint8_t data);
void parseLCDContent();
void reconnectSTA();

// Web handlers
void handleRoot();
void handleStatus();
void handleBand();
void handlePTT();
void handleAnt();
void handleTune();
void handleFan();
void handleWiFiConnect();
void handleWiFiDisconnect();
void handleWiFiScan();
void handleAPConfig();
void handleSavePrefs();
void handleReboot();
void handleNotFound();

// Utility
String uptimeStr();

// I2C callback (deve essere fuori da una classe)
void IRAM_ATTR onI2CReceive(int numBytes);

// ============================================================
// PAGINA WEB (PROGMEM)
// ============================================================
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="it">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Minipa 70W</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0a0a0a;color:#e0e0e0;font-family:'Segoe UI',Arial,sans-serif;min-height:100vh}
h1{color:#00ff88;text-align:center;padding:15px 0 5px;font-size:1.4em;letter-spacing:3px}
.subtitle{text-align:center;color:#555;font-size:.78em;padding-bottom:10px;letter-spacing:1px}
.tabs{display:flex;border-bottom:2px solid #1a1a1a;background:#111;position:sticky;top:0;z-index:10}
.tab{flex:1;padding:12px 4px;text-align:center;cursor:pointer;color:#666;font-size:.78em;transition:.2s;border-bottom:3px solid transparent;user-select:none}
.tab.active{color:#00ff88;border-bottom:3px solid #00ff88}
.tab:hover{color:#00cc66}
.content{display:none;padding:14px}
.content.active{display:block}
.section{background:#111;border:1px solid #1e1e1e;border-radius:8px;padding:14px;margin-bottom:14px}
.section h3{color:#00ff88;font-size:.88em;margin-bottom:10px;border-bottom:1px solid #1e1e1e;padding-bottom:6px;letter-spacing:1px}
/* Band buttons */
.band-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.band-btn{padding:20px 10px;font-size:1.05em;font-weight:bold;border:2px solid #2a2a2a;border-radius:10px;background:#151515;color:#888;cursor:pointer;transition:.2s;text-align:center;-webkit-tap-highlight-color:transparent}
.band-btn.active{background:#00ff88;color:#000;border-color:#00ff88;box-shadow:0 0 12px #00ff8866}
.band-btn:active{opacity:.7}
/* PTT */
.ptt-btn{width:100%;padding:30px 10px;font-size:1.8em;font-weight:bold;border:3px solid #880000;border-radius:15px;background:#150000;color:#ff4444;cursor:pointer;user-select:none;-webkit-user-select:none;touch-action:none;transition:.1s;margin:6px 0;letter-spacing:4px;-webkit-tap-highlight-color:transparent}
.ptt-btn.active{background:#cc0000;color:#fff;border-color:#ff4444;box-shadow:0 0 35px #ff000099}
/* ANT switch */
.ant-row{display:flex;align-items:center;gap:16px;padding:6px 0}
.sw-track{width:68px;height:34px;background:#2a2a2a;border-radius:17px;position:relative;cursor:pointer;transition:.3s;border:2px solid #3a3a3a;flex-shrink:0}
.sw-track.on{background:#00ff88;border-color:#00ff88}
.sw-thumb{width:26px;height:26px;background:#ddd;border-radius:50%;position:absolute;top:2px;left:2px;transition:.3s;box-shadow:0 2px 4px #0006}
.sw-track.on .sw-thumb{left:36px}
.ant-label{color:#00ff88;font-weight:bold;font-size:1.1em;min-width:55px}
/* TUNE */
.tune-btn{width:100%;padding:16px;font-size:1.05em;font-weight:bold;border:2px solid #ff8800;border-radius:10px;background:#120800;color:#ff8800;cursor:pointer;transition:.15s;letter-spacing:2px;-webkit-tap-highlight-color:transparent}
.tune-btn.fired{background:#ff8800;color:#000}
/* LCD */
.lcd-display{background:#001800;border:2px solid #00cc55;border-radius:6px;padding:12px 16px;font-family:'Courier New',monospace;font-size:1.1em;color:#00ff88;letter-spacing:2px;text-shadow:0 0 6px #00ff8866;min-height:60px}
.lcd-row{white-space:pre;min-height:1.4em}
/* Meter */
.meter-bar{width:100%;height:22px;background:#1a1a1a;border-radius:11px;overflow:hidden;border:1px solid #2a2a2a}
.meter-fill{height:100%;background:linear-gradient(to right,#00ff88 0%,#aaff00 50%,#ffcc00 75%,#ff4444 100%);border-radius:11px;transition:width .3s;min-width:0}
.meter-labels{display:flex;justify-content:space-between;font-size:.75em;color:#555;margin-bottom:3px}
.swr-box{display:flex;gap:20px;margin-top:12px}
.swr-card{flex:1;background:#0d0d0d;border:1px solid #1e1e1e;border-radius:8px;padding:10px;text-align:center}
.swr-val{font-size:1.8em;font-weight:bold;color:#00ff88;font-family:monospace}
.swr-lbl{font-size:.75em;color:#555;margin-top:2px}
/* Relay grid */
.relay-grid{display:grid;grid-template-columns:1fr 1fr;gap:7px}
.relay-item{display:flex;align-items:center;gap:8px;background:#0d0d0d;border:1px solid #1a1a1a;border-radius:6px;padding:8px 10px}
.relay-led{width:11px;height:11px;border-radius:50%;background:#1e1e1e;flex-shrink:0;transition:.3s}
.relay-led.on{background:#00ff88;box-shadow:0 0 6px #00ff88}
.relay-name{font-size:.75em;color:#777;line-height:1.3}
/* Info rows */
.info-row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #141414;font-size:.82em}
.info-row:last-child{border-bottom:none}
.info-row .lbl{color:#666}
.info-row .val{color:#00ff88;font-family:monospace;text-align:right;word-break:break-all}
/* Form */
.inp-grp{margin-bottom:11px}
.inp-grp label{display:block;color:#777;font-size:.8em;margin-bottom:4px}
.inp-grp input,.inp-grp select{width:100%;padding:8px 11px;background:#141414;border:1px solid #2a2a2a;border-radius:6px;color:#ddd;font-size:.88em}
.inp-grp input:focus,.inp-grp select:focus{outline:none;border-color:#00ff88}
/* Buttons */
.btn{padding:9px 18px;border:1px solid #333;border-radius:6px;background:#141414;color:#ccc;cursor:pointer;font-size:.85em;transition:.2s;-webkit-tap-highlight-color:transparent}
.btn:hover{border-color:#00ff88;color:#00ff88}
.btn-ok{border-color:#00ff88;color:#00ff88}
.btn-ok:hover{background:#00ff88;color:#000}
.btn-danger{border-color:#aa2200;color:#ff4444}
.btn-danger:hover{background:#cc2200;color:#fff}
.btn-full{width:100%;margin-top:4px}
.btn-row{display:flex;gap:9px;margin-bottom:12px}
/* WiFi list */
.wifi-item{padding:10px 12px;border:1px solid #1e1e1e;border-radius:6px;cursor:pointer;margin-bottom:6px;transition:.2s;display:flex;justify-content:space-between;align-items:center;font-size:.85em}
.wifi-item:hover{border-color:#00ff88;color:#00ff88}
.wifi-rssi{font-size:.75em;color:#555}
/* Tables */
.tbl{width:100%;border-collapse:collapse;font-size:.8em}
.tbl th{background:#0d0d0d;color:#00ff88;padding:7px 8px;text-align:left;border-bottom:1px solid #2a2a2a}
.tbl td{padding:6px 8px;border-bottom:1px solid #141414;color:#aaa}
.tbl tr:hover td{background:#0a0a0a}
/* Alerts */
.alert{padding:9px 12px;border-radius:6px;margin-bottom:10px;font-size:.82em}
.alert-ok{background:#001800;border:1px solid #00ff88;color:#00ff88}
.alert-err{background:#1a0000;border:1px solid #cc2200;color:#ff4444}
/* Cmd list */
.cmd-list{font-family:'Courier New',monospace;font-size:.8em;line-height:2.1;color:#888}
.cmd-list .cmd{color:#00ff88}
/* Fan controls */
.fan-row{display:flex;align-items:center;gap:12px;margin-bottom:10px}
.fan-slider{flex:1;-webkit-appearance:none;appearance:none;height:6px;border-radius:3px;background:#2a2a2a;outline:none}
.fan-slider::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;background:#00ff88;cursor:pointer}
.fan-slider::-moz-range-thumb{width:20px;height:20px;border-radius:50%;background:#00ff88;cursor:pointer;border:none}
.fan-val{color:#00ff88;font-family:monospace;font-size:.9em;min-width:38px;text-align:right}
.fan-led{width:11px;height:11px;border-radius:50%;background:#1e1e1e;flex-shrink:0;transition:.3s}
.fan-led.on{background:#00ff88;box-shadow:0 0 6px #00ff88}
</style>
</head>
<body>

<h1>🎛️ MINIPA 70W</h1>
<div class="subtitle">HF AMPLIFIER CONTROLLER — ESP32</div>

<div class="tabs">
  <div class="tab active" onclick="showTab(0)">📡 Dashboard</div>
  <div class="tab" onclick="showTab(1)">📊 Monitor</div>
  <div class="tab" onclick="showTab(2)">⚙️ Impostazioni</div>
  <div class="tab" onclick="showTab(3)">ℹ️ Info</div>
</div>

<!-- ==================== TAB 0: DASHBOARD ==================== -->
<div class="content active" id="tab0">

  <div class="section">
    <h3>🎚️ SELEZIONE BANDA LPF</h3>
    <div class="band-grid">
      <button class="band-btn" id="band4" onclick="setBand(4)">80m</button>
      <button class="band-btn" id="band3" onclick="setBand(3)">40m</button>
      <button class="band-btn" id="band2" onclick="setBand(2)">20/17m</button>
      <button class="band-btn" id="band1" onclick="setBand(1)">15/10m</button>
    </div>
  </div>

  <div class="section">
    <h3>🔴 PTT — Tieni premuto per trasmettere</h3>
    <button class="ptt-btn" id="pttBtn"
      onmousedown="setPTT(1)" onmouseup="setPTT(0)" onmouseleave="setPTT(0)"
      ontouchstart="evt_pttOn(event)" ontouchend="evt_pttOff(event)" ontouchcancel="evt_pttOff(event)">
      PTT
    </button>
  </div>

  <div class="section">
    <h3>🔌 SWITCH ANTENNA</h3>
    <div class="ant-row">
      <div class="sw-track" id="antSwitch" onclick="toggleAnt()">
        <div class="sw-thumb"></div>
      </div>
      <div>
        <div class="ant-label" id="antLabel">ANT 1</div>
        <div style="color:#555;font-size:.78em">Tocca per cambiare</div>
      </div>
    </div>
  </div>

  <div class="section">
    <h3>🎯 ATU-100 ACCORDO</h3>
    <button class="tune-btn" id="tuneBtn" onclick="doTune()">▶ TUNE</button>
  </div>

  <div class="section">
    <h3>🌀 VENTOLE DI RAFFREDDAMENTO (L298N)</h3>
    <div class="fan-row">
      <div class="fan-led" id="fanLed1"></div>
      <span style="color:#777;font-size:.82em;min-width:70px">Ventola 1</span>
      <input type="range" class="fan-slider" id="fan1Slider" min="0" max="100" value="0" oninput="onFan1Input(this.value)">
      <span class="fan-val" id="fan1Val">0%</span>
    </div>
    <div class="fan-row">
      <div class="fan-led" id="fanLed2"></div>
      <span style="color:#777;font-size:.82em;min-width:70px">Ventola 2</span>
      <input type="range" class="fan-slider" id="fan2Slider" min="0" max="100" value="0" oninput="onFan2Input(this.value)">
      <span class="fan-val" id="fan2Val">0%</span>
    </div>
    <div class="btn-row" style="margin-top:10px">
      <button class="btn btn-ok" onclick="setFanBoth(100)">💨 Entrambe 100%</button>
      <button class="btn btn-danger" onclick="setFanBoth(0)">⏹ Spegni</button>
    </div>
  </div>

</div>

<!-- ==================== TAB 1: MONITOR ==================== -->
<div class="content" id="tab1">

  <div class="section">
    <h3>📟 DISPLAY ATU-100 (16×2)</h3>
    <div class="lcd-display">
      <div class="lcd-row" id="lcdL1">                </div>
      <div class="lcd-row" id="lcdL2">                </div>
    </div>
  </div>

  <div class="section">
    <h3>⚡ POTENZA &amp; ROE</h3>
    <div class="meter-labels"><span>0 W</span><span id="wattVal">0.0 W</span><span>70 W</span></div>
    <div class="meter-bar"><div class="meter-fill" id="wattBar" style="width:0%"></div></div>
    <div class="swr-box">
      <div class="swr-card">
        <div class="swr-val" id="swrVal">1.0</div>
        <div class="swr-lbl">SWR / ROE</div>
      </div>
      <div class="swr-card">
        <div class="swr-val" id="wattVal2">0.0</div>
        <div class="swr-lbl">Watt RF</div>
      </div>
    </div>
  </div>

  <div class="section">
    <h3>🔵 STATO RELÈ</h3>
    <div class="relay-grid">
      <div class="relay-item"><div class="relay-led" id="rl1"></div><div class="relay-name">Relè 1<br>LPF 15/10m (GPIO13)</div></div>
      <div class="relay-item"><div class="relay-led" id="rl2"></div><div class="relay-name">Relè 2<br>LPF 20/17m (GPIO12)</div></div>
      <div class="relay-item"><div class="relay-led" id="rl3"></div><div class="relay-name">Relè 3<br>LPF 40m (GPIO14)</div></div>
      <div class="relay-item"><div class="relay-led" id="rl4"></div><div class="relay-name">Relè 4<br>LPF 80m (GPIO27)</div></div>
      <div class="relay-item"><div class="relay-led" id="rl5"></div><div class="relay-name">Relè 5<br>ATU TUNE (GPIO32)</div></div>
      <div class="relay-item"><div class="relay-led" id="rl6"></div><div class="relay-name">Relè 6<br>PTT (GPIO25)</div></div>
      <div class="relay-item"><div class="relay-led" id="rl7"></div><div class="relay-name">Relè 7<br>ANT Switch (GPIO33)</div></div>
    </div>
  </div>

  <div class="section">
    <h3>🌀 STATO VENTOLE</h3>
    <div class="relay-grid">
      <div class="relay-item"><div class="fan-led" id="monFanLed1"></div><div class="relay-name">Ventola 1<br><span id="monFan1Pct">0%</span> (GPIO26 ENA)</div></div>
      <div class="relay-item"><div class="fan-led" id="monFanLed2"></div><div class="relay-name">Ventola 2<br><span id="monFan2Pct">0%</span> (GPIO18 ENB)</div></div>
    </div>
  </div>

  <div class="section">
    <h3>🖥️ INFORMAZIONI SISTEMA</h3>
    <div class="info-row"><span class="lbl">Uptime</span><span class="val" id="sUptime">-</span></div>
    <div class="info-row"><span class="lbl">Free Heap</span><span class="val" id="sHeap">-</span></div>
    <div class="info-row"><span class="lbl">IP Access Point</span><span class="val" id="sApIp">192.168.4.1</span></div>
    <div class="info-row"><span class="lbl">IP Router (STA)</span><span class="val" id="sStaIp">-</span></div>
    <div class="info-row"><span class="lbl">Router SSID</span><span class="val" id="sWifiSta">-</span></div>
    <div class="info-row"><span class="lbl">Banda attiva</span><span class="val" id="sBand">-</span></div>
    <div class="info-row"><span class="lbl">Antenna</span><span class="val" id="sAnt">-</span></div>
  </div>

</div>

<!-- ==================== TAB 2: IMPOSTAZIONI ==================== -->
<div class="content" id="tab2">

  <div id="globalAlert"></div>

  <div class="section">
    <h3>📶 CONNESSIONE ROUTER WIFI</h3>
    <div class="inp-grp">
      <label>SSID Rete</label>
      <input type="text" id="staSSID" placeholder="Nome rete WiFi">
    </div>
    <div class="inp-grp">
      <label>Password</label>
      <input type="password" id="staPass" placeholder="Password WiFi">
    </div>
    <div class="btn-row">
      <button class="btn btn-ok" onclick="connectWiFi()">Connetti</button>
      <button class="btn btn-danger" onclick="disconnectWiFi()">Disconnetti</button>
    </div>
    <button class="btn btn-full" onclick="scanWiFi()">🔍 Scansione reti</button>
    <div id="wifiList" style="margin-top:10px"></div>
  </div>

  <div class="section">
    <h3>📡 CONFIGURAZIONE ACCESS POINT</h3>
    <div class="inp-grp">
      <label>SSID AP</label>
      <input type="text" id="apSSIDi" placeholder="Minipa-70W">
    </div>
    <div class="inp-grp">
      <label>Password AP (min 8 caratteri)</label>
      <input type="text" id="apPassi" placeholder="hamradio73">
    </div>
    <div class="inp-grp">
      <label>Canale (1-13)</label>
      <input type="number" id="apChi" min="1" max="13" value="1">
    </div>
    <button class="btn btn-ok btn-full" onclick="saveAP()">💾 Salva AP (richiede riavvio)</button>
  </div>

  <div class="section">
    <h3>⭐ PREFERENZE AVVIO</h3>
    <div class="inp-grp">
      <label>Banda preferita all'avvio</label>
      <select id="prefBand">
        <option value="0">Nessuna</option>
        <option value="1">15/10m</option>
        <option value="2">20/17m</option>
        <option value="3">40m</option>
        <option value="4">80m</option>
      </select>
    </div>
    <div class="inp-grp">
      <label>Antenna preferita all'avvio</label>
      <select id="prefAnt">
        <option value="1">ANT 1</option>
        <option value="2">ANT 2</option>
      </select>
    </div>
    <button class="btn btn-ok btn-full" onclick="savePrefs()">💾 Salva preferenze</button>
  </div>

  <div class="section">
    <button class="btn btn-danger btn-full" onclick="if(confirm('Riavviare il dispositivo?'))fetch('/reboot').then(()=>showAlert('Riavvio in corso...','ok'))">🔄 Riavvia ESP32</button>
  </div>

</div>

<!-- ==================== TAB 3: INFO ==================== -->
<div class="content" id="tab3">

  <div class="section">
    <h3>📌 PINOUT ESP32 WROOM-32</h3>
    <table class="tbl">
      <tr><th>Funzione</th><th>GPIO</th><th>Dir</th><th>Note</th></tr>
      <tr><td>Relè 1 — LPF 15/10m</td><td>13</td><td>OUT</td><td></td></tr>
      <tr><td>Relè 2 — LPF 20/17m</td><td>12</td><td>OUT</td><td></td></tr>
      <tr><td>Relè 3 — LPF 40m</td><td>14</td><td>OUT</td><td></td></tr>
      <tr><td>Relè 4 — LPF 80m</td><td>27</td><td>OUT</td><td></td></tr>
      <tr><td>Relè 5 — ATU-100 TUNE</td><td>32</td><td>OUT</td><td></td></tr>
      <tr><td>Relè 6 — Minipa PTT</td><td>25</td><td>OUT</td><td></td></tr>
      <tr><td>Relè 7 — Switch ANT</td><td>33</td><td>OUT</td><td></td></tr>
      <tr><td>SDA I2C (LCD ATU-100)</td><td>21</td><td>I2C</td><td></td></tr>
      <tr><td>SCL I2C (LCD ATU-100)</td><td>22</td><td>I2C</td><td></td></tr>
      <tr><td>LED Status</td><td>2</td><td>OUT</td><td>⚠ Condiviso con TFT_DC</td></tr>
      <tr style="color:#00cc66"><td><b>FAN1 PWM (ENA)</b></td><td><b>26</b></td><td>OUT/PWM</td><td>L298N — ledcAttach 20kHz</td></tr>
      <tr style="color:#00cc66"><td><b>FAN1 DIR (IN1)</b></td><td><b>4</b></td><td>OUT</td><td>L298N — sempre HIGH</td></tr>
      <tr style="color:#00cc66"><td><b>FAN2 PWM (ENB)</b></td><td><b>18</b></td><td>OUT/PWM</td><td>L298N — ledcAttach 20kHz ⚠ TFT CLK</td></tr>
      <tr style="color:#00cc66"><td><b>FAN2 DIR (IN3)</b></td><td><b>19</b></td><td>OUT</td><td>L298N — sempre HIGH ⚠ TFT MISO</td></tr>
      <tr style="color:#888"><td>TFT CS</td><td>15</td><td>OUT</td><td>Riservato — non implementato</td></tr>
      <tr style="color:#888"><td>TFT DC</td><td>2</td><td>OUT</td><td>⚠ Conflitto con LED_STATUS</td></tr>
      <tr style="color:#888"><td>TFT RST</td><td>0</td><td>OUT</td><td>Riservato</td></tr>
      <tr style="color:#888"><td>TFT MOSI</td><td>23</td><td>OUT</td><td>Riservato</td></tr>
      <tr style="color:#888"><td>TFT CLK</td><td>18</td><td>OUT</td><td>⚠ Conflitto con FAN2 PWM</td></tr>
      <tr style="color:#888"><td>TFT MISO</td><td>19</td><td>IN</td><td>⚠ Conflitto con FAN2 DIR</td></tr>
      <tr style="color:#888"><td>Touch CS</td><td>5</td><td>OUT</td><td>Riservato</td></tr>
      <tr style="color:#888"><td>Touch IRQ</td><td>34</td><td>IN</td><td>Input only</td></tr>
    </table>
  </div>

  <div class="section">
    <h3>🔧 MODULI COLLEGATI</h3>
    <div class="info-row"><span class="lbl">XF-LPF-HF</span><span class="val">Filtro passa-basso 4 bande (15/10, 20/17, 40, 80m)</span></div>
    <div class="info-row"><span class="lbl">ATU-100</span><span class="val">Accordatore auto + LCD I2C 16×2 @ 0x27</span></div>
    <div class="info-row"><span class="lbl">Minipa 70W PA</span><span class="val">Finale HF 70W — PTT + switch antenna 2 pos.</span></div>
  </div>

  <div class="section">
    <h3>💻 COMANDI SERIALI (9600 baud)</h3>
    <div class="cmd-list">
      <span class="cmd">BAND:80</span> → attiva LPF 80m<br>
      <span class="cmd">BAND:40</span> → attiva LPF 40m<br>
      <span class="cmd">BAND:20</span> → attiva LPF 20/17m<br>
      <span class="cmd">BAND:15</span> → attiva LPF 15/10m<br>
      <span class="cmd">PTT:1</span> → PTT ON (TX)<br>
      <span class="cmd">PTT:0</span> → PTT OFF (RX)<br>
      <span class="cmd">TUNE</span> → impulso accordo ATU 100ms<br>
      <span class="cmd">ANT:1</span> → seleziona Antenna 1<br>
      <span class="cmd">ANT:2</span> → seleziona Antenna 2<br>
      <span class="cmd">FAN1:&lt;0-255&gt;</span> → imposta velocità ventola 1<br>
      <span class="cmd">FAN2:&lt;0-255&gt;</span> → imposta velocità ventola 2<br>
      <span class="cmd">FAN:&lt;0-255&gt;</span> → imposta entrambe le ventole<br>
      <span class="cmd">STATUS?</span> → diagnostica su porta seriale
    </div>
  </div>

  <div class="section">
    <h3>🖥️ DISPLAY TFT TOUCH (PIN RISERVATI)</h3>
    <p style="color:#666;font-size:.8em;margin-bottom:10px">Libreria TFT non ancora inclusa — pin documentati per espansione futura.</p>
    <table class="tbl">
      <tr><th>Segnale</th><th>GPIO</th><th>Nota conflitti</th></tr>
      <tr><td>TFT CS (ILI9341)</td><td>15</td><td>Libero</td></tr>
      <tr><td>TFT DC / RS</td><td>2</td><td>⚠ Condiviso con LED_STATUS</td></tr>
      <tr><td>TFT RST</td><td>0</td><td>Usare -1 se collegato a RST board</td></tr>
      <tr><td>SPI MOSI</td><td>23</td><td>Libero</td></tr>
      <tr><td>SPI CLK</td><td>18</td><td>⚠ Condiviso con L298N FAN2_PWM</td></tr>
      <tr><td>SPI MISO</td><td>19</td><td>⚠ Condiviso con L298N FAN2_DIR</td></tr>
      <tr><td>Touch CS (XPT2046)</td><td>5</td><td>Libero</td></tr>
      <tr><td>Touch IRQ</td><td>34</td><td>Input only — Libero</td></tr>
    </table>
    <p style="color:#886600;font-size:.78em;margin-top:8px">⚠ Per usare TFT contemporaneamente alle ventole, usare pin SPI alternativi liberi (es. GPIO 16/17 per CLK/MISO e GPIO 2 rimappato).</p>
  </div>

  <div class="section">
    <h3>📋 FIRMWARE</h3>
    <div class="info-row"><span class="lbl">Versione</span><span class="val" id="fwVer">-</span></div>
    <div class="info-row"><span class="lbl">Data build</span><span class="val" id="fwDate">-</span></div>
    <div class="info-row"><span class="lbl">Board</span><span class="val">ESP32 Dev Module (ESP32 WROOM-32)</span></div>
    <div class="info-row"><span class="lbl">AP IP</span><span class="val">192.168.4.1</span></div>
    <div class="info-row"><span class="lbl">Web port</span><span class="val">80</span></div>
  </div>

</div><!-- end tabs -->

<script>
// =========== STATO LOCALE ===========
var cBand = 0, cAnt = 1;

// =========== FAN ===========
function onFan1Input(pct) {
  document.getElementById('fan1Val').textContent = pct + '%';
  var speed = Math.round(parseInt(pct) * 255 / 100);
  fetch('/fan?ch=1&v=' + speed).catch(function(){});
}
function onFan2Input(pct) {
  document.getElementById('fan2Val').textContent = pct + '%';
  var speed = Math.round(parseInt(pct) * 255 / 100);
  fetch('/fan?ch=2&v=' + speed).catch(function(){});
}
function setFanBoth(pct) {
  var speed = Math.round(pct * 255 / 100);
  fetch('/fan?ch=0&v=' + speed).catch(function(){});
  document.getElementById('fan1Slider').value = pct;
  document.getElementById('fan2Slider').value = pct;
  document.getElementById('fan1Val').textContent = pct + '%';
  document.getElementById('fan2Val').textContent = pct + '%';
  updateFanUI(speed, speed);
}
function updateFanUI(s1, s2) {
  var led1 = document.getElementById('fanLed1');
  var led2 = document.getElementById('fanLed2');
  if (led1) led1.classList.toggle('on', s1 > 0);
  if (led2) led2.classList.toggle('on', s2 > 0);
  var ml1 = document.getElementById('monFanLed1');
  var ml2 = document.getElementById('monFanLed2');
  if (ml1) ml1.classList.toggle('on', s1 > 0);
  if (ml2) ml2.classList.toggle('on', s2 > 0);
  var p1 = Math.round(s1 / 255 * 100);
  var p2 = Math.round(s2 / 255 * 100);
  var mp1 = document.getElementById('monFan1Pct');
  var mp2 = document.getElementById('monFan2Pct');
  if (mp1) mp1.textContent = p1 + '%';
  if (mp2) mp2.textContent = p2 + '%';
}

// =========== TABS ===========
function showTab(n) {
  document.querySelectorAll('.tab').forEach(function(t,i){ t.classList.toggle('active', i===n); });
  document.querySelectorAll('.content').forEach(function(c,i){ c.classList.toggle('active', i===n); });
}

// =========== BANDA ===========
function setBand(b) {
  fetch('/band?b=' + b).then(function(r){ return r.json(); }).then(function(d){
    cBand = parseInt(d.band);
    updateBandUI();
  }).catch(function(){});
}
function updateBandUI() {
  for (var i = 1; i <= 4; i++) {
    var el = document.getElementById('band' + i);
    if (el) el.classList.toggle('active', i === cBand);
  }
}

// =========== PTT ===========
function setPTT(v) {
  fetch('/ptt?v=' + v).catch(function(){});
  document.getElementById('pttBtn').classList.toggle('active', v === 1);
}
function evt_pttOn(e) { e.preventDefault(); setPTT(1); }
function evt_pttOff(e) { e.preventDefault(); setPTT(0); }

// =========== ANT ===========
function toggleAnt() {
  var newAnt = (cAnt === 1) ? 2 : 1;
  fetch('/ant?v=' + newAnt).then(function(r){ return r.json(); }).then(function(d){
    cAnt = parseInt(d.ant);
    updateAntUI();
  }).catch(function(){});
}
function updateAntUI() {
  var sw = document.getElementById('antSwitch');
  if (sw) sw.classList.toggle('on', cAnt === 1);
  var lbl = document.getElementById('antLabel');
  if (lbl) lbl.textContent = 'ANT ' + cAnt;
}

// =========== TUNE ===========
function doTune() {
  var btn = document.getElementById('tuneBtn');
  btn.classList.add('fired');
  fetch('/tune').catch(function(){});
  setTimeout(function(){ btn.classList.remove('fired'); }, 300);
}

// =========== WIFI SETTINGS ===========
function connectWiFi() {
  var ssid = document.getElementById('staSSID').value.trim();
  var pass = document.getElementById('staPass').value;
  if (!ssid) { showAlert('SSID obbligatorio!', 'err'); return; }
  fetch('/wifi/connect?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass))
    .then(function(r){ return r.json(); })
    .then(function(d){ showAlert(d.msg, d.ok ? 'ok' : 'err'); })
    .catch(function(){ showAlert('Errore di comunicazione', 'err'); });
}
function disconnectWiFi() {
  fetch('/wifi/disconnect').then(function(r){ return r.json(); })
    .then(function(d){ showAlert(d.msg, 'ok'); }).catch(function(){});
}
function scanWiFi() {
  document.getElementById('wifiList').innerHTML = '<div style="color:#555;padding:10px;font-size:.82em">Scansione in corso...</div>';
  fetch('/wifi/scan').then(function(r){ return r.json(); }).then(function(d){
    var h = '';
    d.nets.forEach(function(n){
      h += '<div class="wifi-item" onclick="document.getElementById(\'staSSID\').value=\'' + n.ssid.replace(/'/g,"\\'") + '\'">'
        + '<span>' + n.ssid + '</span><span class="wifi-rssi">' + n.rssi + ' dBm</span></div>';
    });
    document.getElementById('wifiList').innerHTML = h || '<div style="color:#555;padding:10px;font-size:.82em">Nessuna rete trovata</div>';
  }).catch(function(){ document.getElementById('wifiList').innerHTML = ''; });
}
function saveAP() {
  var ssid = document.getElementById('apSSIDi').value.trim();
  var pass = document.getElementById('apPassi').value;
  var ch   = document.getElementById('apChi').value;
  if (!ssid || pass.length < 8) { showAlert('SSID e password (min 8 char) obbligatori!', 'err'); return; }
  fetch('/ap/config?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass) + '&ch=' + ch)
    .then(function(r){ return r.json(); })
    .then(function(d){ showAlert(d.msg, d.ok ? 'ok' : 'err'); }).catch(function(){});
}
function savePrefs() {
  var b = document.getElementById('prefBand').value;
  var a = document.getElementById('prefAnt').value;
  fetch('/prefs/save?band=' + b + '&ant=' + a)
    .then(function(r){ return r.json(); })
    .then(function(d){ showAlert(d.msg, 'ok'); }).catch(function(){});
}
function showAlert(msg, type) {
  var el = document.getElementById('globalAlert');
  if (el) {
    el.innerHTML = '<div class="alert alert-' + type + '">' + msg + '</div>';
    setTimeout(function(){ el.innerHTML = ''; }, 4000);
  }
}

// =========== STATUS POLL ===========
function pollStatus() {
  fetch('/status').then(function(r){ return r.json(); }).then(function(d) {
    // LCD
    document.getElementById('lcdL1').textContent = d.lcd1 || '                ';
    document.getElementById('lcdL2').textContent = d.lcd2 || '                ';
    // Power & SWR
    var w = parseFloat(d.watts) || 0;
    document.getElementById('wattVal').textContent  = w.toFixed(1) + ' W';
    document.getElementById('wattVal2').textContent = w.toFixed(1);
    document.getElementById('wattBar').style.width  = Math.min(w / 70 * 100, 100) + '%';
    document.getElementById('swrVal').textContent   = parseFloat(d.swr).toFixed(1);
    // Relay LEDs
    for (var i = 1; i <= 7; i++) {
      var led = document.getElementById('rl' + i);
      if (led) led.classList.toggle('on', !!d['r' + i]);
    }
    // System info
    document.getElementById('sUptime').textContent  = d.uptime || '-';
    document.getElementById('sHeap').textContent    = d.heap + ' B';
    document.getElementById('sApIp').textContent    = d.apIp  || '192.168.4.1';
    document.getElementById('sStaIp').textContent   = d.staIp || 'non connesso';
    document.getElementById('sWifiSta').textContent = d.wifiSta || '-';
    var bandNames = ['Nessuna','15/10m','20/17m','40m','80m'];
    document.getElementById('sBand').textContent    = bandNames[parseInt(d.band)] || '-';
    document.getElementById('sAnt').textContent     = 'ANT ' + d.ant;
    document.getElementById('fwVer').textContent    = d.fwVer  || '-';
    document.getElementById('fwDate').textContent   = d.fwDate || '-';
    // Sync dashboard state
    if (cBand !== parseInt(d.band)) { cBand = parseInt(d.band); updateBandUI(); }
    if (cAnt  !== parseInt(d.ant))  { cAnt  = parseInt(d.ant);  updateAntUI(); }
    // Fan state
    var s1 = parseInt(d.fan1) || 0;
    var s2 = parseInt(d.fan2) || 0;
    var p1 = Math.round(s1 / 255 * 100);
    var p2 = Math.round(s2 / 255 * 100);
    var sl1 = document.getElementById('fan1Slider');
    var sl2 = document.getElementById('fan2Slider');
    if (sl1) { sl1.value = p1; document.getElementById('fan1Val').textContent = p1 + '%'; }
    if (sl2) { sl2.value = p2; document.getElementById('fan2Val').textContent = p2 + '%'; }
    updateFanUI(s1, s2);
    // Sync settings selects
    document.getElementById('prefBand').value = d.band;
    document.getElementById('prefAnt').value  = d.ant;
  }).catch(function(){});
}

setInterval(pollStatus, 500);
pollStatus();
updateAntUI();
</script>
</body>
</html>
)rawhtml";

// ============================================================
// SETUP
// ============================================================
void setup() {
  g_startMs = millis();

  // Seriale
  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println(F("=== Minipa 70W Controller ==="));
  Serial.println(F("Firmware v" FIRMWARE_VERSION " - " BUILD_DATE));

  // Carica preferenze NVS
  loadPreferences();

  // Inizializza pin relè
  setupPins();

  // Inizializza ventole L298N
  setupFans();

  // WiFi
  setupWiFi();

  // I2C slave (intercetta LCD ATU-100)
  setupI2CSlave();

  // Web server
  setupWebServer();

  Serial.println(F("Setup completato."));
  Serial.printf("AP: %s  IP: 192.168.4.1\n", g_apSSID.c_str());
  if (g_staSSID.length() > 0) {
    Serial.printf("Connessione a router: %s\n", g_staSSID.c_str());
  }
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  g_server.handleClient();
  handleSerial();
  handleTunePulse();
  handleLED();
  handleI2CData();
  reconnectSTA();
}

// ============================================================
// SETUP FUNCTIONS
// ============================================================

void setupPins() {
  // Relè — tutti OUTPUT, inizializzati LOW (off)
  int relayPins[] = {
    PIN_RELAY1_LPF1510,
    PIN_RELAY2_LPF2017,
    PIN_RELAY3_LPF40,
    PIN_RELAY4_LPF80,
    PIN_RELAY5_TUNE,
    PIN_RELAY6_PTT,
    PIN_RELAY7_ANT
  };
  for (int i = 0; i < 7; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }

  // LED
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // Applica stato iniziale da preferenze
  setLPF(g_band);
  setAnt(g_ant);
}

void setupFans() {
  ledcAttach(PIN_FAN1_PWM, FAN_PWM_FREQ, FAN_PWM_RES);
  ledcAttach(PIN_FAN2_PWM, FAN_PWM_FREQ, FAN_PWM_RES);
  pinMode(PIN_FAN1_DIR, OUTPUT);
  digitalWrite(PIN_FAN1_DIR, HIGH);
  pinMode(PIN_FAN2_DIR, OUTPUT);
  digitalWrite(PIN_FAN2_DIR, HIGH);
  ledcWrite(PIN_FAN1_PWM, 0);
  ledcWrite(PIN_FAN2_PWM, 0);
  Serial.println(F("Ventole L298N inizializzate (OFF)"));
}

void setFan1(uint8_t speed) {
  g_fan1Speed = speed;
  ledcWrite(PIN_FAN1_PWM, speed);
}

void setFan2(uint8_t speed) {
  g_fan2Speed = speed;
  ledcWrite(PIN_FAN2_PWM, speed);
}

void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);

  // Avvia Access Point
  IPAddress apIP(192, 168, 4, 1);
  IPAddress apSubnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apIP, apSubnet);
  WiFi.softAP(g_apSSID.c_str(), g_apPass.c_str(), g_apChannel);
  Serial.printf("AP avviato: SSID=%s\n", g_apSSID.c_str());

  // Connessione al router (se configurato)
  if (g_staSSID.length() > 0) {
    WiFi.begin(g_staSSID.c_str(), g_staPass.c_str());
    Serial.println(F("Connessione al router in corso..."));
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
      delay(200);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Router connesso! IP STA: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println(F("Connessione router fallita (riprovo ogni 30s)"));
    }
  }
}

void setupWebServer() {
  g_server.on("/",          HTTP_GET, handleRoot);
  g_server.on("/status",    HTTP_GET, handleStatus);
  g_server.on("/band",      HTTP_GET, handleBand);
  g_server.on("/ptt",       HTTP_GET, handlePTT);
  g_server.on("/ant",       HTTP_GET, handleAnt);
  g_server.on("/tune",      HTTP_GET, handleTune);
  g_server.on("/fan",       HTTP_GET, handleFan);
  g_server.on("/wifi/connect",    HTTP_GET, handleWiFiConnect);
  g_server.on("/wifi/disconnect", HTTP_GET, handleWiFiDisconnect);
  g_server.on("/wifi/scan",       HTTP_GET, handleWiFiScan);
  g_server.on("/ap/config",       HTTP_GET, handleAPConfig);
  g_server.on("/prefs/save",      HTTP_GET, handleSavePrefs);
  g_server.on("/reboot",          HTTP_GET, handleReboot);
  g_server.onNotFound(handleNotFound);
  g_server.begin();
  Serial.println(F("Web server avviato su porta 80"));
}

void setupI2CSlave() {
  // L'ESP32 ascolta come slave I2C all'indirizzo 0x27 (stesso del PCF8574)
  // L'ATU-100 invia i dati LCD al PCF8574; l'ESP32 intercetta quei pacchetti
  // Slave mode: Wire.begin(slaveAddr, sda, scl)
  Wire.begin((uint8_t)LCD_SLAVE_ADDR, I2C_SDA, I2C_SCL);
  Wire.onReceive(onI2CReceive);
  Serial.printf("I2C slave avviato su addr=0x%02X (SDA=%d SCL=%d)\n",
                LCD_SLAVE_ADDR, I2C_SDA, I2C_SCL);
}

// ============================================================
// NVS PREFERENCES
// ============================================================

void loadPreferences() {
  g_prefs.begin(NVS_NS, false);
  g_apSSID    = g_prefs.getString("apSSID",  DEFAULT_AP_SSID);
  g_apPass    = g_prefs.getString("apPass",  DEFAULT_AP_PASS);
  g_apChannel = g_prefs.getInt   ("apCh",    DEFAULT_AP_CHANNEL);
  g_staSSID   = g_prefs.getString("staSSID", "");
  g_staPass   = g_prefs.getString("staPass", "");
  g_band      = g_prefs.getInt   ("band",    0);
  g_ant       = g_prefs.getInt   ("ant",     1);
  g_prefs.end();
  Serial.printf("Preferenze caricate: band=%d ant=%d\n", g_band, g_ant);
}

void savePreferences() {
  g_prefs.begin(NVS_NS, false);
  g_prefs.putString("apSSID",  g_apSSID);
  g_prefs.putString("apPass",  g_apPass);
  g_prefs.putInt   ("apCh",    g_apChannel);
  g_prefs.putString("staSSID", g_staSSID);
  g_prefs.putString("staPass", g_staPass);
  g_prefs.putInt   ("band",    g_band);
  g_prefs.putInt   ("ant",     g_ant);
  g_prefs.end();
}

// ============================================================
// RELAY CONTROL
// ============================================================

// Mappa indice relè (1..7) → pin fisico
void applyRelayPin(int relay, bool state) {
  // Array 1-based: pins[0] è uno placeholder (0) non usato mai,
  // pins[1..7] corrispondono ai pin fisici dei sette relè
  const int pins[] = {0,
    PIN_RELAY1_LPF1510,
    PIN_RELAY2_LPF2017,
    PIN_RELAY3_LPF40,
    PIN_RELAY4_LPF80,
    PIN_RELAY5_TUNE,
    PIN_RELAY6_PTT,
    PIN_RELAY7_ANT
  };
  if (relay >= 1 && relay <= 7) {
    g_relay[relay] = state;
    digitalWrite(pins[relay], state ? HIGH : LOW);
  }
}

// Attiva LPF della banda selezionata (mutuamente esclusivi)
void setLPF(int band) {
  g_band = band;
  applyRelayPin(1, band == 1);
  applyRelayPin(2, band == 2);
  applyRelayPin(3, band == 3);
  applyRelayPin(4, band == 4);
}

void setPTT(bool on) {
  applyRelayPin(6, on);
}

void setAnt(int ant) {
  g_ant = ant;
  // ANT1 = HIGH, ANT2 = LOW
  applyRelayPin(7, ant == 1);
}

void triggerTune() {
  if (!g_tuneActive) {
    applyRelayPin(5, true);
    g_tuneActive  = true;
    g_tuneStartMs = millis();
    Serial.println(F("TUNE: impulso accordo ON"));
  }
}

// ============================================================
// LOOP HANDLERS
// ============================================================

// Gestisce il timeout dell'impulso TUNE (100ms, non bloccante)
void handleTunePulse() {
  if (g_tuneActive && (millis() - g_tuneStartMs >= TUNE_PULSE_MS)) {
    applyRelayPin(5, false);
    g_tuneActive = false;
    Serial.println(F("TUNE: impulso accordo OFF (auto-reset)"));
  }
}

// LED lampeggio: lento in RX, veloce se PTT attivo
void handleLED() {
  unsigned long interval = g_relay[6] ? LED_FAST_MS : LED_SLOW_MS;
  if (millis() - g_lastLedMs >= interval) {
    g_lastLedMs = millis();
    g_ledState  = !g_ledState;
    digitalWrite(PIN_LED_STATUS, g_ledState ? HIGH : LOW);
  }
}

// Gestisce i comandi seriali
void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      g_serialBuf.trim();
      if (g_serialBuf.length() > 0) {
        processSerialCommand(g_serialBuf);
      }
      g_serialBuf = "";
    } else {
      if (g_serialBuf.length() < 64) {
        g_serialBuf += c;
      }
    }
  }
}

// Tenta riconnessione STA se configurata e non connessa
void reconnectSTA() {
  if (g_staSSID.length() == 0) return;
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - g_lastStaAttemptMs < STA_RECONNECT_MS) return;
  g_lastStaAttemptMs = millis();
  Serial.printf("Tentativo riconnessione a %s...\n", g_staSSID.c_str());
  WiFi.disconnect(false);
  WiFi.begin(g_staSSID.c_str(), g_staPass.c_str());
}

// ============================================================
// SERIALE — ELABORAZIONE COMANDI
// ============================================================

void processSerialCommand(const String& cmd) {
  Serial.printf("CMD seriale: %s\n", cmd.c_str());

  if (cmd.startsWith("BAND:")) {
    String arg = cmd.substring(5);
    if      (arg == "80") setLPF(4);
    else if (arg == "40") setLPF(3);
    else if (arg == "20") setLPF(2);
    else if (arg == "15") setLPF(1);
    else Serial.println(F("ERR: banda non valida (80/40/20/15)"));
    Serial.printf("Banda impostata: %s\n", arg.c_str());

  } else if (cmd.startsWith("PTT:")) {
    int v = cmd.substring(4).toInt();
    setPTT(v != 0);
    Serial.printf("PTT %s\n", v ? "ON" : "OFF");

  } else if (cmd == "TUNE") {
    triggerTune();

  } else if (cmd.startsWith("ANT:")) {
    int v = cmd.substring(4).toInt();
    if (v == 1 || v == 2) {
      setAnt(v);
      Serial.printf("Antenna: ANT%d\n", v);
    } else {
      Serial.println(F("ERR: antenna non valida (1 o 2)"));
    }

  } else if (cmd.startsWith("FAN1:")) {
    int v = cmd.substring(5).toInt();
    uint8_t speed = (uint8_t)constrain(v, 0, 255);
    setFan1(speed);
    Serial.printf("Ventola 1 velocità: %d\n", speed);

  } else if (cmd.startsWith("FAN2:")) {
    int v = cmd.substring(5).toInt();
    uint8_t speed = (uint8_t)constrain(v, 0, 255);
    setFan2(speed);
    Serial.printf("Ventola 2 velocità: %d\n", speed);

  } else if (cmd.startsWith("FAN:")) {
    int v = cmd.substring(4).toInt();
    uint8_t speed = (uint8_t)constrain(v, 0, 255);
    setFan1(speed);
    setFan2(speed);
    Serial.printf("Ventole 1+2 velocità: %d\n", speed);

  } else if (cmd == "STATUS?") {
    Serial.println(F("--- STATUS ---"));
    Serial.printf("Firmware: v%s  Build: %s\n", FIRMWARE_VERSION, BUILD_DATE);
    Serial.printf("Uptime  : %s\n", uptimeStr().c_str());
    Serial.printf("Free heap: %u B\n", ESP.getFreeHeap());
    Serial.printf("AP SSID : %s  IP: 192.168.4.1\n", g_apSSID.c_str());
    Serial.printf("STA SSID: %s  IP: %s  Status: %d\n",
                  g_staSSID.c_str(),
                  WiFi.localIP().toString().c_str(),
                  (int)WiFi.status());
    Serial.printf("Banda   : %d (%s)\n", g_band,
                  g_band == 0 ? "nessuna" :
                  g_band == 1 ? "15/10m"  :
                  g_band == 2 ? "20/17m"  :
                  g_band == 3 ? "40m"     : "80m");
    Serial.printf("Antenna : ANT%d\n", g_ant);
    for (int i = 1; i <= 7; i++) {
      Serial.printf("Relè %d   : %s\n", i, g_relay[i] ? "ON" : "OFF");
    }
    Serial.printf("LCD L1  : [%s]\n", g_lcd1);
    Serial.printf("LCD L2  : [%s]\n", g_lcd2);
    Serial.printf("Watt    : %.1f  SWR: %.2f\n", g_watts, g_swr);
    Serial.printf("Ventola1: %d/255  Ventola2: %d/255\n", g_fan1Speed, g_fan2Speed);

  } else {
    Serial.printf("ERR: comando sconosciuto: %s\n", cmd.c_str());
    Serial.println(F("Comandi: BAND:80/40/20/15, PTT:1/0, TUNE, ANT:1/2, FAN1:<0-255>, FAN2:<0-255>, FAN:<0-255>, STATUS?"));
  }
}

// ============================================================
// I2C SLAVE — INTERCETTAZIONE LCD ATU-100
// ============================================================

// Callback chiamata in interrupt context quando l'ATU-100 manda dati I2C
void IRAM_ATTR onI2CReceive(int numBytes) {
  int len = 0;
  while (Wire.available() && len < LCD_BUF_SIZE) {
    g_i2cRaw[len++] = Wire.read();
  }
  g_i2cRawLen  = len;
  g_i2cNewData = true;
}

// Chiamata nel loop per elaborare il buffer I2C ricevuto
void handleI2CData() {
  if (!g_i2cNewData) return;
  g_i2cNewData = false;

  // Copia buffer in zona non-volatile
  uint8_t buf[LCD_BUF_SIZE];
  int     len;
  noInterrupts();
  len = g_i2cRawLen;
  for (int i = 0; i < len; i++) buf[i] = g_i2cRaw[i];
  interrupts();

  // Il PCF8574 invia ogni nibble come un byte: bit2=E, bit0=RS
  // Formato: nibble_high (E=1), nibble_high (E=0), nibble_low (E=1), nibble_low (E=0)
  for (int i = 0; i < len; i++) {
    uint8_t raw = buf[i];
    uint8_t E   = (raw >> 2) & 1;
    uint8_t RS  = (raw >> 0) & 1;
    uint8_t nib = (raw >> 4) & 0x0F;  // nibble dati sui bit 7-4

    // Consideriamo solo i byte con E=1 (fronte di discesa → dato valido)
    if (E) {
      if (!s_lcdNibbleHigh) {
        // Primo nibble (quello alto)
        s_lcdNibble      = nib << 4;
        s_lcdNibbleHigh  = true;
      } else {
        // Secondo nibble (quello basso) → assembla il byte
        uint8_t data = s_lcdNibble | nib;
        s_lcdNibbleHigh = false;
        lcdProcessByte(RS, data);
      }
    }
  }

  // Aggiorna valori estratti
  parseLCDContent();
}

// Processa un byte completamente decodificato dall'HD44780
void lcdProcessByte(uint8_t rs, uint8_t data) {
  if (rs == 0) {
    // Istruzione LCD
    if (data == 0x01) {
      // Clear display → azzera buffer
      memset(g_lcd1, ' ', LCD_COLS); g_lcd1[LCD_COLS] = '\0';
      memset(g_lcd2, ' ', LCD_COLS); g_lcd2[LCD_COLS] = '\0';
      s_lcdCursorRow = false;
      s_lcdCursorCol = 0;
    } else if ((data & 0xC0) == 0x80) {
      // Set DDRAM address
      uint8_t addr = data & 0x7F;
      if (addr < 0x40) {
        s_lcdCursorRow = false;
        s_lcdCursorCol = addr;
      } else {
        s_lcdCursorRow = true;
        s_lcdCursorCol = addr - 0x40;
      }
    }
    // Altre istruzioni ignorate
  } else {
    // Dato carattere
    if (s_lcdCursorCol < LCD_COLS) {
      char ch = (data >= 0x20 && data < 0x80) ? (char)data : ' ';
      if (!s_lcdCursorRow) {
        g_lcd1[s_lcdCursorCol] = ch;
      } else {
        g_lcd2[s_lcdCursorCol] = ch;
      }
      s_lcdCursorCol++;
    }
  }
}

// Parsing euristico del contenuto LCD per estrarre Watt e SWR
void parseLCDContent() {
  // Ricerca pattern "XXW" o "XX.XW" nella riga 1 o 2
  // e pattern "SWR:X.XX" o "ROE:X.XX"
  String l1 = String(g_lcd1);
  String l2 = String(g_lcd2);

  // Cerca Watt: numero seguito da 'W'
  for (int ri = 0; ri < 2; ri++) {
    String& row = (ri == 0) ? l1 : l2;
    int wIdx = row.indexOf('W');
    while (wIdx > 0) {
      // Troviamo l'inizio del numero precedente la 'W'
      int start = wIdx - 1;
      // Scorriamo all'indietro finché ci sono cifre o punto decimale
      // La condizione `start > 0` garantisce che row[start-1] sia sempre valido (indice >= 0)
      while (start > 0 && (isdigit(row[start - 1]) || row[start - 1] == '.')) {
        start--;
      }
      String numStr = row.substring(start, wIdx);
      if (numStr.length() > 0) {
        float v = numStr.toFloat();
        if (v >= 0 && v <= 200) {
          g_watts = v;
          break;
        }
      }
      wIdx = row.indexOf('W', wIdx + 1);
    }
  }

  // Cerca SWR
  String all = l1 + l2;
  int swrIdx = all.indexOf("SWR");
  if (swrIdx < 0) swrIdx = all.indexOf("ROE");
  if (swrIdx < 0) swrIdx = all.indexOf("swr");
  if (swrIdx >= 0) {
    // Cerca ':' dopo l'etichetta SWR/ROE; indexOf restituisce -1 se non trovato
    int colonIdx = all.indexOf(':', swrIdx);
    if (colonIdx >= swrIdx && colonIdx < swrIdx + 6) {
      String sub = all.substring(colonIdx + 1, colonIdx + 7);
      sub.trim();
      float v = sub.toFloat();
      if (v >= 1.0f && v <= 99.0f) {
        g_swr = v;
      }
    }
  }
}

// ============================================================
// WEB SERVER HANDLERS
// ============================================================

// Invia la pagina principale
void handleRoot() {
  g_server.sendHeader("Cache-Control", "no-cache");
  g_server.send_P(200, "text/html", INDEX_HTML);
}

// /status — JSON con tutto lo stato
void handleStatus() {
  String ip_sta = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "";
  String wifiSta = (WiFi.status() == WL_CONNECTED) ? g_staSSID : "non connesso";

  String json = "{";
  json += "\"lcd1\":\""   + String(g_lcd1)           + "\",";
  json += "\"lcd2\":\""   + String(g_lcd2)           + "\",";
  json += "\"watts\":"    + String(g_watts, 1)        + ",";
  json += "\"swr\":"      + String(g_swr, 2)          + ",";
  json += "\"band\":"     + String(g_band)            + ",";
  json += "\"ant\":"      + String(g_ant)             + ",";
  for (int i = 1; i <= 7; i++) {
    json += "\"r" + String(i) + "\":" + (g_relay[i] ? "true" : "false") + ",";
  }
  json += "\"uptime\":\""  + uptimeStr()              + "\",";
  json += "\"heap\":"      + String(ESP.getFreeHeap()) + ",";
  json += "\"apIp\":\"192.168.4.1\",";
  json += "\"staIp\":\""   + ip_sta                   + "\",";
  json += "\"wifiSta\":\""  + wifiSta                 + "\",";
  json += "\"fwVer\":\""   + String(FIRMWARE_VERSION) + "\",";
  json += "\"fwDate\":\""  + String(BUILD_DATE) + " " + String(BUILD_TIME) + "\",";
  json += "\"fan1\":"      + String(g_fan1Speed) + ",";
  json += "\"fan2\":"      + String(g_fan2Speed);
  json += "}";

  g_server.sendHeader("Access-Control-Allow-Origin", "*");
  g_server.send(200, "application/json", json);
}

// /band?b=<1..4>
void handleBand() {
  if (g_server.hasArg("b")) {
    int b = g_server.arg("b").toInt();
    if (b >= 0 && b <= 4) {
      setLPF(b);
    }
  }
  g_server.send(200, "application/json", "{\"band\":" + String(g_band) + "}");
}

// /ptt?v=<0|1>
void handlePTT() {
  if (g_server.hasArg("v")) {
    int v = g_server.arg("v").toInt();
    setPTT(v != 0);
  }
  g_server.send(200, "application/json", "{\"ptt\":" + String(g_relay[6] ? "true" : "false") + "}");
}

// /ant?v=<1|2>
void handleAnt() {
  if (g_server.hasArg("v")) {
    int v = g_server.arg("v").toInt();
    if (v == 1 || v == 2) setAnt(v);
  }
  g_server.send(200, "application/json", "{\"ant\":" + String(g_ant) + "}");
}

// /tune — avvia impulso 100ms (non bloccante)
void handleTune() {
  triggerTune();
  g_server.send(200, "application/json", "{\"tune\":true}");
}

// /fan?ch=<0|1|2>&v=<0-255>
// ch=0 → entrambe, ch=1 → ventola 1, ch=2 → ventola 2
void handleFan() {
  if (!g_server.hasArg("ch") || !g_server.hasArg("v")) {
    g_server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Parametri mancanti\"}");
    return;
  }
  int ch    = g_server.arg("ch").toInt();
  uint8_t speed = (uint8_t)constrain(g_server.arg("v").toInt(), 0, 255);
  if (ch == 0) { setFan1(speed); setFan2(speed); }
  else if (ch == 1) { setFan1(speed); }
  else if (ch == 2) { setFan2(speed); }
  g_server.send(200, "application/json",
    "{\"ok\":true,\"fan1\":" + String(g_fan1Speed) + ",\"fan2\":" + String(g_fan2Speed) + "}");
}

// /wifi/connect?ssid=...&pass=...
void handleWiFiConnect() {
  String ssid = g_server.hasArg("ssid") ? g_server.arg("ssid") : "";
  String pass = g_server.hasArg("pass") ? g_server.arg("pass") : "";
  if (ssid.length() == 0) {
    g_server.send(200, "application/json", "{\"ok\":false,\"msg\":\"SSID mancante\"}");
    return;
  }
  g_staSSID = ssid;
  g_staPass = pass;
  savePreferences();

  WiFi.disconnect(false);
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.printf("Connessione a: %s\n", ssid.c_str());

  // Attesa non bloccante breve (3s) per risposta rapida
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 3000) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    Serial.printf("Connesso! IP: %s\n", ip.c_str());
    g_server.send(200, "application/json",
      "{\"ok\":true,\"msg\":\"Connesso a " + ssid + " — IP: " + ip + "\"}");
  } else {
    g_server.send(200, "application/json",
      "{\"ok\":false,\"msg\":\"Connessione in corso (controlla Monitor tra 30s)\"}");
  }
}

// /wifi/disconnect
void handleWiFiDisconnect() {
  g_staSSID = "";
  g_staPass = "";
  savePreferences();
  WiFi.disconnect(true);
  Serial.println(F("WiFi STA disconnesso"));
  g_server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Disconnesso dal router\"}");
}

// /wifi/scan — restituisce lista reti
void handleWiFiScan() {
  Serial.println(F("Scansione WiFi..."));
  int n = WiFi.scanNetworks();
  String json = "{\"nets\":[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    String ssid = WiFi.SSID(i);
    ssid.replace("\"", "\\\"");
    json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
  }
  json += "]}";
  WiFi.scanDelete();
  g_server.send(200, "application/json", json);
}

// /ap/config?ssid=...&pass=...&ch=...
void handleAPConfig() {
  String ssid = g_server.hasArg("ssid") ? g_server.arg("ssid") : "";
  String pass = g_server.hasArg("pass") ? g_server.arg("pass") : "";
  int    ch   = g_server.hasArg("ch")   ? g_server.arg("ch").toInt() : 1;

  if (ssid.length() == 0 || pass.length() < 8) {
    g_server.send(200, "application/json",
      "{\"ok\":false,\"msg\":\"SSID e password (min 8 char) obbligatori\"}");
    return;
  }
  if (ch < 1 || ch > 13) ch = 1;

  g_apSSID    = ssid;
  g_apPass    = pass;
  g_apChannel = ch;
  savePreferences();

  g_server.send(200, "application/json",
    "{\"ok\":true,\"msg\":\"Configurazione AP salvata — riavvia per applicare\"}");
}

// /prefs/save?band=...&ant=...
void handleSavePrefs() {
  if (g_server.hasArg("band")) {
    int b = g_server.arg("band").toInt();
    if (b >= 0 && b <= 4) g_band = b;
  }
  if (g_server.hasArg("ant")) {
    int a = g_server.arg("ant").toInt();
    if (a == 1 || a == 2) g_ant = a;
  }
  savePreferences();
  g_server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Preferenze salvate\"}");
}

// /reboot
void handleReboot() {
  g_server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Riavvio in corso...\"}");
  delay(200);
  ESP.restart();
}

void handleNotFound() {
  g_server.send(404, "text/plain", "404 Not Found");
}

// ============================================================
// UTILITY
// ============================================================

// Restituisce l'uptime formattato come "Xh Xm Xs"
String uptimeStr() {
  unsigned long ms   = millis() - g_startMs;
  unsigned long sec  = ms / 1000;
  unsigned long mins = sec / 60;
  unsigned long hr   = mins / 60;
  sec  %= 60;
  mins %= 60;
  char buf[24];
  snprintf(buf, sizeof(buf), "%luh %02lum %02lus", hr, mins, sec);
  return String(buf);
}
