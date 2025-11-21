#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <FS.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "mbedtls/md5.h"
#include <EthernetUdp.h>
#include <Dns.h>
#include <sys/time.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SSLClient.h>
#include <trust_anchors.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#if defined(ESP32)
#include "esp_timer.h"
#endif

#include "index_html.h"   // UI page
#include "alarm_types.h"

#define ADS_PREF_NS "ads"   // make sure you use this same namespace everywhere

// Forward declarations so auto-generated prototypes can reference the type.
struct MeasFrame;
enum LedMode : uint8_t;
struct LedState;

static const size_t NUM_SENSORS = 4;

static SemaphoreHandle_t g_adsMutex = nullptr;

static volatile float g_lastMv[NUM_SENSORS]  = {0,0,0,0};
static volatile float g_lastmA[NUM_SENSORS]  = {0,0,0,0};
static volatile float g_lastPct[NUM_SENSORS] = {0,0,0,0};

// Debounce for alarms (min dwell before state change)
static uint32_t g_alarmLastChangeMs[NUM_SENSORS] = {0,0,0,0};
static const uint32_t ALARM_MIN_DWELL_MS = 200; // ms

static void seedRNG_noADC() {
  uint32_t s = esp_random() ^ (uint32_t)micros() ^ (uint32_t)ESP.getEfuseMac();
  randomSeed(s);
}

#if defined(ESP32)
static inline uint64_t monotonicMicros(){
  return static_cast<uint64_t>(esp_timer_get_time());
}
#else
static inline uint64_t monotonicMicros(){
  return static_cast<uint64_t>(micros());
}
#endif

static inline bool validSps(int sps){
  switch (sps) {
    case 8: case 16: case 32: case 64: case 128: case 250: case 475: case 860:
      return true;
    default:
      return false;
  }
}

// --- Data-rate helper: works across different Adafruit ADS libs ---
// Call ads.setDataRate(...) using whatever this library version provides.
// If the lib has no data-rate API, this becomes a no-op.
static void adsSetRateSps(Adafruit_ADS1115& ads, int sps) {
  switch (sps) { case 8: case 16: case 32: case 64: case 128: case 250: case 475: case 860: break; default: sps = 250; }
  #if defined(RATE_ADS1115_8SPS)
    switch (sps) {
      case 8:   ads.setDataRate(RATE_ADS1115_8SPS);   break;
      case 16:  ads.setDataRate(RATE_ADS1115_16SPS);  break;
      case 32:  ads.setDataRate(RATE_ADS1115_32SPS);  break;
      case 64:  ads.setDataRate(RATE_ADS1115_64SPS);  break;
      case 128: ads.setDataRate(RATE_ADS1115_128SPS); break;
      case 250: ads.setDataRate(RATE_ADS1115_250SPS); break;
      case 475: ads.setDataRate(RATE_ADS1115_475SPS); break;
      case 860: ads.setDataRate(RATE_ADS1115_860SPS); break;
    }
  #elif defined(RATE_ADS1X15_8SPS)
    switch (sps) {
      case 8:   ads.setDataRate(RATE_ADS1X15_8SPS);   break;
      case 16:  ads.setDataRate(RATE_ADS1X15_16SPS);  break;
      case 32:  ads.setDataRate(RATE_ADS1X15_32SPS);  break;
      case 64:  ads.setDataRate(RATE_ADS1X15_64SPS);  break;
      case 128: ads.setDataRate(RATE_ADS1X15_128SPS); break;
      case 250: ads.setDataRate(RATE_ADS1X15_250SPS); break;
      case 475: ads.setDataRate(RATE_ADS1X15_475SPS); break;
      case 860: ads.setDataRate(RATE_ADS1X15_860SPS); break;
    }
  #else
    (void)ads; (void)sps; // very old lib -> no data-rate API
  #endif
}

// ---- TLS (SSLClient) ----
static EthernetClient _tcp;
// Debug level: NONE, ERROR, WARN, INFO  (prints to Serial)
static const SSLClient::DebugLevel TLS_DBG = SSLClient::SSL_ERROR;
// ctor: (base client, trust anchors, count, analog pin (unused on ESP32),
//        handshake timeout (sec), debug level)
SSLClient _tls(_tcp, TAs, TAs_NUM, -1, 8, TLS_DBG);

static inline void tlsPrepare(const String& /*host*/) {
  // No setInsecure() here — this SSLClient doesn’t implement it.
  // SNI is handled internally from the hostname passed to connect().
}



Adafruit_ADS1115 ads;
Preferences adsPrefs;            // NVS namespace handle for ADS config
static bool adsPrefsReady = false;
static bool adsReady = false;

// Selection (single, both legacy, or all four)
enum AdsSel : uint8_t { ADS_SEL_A0=0, ADS_SEL_A1=1, ADS_SEL_BOTH=2, ADS_SEL_ALL=3 };
static AdsSel  g_adsSel = ADS_SEL_ALL;

// --- Per-channel configuration (NEW) ---
static adsGain_t g_gainCh[NUM_SENSORS]  = { GAIN_ONE, GAIN_ONE, GAIN_ONE, GAIN_ONE }; // ±4.096 V
static int       g_rateCh[NUM_SENSORS]  = { 250, 250, 250, 250 };                     // SPS
static float     g_shuntCh[NUM_SENSORS] = { 160.0f, 160.0f, 160.0f, 160.0f };         // Ω

// --- Legacy single-channel shadows (mirror A0 so older code compiles) ---
static adsGain_t g_adsGain    = GAIN_ONE;
static int       g_adsRateSps = 250;
static float     g_shuntOhms  = 160.0f;

// Per-channel engineering units config for mm
static float g_engFSmm[NUM_SENSORS]  = {40.0f, 40.0f, 40.0f, 40.0f};  // full-scale in mm — selectable 40/80
static float g_engOffmm[NUM_SENSORS] = {0.0f,  0.0f,  0.0f,  0.0f };  // offset added after scaling (mm)

// I2C pins
static const int I2C_SDA = 21, I2C_SCL = 22;

// Single definition only (remove any duplicates elsewhere!)
static inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

// LSB (mV per code) for ADS1115 by gain
static float adsLSB_mV(adsGain_t g){
  switch(g){
    case GAIN_TWOTHIRDS: return 0.1875f;
    case GAIN_ONE:       return 0.1250f;
    case GAIN_TWO:       return 0.0625f;
    case GAIN_FOUR:      return 0.03125f;
    case GAIN_EIGHT:     return 0.015625f;
    case GAIN_SIXTEEN:   return 0.0078125f;
  }
  return 0.1250f;
}

// Map 4–20 mA % to mm for channel ch
static float mapToMM(uint8_t ch, float pct){
  if (ch >= NUM_SENSORS) ch = NUM_SENSORS - 1;
  float p = pct; if (p < 0) p = 0; if (p > 100) p = 100;
  return (p/100.0f) * g_engFSmm[ch] + g_engOffmm[ch];
}

// Map adsGain_t <-> byte
static uint8_t gainToCode(adsGain_t g){
  switch(g){
    case GAIN_TWOTHIRDS: return 0;
    case GAIN_ONE:       return 1;
    case GAIN_TWO:       return 2;
    case GAIN_FOUR:      return 3;
    case GAIN_EIGHT:     return 4;
    case GAIN_SIXTEEN:   return 5;
  }
  return 1; // default GAIN_ONE
}
static adsGain_t codeToGain(uint8_t c){
  switch(c){
    case 0: return GAIN_TWOTHIRDS;
    case 1: return GAIN_ONE;
    case 2: return GAIN_TWO;
    case 3: return GAIN_FOUR;
    case 4: return GAIN_EIGHT;
    case 5: return GAIN_SIXTEEN;
  }
  return GAIN_ONE;
}

// ---- ADS reliability telemetry ----
static uint32_t g_adsFailCount   = 0;
static String   g_adsLastErr     = "none";
static uint32_t g_adsLastErrMs   = 0;
static uint32_t g_adsLastReinitMs= 0;

// ---- CSV export scratch buffers (off-stack) ----
struct __attribute__((packed)) CsvFrame { uint32_t t_10us; int16_t raw[NUM_SENSORS]; };
static constexpr size_t CSV_EXPORT_FRAME_CHUNK = 512;
static char      g_csvRowBuf[256];
static char      g_csvBuf[4096];
static uint8_t   g_csvFrameBuf[CSV_EXPORT_FRAME_CHUNK * sizeof(CsvFrame)];

// Apply current RAM settings to the ADS chip (no defaults here!)
static void adsApplyHW(){
  if (!adsReady) return;
  // Program chip with A0 defaults; per-read reconfig handles the actual channel
  ads.setGain(g_adsGain);
  adsSetRateSps(ads, g_adsRateSps);
}

// ---- Alarm thresholds (mA) with hysteresis ----
static const float UC_SET = 3.0f;   // undercurrent set
static const float UC_CLR = 3.5f;   // undercurrent clear
static const float OC_SET = 22.0f;  // overcurrent set
static const float OC_CLR = 21.5f;  // overcurrent clear

// Status LEDs
static const int   LED_RUN_PIN   = 25; // Blue  (shared with ADS1)
static const int   LED_NET_PIN   = 26; // White (shared with INT)
static const int   LED_ERROR_PIN = 14; // Red
static const int   LED_MEAS_PIN  = 13; // Yellow

static AlarmState g_alarmCh[NUM_SENSORS] = { ALARM_NORMAL, ALARM_NORMAL, ALARM_NORMAL, ALARM_NORMAL };
static bool       g_alarmActive = false;

enum LedMode : uint8_t {
  LED_OFF = 0,
  LED_ON,
  LED_BLINK_SLOW,
  LED_BLINK_FAST,
  LED_HEARTBEAT,
  LED_DOUBLE_BLINK
};

struct LedState {
  int     pin;
  LedMode mode;
  bool    level;
};

static LedState g_ledRun  { LED_RUN_PIN,   LED_OFF, false };
static LedState g_ledNet  { LED_NET_PIN,   LED_OFF, false };
static LedState g_ledError{ LED_ERROR_PIN, LED_OFF, false };
static LedState g_ledMeas { LED_MEAS_PIN,  LED_OFF, false };

// --------- PINS ----------
static const int WIZ_CS   = 5;    // WIZ850io CS
static const int WIZ_RST  = 27;   // WIZ850io RST (ACTIVE LOW)
static const int SD_CS    = 4;    // SD card CS
// VSPI default pins:
static const int VSPI_SCK  = 18;
static const int VSPI_MISO = 19;
static const int VSPI_MOSI = 23;

// --------- GLOBALS ----------
WebServer server(80);
DNSServer dns;
Preferences prefs;
EthernetUDP ntpUDP;
static bool g_timeSynced = false;

static bool     g_internetOk       = false;
static uint32_t g_lastInetCheckMs  = 0;
static const uint32_t INTERNET_CHECK_INTERVAL_MS = 5000;
static const uint16_t INTERNET_CHECK_TIMEOUT_MS  = 750;

String apSsid;
const char* apPass = "changeme123";

byte ethMac[6] = { 0x02,0x11,0x22,0x33,0x55,0x77 };
IPAddress testHost(1,1,1,1);
const uint16_t testPort = 53;

struct AppCfg {
  String devName   = "sensor-prototype";
  String serverUrl = "http://example.com/ingest";
  String apiKey    = "";
  bool useStatic   = false;
  IPAddress ip     = IPAddress(0,0,0,0);
  IPAddress gw     = IPAddress(0,0,0,0);
  IPAddress mask   = IPAddress(255,255,255,0);
  IPAddress dns    = IPAddress(1,1,1,1);
  bool cloudEnabled = false;
  uint32_t cloudPeriodS = 30;
  String tlsFp = "";
} cfg;

bool sdMounted = false;
bool ethUpOnce = false;
EthernetLinkStatus lastLink = Unknown;

// logging
static const size_t MAX_LOG_SIZE = 512 * 1024; // 512 KB per file
static const int    MAX_LOG_FILES = 5;         // log0..log4
String currentLogPath = "/logs/log0.log";

// OTA upload MD5 context
mbedtls_md5_context md5ctx;
static bool          md5ctxActive = false;

static bool normalizeMd5Hex(String& md5) {
  md5.trim();
  if (md5.length() != 32) return false;
  md5.toLowerCase();
  for (size_t i = 0; i < md5.length(); ++i) {
    char c = md5.charAt(i);
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
      return false;
    }
  }
  return true;
}
bool otaInProgress = false;

static bool g_mdnsRunning = false;

// ----- Cloud push state -----
static uint32_t g_lastPushMs   = 0;
static int      g_lastHttpCode = -1;
static String   g_lastCloudErr = "";
static String   g_lastPushIso  = "";
// For periodic "next in X s" status
static uint32_t g_nextPushInS  = 0;

// Measurement session
static bool     g_measActive   = false;
static String   g_measId       = "";         // e.g. "2025-09-19_14-05-33"
static String   g_measDir      = "";         // e.g. "/meas/sess_YYYY..."
static String   g_measFile     = "";         // current file path within session dir
static uint32_t g_measFileIndex= 0xFFFFFFFFu; // current file index (span segment)
static uint64_t g_measFileIdxOffset = 0;      // offset applied after wrap detection
static bool     g_measHaveRawIdx = false;     // have we seen a raw index yet?
static uint32_t g_measLastRawIdx = 0;         // last raw index observed
static const uint32_t MEAS_FILE_SPAN_TICKS = 1800UL * 100000UL; // 30 min in 10 µs ticks

// Derived from ADS data-rate (read-only while measuring)
static int      g_measSps[NUM_SENSORS]   = {250, 250, 250, 250};
static float    g_measDtMs[NUM_SENSORS]  = {4.0f, 4.0f, 4.0f, 4.0f};
static uint32_t g_measLastMs[NUM_SENSORS]= {0, 0, 0, 0};

// Counters for status
static uint32_t g_measSamples  = 0;          // lines written (sum of both ch)

// ---- Meas task control ----
static TaskHandle_t g_measTask = nullptr;

// live Hz estimate for /measure/status
static uint32_t g_hzLastMs    = 0;
static uint32_t g_hzLastCount = 0;
static volatile float    g_pairHz      = 0.0f;  // measured frames/sec

// ---------- Binary logger format ----------
static constexpr uint16_t MEAS_HEADER_VER = 2;

struct __attribute__((packed)) MeasHeader {
  char     magic[4];          // "AM01"
  uint16_t ver;               // MEAS_HEADER_VER
  uint16_t reserved;
  uint32_t start_epoch;       // UNIX seconds
  uint32_t time_scale_us;     // 10 for t_10us
  uint16_t sps[NUM_SENSORS];
  uint8_t  gain_code[NUM_SENSORS]; // use gainToCode(...)
  float    sh[NUM_SENSORS];          // Ohms
  float    fs[NUM_SENSORS];          // mm full-scale
  float    off[NUM_SENSORS];         // mm offset
};

struct __attribute__((packed)) MeasFrame {
  uint32_t t_10us; // time since session start in 10 µs units
  int16_t  raw[NUM_SENSORS];
};

// ---------- Logger state (non-live, batched) ----------
static const size_t BATCH_FRAMES = 1024;         // ~12 KB per flush with 4 channels
static MeasFrame   g_batch[BATCH_FRAMES];
static volatile size_t   g_batchFill    = 0;

static uint64_t g_startUs      = 0;             // monotonic microseconds at session start
static uint64_t g_nextDueUs    = 0;             // next pair due time
static uint32_t g_pairPeriodUs = 0;             // ~4200 us @ 475+475

// samples/bytes for status
static volatile uint32_t g_frameCount   = 0;             // number of MeasFrame written
static uint64_t g_measBytes    = 0;             // total bytes on disk

// ---------- Raw ADS1115 register access (fast) ----------
static inline void adsWriteReg(uint8_t reg, uint16_t val){
  Wire.beginTransmission(0x48); Wire.write(reg);
  Wire.write(uint8_t(val>>8)); Wire.write(uint8_t(val)); Wire.endTransmission();
}

static inline uint16_t adsReadRegRaw(uint8_t reg){
  Wire.beginTransmission(0x48); Wire.write(reg); Wire.endTransmission();
  Wire.requestFrom((int)0x48, 2); uint16_t v=0;
  if (Wire.available()>=2) v = (Wire.read()<<8) | Wire.read();
  return v;
}

static inline uint16_t adsPgaBits(adsGain_t g){
  switch(g){ case GAIN_TWOTHIRDS:return 0<<9; case GAIN_ONE:return 1<<9; case GAIN_TWO:return 2<<9;
    case GAIN_FOUR:return 3<<9; case GAIN_EIGHT:return 4<<9; default:return 5<<9; }
}

static inline uint16_t adsDrBits(int sps){
  switch(sps){ case 8:return 0<<5; case 16:return 1<<5; case 32:return 2<<5; case 64:return 3<<5;
    case 128:return 4<<5; case 250:return 5<<5; case 475:return 6<<5; default:return 7<<5; }
}
// conservative conversion time (µs) by SPS
static inline uint32_t adsConvTimeUs(int sps){
  switch(sps){ case 860:return 1200; case 475:return 2200; case 250:return 4100;
               case 128:return 7900; case 64:return 15600; default: return (1000000UL/ (uint32_t)sps) + 300; }
}

// Single-shot read timed: start -> wait ~t_conv -> one OS check -> read
static bool adsSingleReadRaw_timed(uint8_t ch, adsGain_t gain, int rateSps, int16_t &raw){
  uint16_t mux = ((4u + (ch & 0x03)) << 12);  // AINx vs GND (A0..A3)
  uint16_t cfg = (1u<<15) | mux | adsPgaBits(gain) | (1u<<8) | adsDrBits(rateSps) | 0x0003;
  if (g_adsMutex) xSemaphoreTake(g_adsMutex, portMAX_DELAY);
  const uint32_t t0 = micros();
  adsWriteReg(0x01, cfg);

  // Busy wait most of the conversion time (no I2C traffic)
  const uint32_t waitUs = adsConvTimeUs(rateSps);
  while ((uint32_t)(micros() - t0) < waitUs) { /* spin */ }

  // One quick OS check (ready bit), then read result
  uint16_t c; do { c = adsReadRegRaw(0x01); } while (!(c & 0x8000));
  uint16_t u = adsReadRegRaw(0x00);
  if (g_adsMutex) xSemaphoreGive(g_adsMutex);
  raw = (int16_t)u;
  return true;
}

// Single-shot read of one channel, polling OS bit (bit15) — no sleeps.
static bool adsSingleReadRaw_fast(uint8_t ch, adsGain_t gain, int rateSps, int16_t &raw){
  // MUX bits [14:12]: AINx vs GND → 100 (A0), 101 (A1), 110 (A2), 111 (A3)
  uint16_t mux = ((4u + (ch & 0x03)) << 12);
  uint16_t cfg = 0;
  cfg |= 1u<<15;                 // OS = 1 (start single conversion)
  cfg |= mux;                    // input mux
  cfg |= adsPgaBits(gain);       // PGA
  cfg |= 1u<<8;                  // MODE = 1 (single-shot)
  cfg |= adsDrBits(rateSps);     // data rate
  cfg |= 0x0003;                 // COMP_QUE = 3 → disable comparator/ALERT

  if (g_adsMutex) xSemaphoreTake(g_adsMutex, portMAX_DELAY);
  adsWriteReg(0x01, cfg);

  // Poll OS ready; 475 SPS → ~2.1 ms; give headroom up to ~6 ms.
  const uint32_t t0 = micros();
  for (;;) {
    uint16_t c = adsReadRegRaw(0x01);
    if (c & 0x8000) break;                  // ready
    if ((uint32_t)(micros() - t0) > 6000) break; // timeout safety
    // no delay(); tight loop to avoid losing throughput
  }

  uint16_t u = adsReadRegRaw(0x00);         // conversion register
  if (g_adsMutex) xSemaphoreGive(g_adsMutex);

  raw = (int16_t)u;
  return true;
}

static String measFileName(){
  String ts = isoNowFileSafe();     // you already added this in a prior fix
  return "/meas/sess_" + ts + ".am1"; // binary extension
}

// Forward declarations for alarm helpers used in meas_task_bin
static const char* alarmStr(AlarmState s);
static AlarmState evalWithHyst(AlarmState cur, float mA);
static void updateAlarmGPIO();

static void meas_task_bin(void*){
  // Start timing
  g_startUs   = monotonicMicros();
  g_nextDueUs = g_startUs;  // not used in fast path, kept for reference
  g_hzLastMs  = millis();
  g_hzLastCount = 0;

  while (g_measActive) {
    int16_t raw[NUM_SENSORS] = {0,0,0,0};
    float mv[NUM_SENSORS]  = {0,0,0,0};
    float ma[NUM_SENSORS]  = {0,0,0,0};
    float pct[NUM_SENSORS] = {0,0,0,0};

    // Fast single-shot conversions paced by the ADC itself
    for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
      adsSingleReadRaw_timed(ch, g_gainCh[ch], g_rateCh[ch], raw[ch]);
      const float lsb = adsLSB_mV(g_gainCh[ch]);
      mv[ch] = raw[ch] * lsb;
      ma[ch] = (g_shuntCh[ch] > 0.1f) ? (mv[ch] / g_shuntCh[ch]) : 0.0f;
      float p  = ((ma[ch] - 4.0f) / 16.0f) * 100.0f;
      if (p < 0) p = 0; if (p > 100) p = 100;
      pct[ch] = p;

      g_lastMv[ch]  = mv[ch];
      g_lastmA[ch]  = ma[ch];
      g_lastPct[ch] = pct[ch];
    }

    const uint32_t nowMs = millis();

    auto evalDebounced = [&](uint8_t ch, float ma){
      AlarmState next = evalWithHyst(g_alarmCh[ch], ma);
      if (next != g_alarmCh[ch]) {
        if (nowMs - g_alarmLastChangeMs[ch] >= ALARM_MIN_DWELL_MS) {
          g_alarmLastChangeMs[ch] = nowMs;
          g_alarmCh[ch] = next;
          logLine(String("[ALARM] A") + ch + " " + alarmStr(next) + " @ " + String(ma,3) + " mA");
          updateAlarmGPIO();
        }
      }
    };

    for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
      evalDebounced(ch, ma[ch]);
    }

    uint32_t total = g_frameCount + g_batchFill;
    if (nowMs - g_hzLastMs >= 500) {
      uint32_t delta = total - g_hzLastCount;
      g_pairHz = (delta * 1000.0f) / (nowMs - g_hzLastMs);
      g_hzLastMs = nowMs;
      g_hzLastCount = total;
    }

    // Append frame to RAM batch and flush on size (your existing code)
    uint64_t nowUs = monotonicMicros();
    uint64_t elapsedUs = nowUs - g_startUs;
    const uint64_t kMaxUsForTicks = (uint64_t)UINT32_MAX * 10ULL;
    uint32_t t10 = (elapsedUs >= kMaxUsForTicks)
                     ? UINT32_MAX
                     : static_cast<uint32_t>(elapsedUs / 10ULL);

    MeasFrame fr{}; fr.t_10us = t10; for (uint8_t i=0;i<NUM_SENSORS;++i) fr.raw[i] = raw[i];
    g_batch[g_batchFill++] = fr;
    if (g_batchFill >= BATCH_FRAMES) flushBatch();

    // (No delay; the conversions fully pace the loop)
  }

  // Session ending: write any leftover frames
  flushBatch();

  // task exits
  g_measTask = nullptr;
  vTaskDelete(nullptr);
}

// --- Read ADS1115 config reg (0x01) and decode data rate ---
uint16_t adsReadReg(uint8_t reg){
  Wire.beginTransmission(0x48);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((int)0x48, 2);
  uint16_t v = 0;
  if (Wire.available()>=2) { v = (Wire.read()<<8) | Wire.read(); }
  return v;
}
static int drBitsToSps(uint8_t dr){
  switch(dr & 0x07){
    case 0: return 8;
    case 1: return 16;
    case 2: return 32;
    case 3: return 64;
    case 4: return 128;
    case 5: return 250;
    case 6: return 475;
    case 7: return 860;
  } return 0;
}
void handleAdsRegs(){
  uint16_t cfg = adsReadReg(0x01);
  uint8_t dr   = (cfg >> 5) & 0x07;   // DR bits
  uint8_t mux  = (cfg >> 12) & 0x07;  // MUX
  uint8_t pga  = (cfg >> 9) & 0x07;   // PGA (gain)
  uint8_t mode = (cfg >> 8) & 0x01;   // 0=continuous, 1=single-shot
  char buf[160];
  snprintf(buf, sizeof(buf),
    "{\"cfg\":\"0x%04X\",\"DR_bits\":%u,\"DR_sps\":%d,"
    "\"MUX\":%u,\"PGA\":%u,\"MODE\":\"%s\"}",
    cfg, dr, drBitsToSps(dr), mux, pga, mode? "single":"cont");
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json", buf);
}

static void ensureMeasDir(){
  if (!sdMounted) return;
  if (!SD.exists("/meas")) SD.mkdir("/meas");
  if (g_measDir.length() && !SD.exists(g_measDir.c_str())) SD.mkdir(g_measDir.c_str());
}

static String sessionFilePath(uint32_t idx){
  char buf[32];
  snprintf(buf, sizeof(buf), "/part_%04lu.am1", (unsigned long)idx);
  String path = g_measDir;
  path += buf;
  return path;
}

static uint32_t frameToFileIndex(const MeasFrame& fr){
  return (MEAS_FILE_SPAN_TICKS == 0) ? 0u : (fr.t_10us / MEAS_FILE_SPAN_TICKS);
}

static bool writeBinHeader(const String& path){
  if (!sdMounted || path.length() == 0) return false;
  ensureMeasDir();
  digitalWrite(WIZ_CS, HIGH);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;

  MeasHeader h{};
  memcpy(h.magic, "AM01", 4);
  h.ver          = MEAS_HEADER_VER;
  h.start_epoch  = (uint32_t)time(nullptr);
  h.time_scale_us= 10;                      // 10 µs ticks in frames
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    h.sps[ch]       = (uint16_t)g_measSps[ch];
    h.gain_code[ch] = gainToCode(g_gainCh[ch]);
    h.sh[ch]        = g_shuntCh[ch];
    h.fs[ch]        = g_engFSmm[ch];
    h.off[ch]       = g_engOffmm[ch];
  }

  f.write((uint8_t*)&h, sizeof(h));
  f.close();
  g_measBytes += sizeof(h);
  return true;
}

static bool switchToMeasFile(uint32_t rawIdx){
  if (g_measDir.length() == 0) return false;

  bool wrapped = false;
  bool forcedForward = false;

  uint64_t offset = g_measFileIdxOffset;

  if (MEAS_FILE_SPAN_TICKS != 0 && g_measHaveRawIdx) {
    if (rawIdx < g_measLastRawIdx) {
      offset += (uint64_t)g_measLastRawIdx + 1ULL;
      wrapped = true;
    }
  }

  uint64_t actualIdx64 = (MEAS_FILE_SPAN_TICKS == 0)
                          ? 0ULL
                          : (offset + (uint64_t)rawIdx);

  if (MEAS_FILE_SPAN_TICKS != 0 && g_measFileIndex != 0xFFFFFFFFu) {
    if (actualIdx64 < (uint64_t)g_measFileIndex) {
      // Clock/tick glitches can make rawIdx fall behind the file index.
      // Nudge the offset forward so numbering never goes backwards.
      uint64_t desired = (uint64_t)g_measFileIndex + 1ULL;
      offset = (desired > (uint64_t)rawIdx)
                 ? (desired - (uint64_t)rawIdx)
                 : 0ULL;
      actualIdx64 = offset + (uint64_t)rawIdx;
      forcedForward = true;
    }
  }

  if (actualIdx64 > UINT32_MAX) actualIdx64 = UINT32_MAX;
  g_measFileIdxOffset = offset;
  uint32_t actualIdx = (uint32_t)actualIdx64;

  g_measHaveRawIdx = true;
  g_measLastRawIdx = rawIdx;

  if (g_measFileIndex == actualIdx && g_measFile.length()) return true;

  String path = sessionFilePath(actualIdx);
  g_measFile = path;
  if (!writeBinHeader(path)) return false;

  g_measFileIndex = actualIdx;
  if (wrapped) {
    logLine(String("[MEAS] file wrap: raw idx ") + String(rawIdx)
            + " → " + path);
  } else if (forcedForward) {
    logLine(String("[MEAS] file rewind detected: raw idx ") + String(rawIdx)
            + " forcing → " + path);
  }
  logLine(String("[MEAS] file ready: ") + path);
  return true;
}

static void flushBatch(){
  if (g_batchFill == 0) return;
  if (!sdMounted) {
    g_batchFill = 0;
    return;
  }

  size_t pos = 0;
  while (pos < g_batchFill) {
    uint32_t idx = frameToFileIndex(g_batch[pos]);
    if (!switchToMeasFile(idx)) break;

    size_t end = pos + 1;
    while (end < g_batchFill && frameToFileIndex(g_batch[end]) == idx) {
      ++end;
    }

    digitalWrite(WIZ_CS, HIGH);
    File f = SD.open(g_measFile, FILE_APPEND);
    if (!f) break;

    size_t frames = end - pos;
    size_t bytes  = frames * sizeof(MeasFrame);
    f.write((uint8_t*)&g_batch[pos], bytes);
    f.close();

    g_measBytes += bytes;
    g_frameCount += frames;
    pos = end;
  }

  if (pos < g_batchFill) {
    size_t remaining = g_batchFill - pos;
    if (pos > 0 && remaining > 0) {
      memmove(g_batch, g_batch + pos, remaining * sizeof(MeasFrame));
      g_batchFill = remaining;
    } else {
      g_batchFill = 0;
    }
  } else {
    g_batchFill = 0;
  }
}

String makeHostname(String s) {
  s.toLowerCase();
  String out; out.reserve(s.length());
  for (char c : s) {
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-') out += c;
    else if (c == ' ' || c == '_' || c == '.') out += '-';
  }
  if (out.length() == 0) out = "esp32";
  if (out.length() > 31) out = out.substring(0, 31); // keep it short
  return out;
}

static String baseName(const String& p) {
  int i = p.lastIndexOf('/');
  return (i >= 0) ? p.substring(i+1) : p;
}

// Resolve hostname using the Ethernet DNS server. Falls back to cfg.dns or 1.1.1.1.
bool resolveHost(const char* host, IPAddress& out) {
  DNSClient dns;
  IPAddress dnsIP = Ethernet.dnsServerIP();
  if (dnsIP == IPAddress(0,0,0,0)) dnsIP = cfg.dns;            // your saved static DNS
  if (dnsIP == IPAddress(0,0,0,0)) dnsIP = IPAddress(1,1,1,1); // last resort
  dns.begin(dnsIP);
  return dns.getHostByName(host, out) == 1;
}

// --------- HELPERS ----------
static const char* alarmStr(AlarmState s){
  switch (s){ case ALARM_UNDER: return "UNDER"; case ALARM_OVER: return "OVER"; default: return "NORMAL"; }
}

static AlarmState evalWithHyst(AlarmState cur, float mA){
  if (cur == ALARM_NORMAL){
    if (mA < UC_SET)  return ALARM_UNDER;
    if (mA > OC_SET)  return ALARM_OVER;
    return ALARM_NORMAL;
  }
  if (cur == ALARM_UNDER){
    if (mA > UC_CLR)  return ALARM_NORMAL;
    if (mA > OC_SET)  return ALARM_OVER;   // direct jump allowed
    return ALARM_UNDER;
  }
  // cur == ALARM_OVER
  if (mA < OC_CLR)    return ALARM_NORMAL;
  if (mA < UC_SET)    return ALARM_UNDER;  // direct jump allowed
  return ALARM_OVER;
}

static void updateAlarmGPIO(){
  g_alarmActive = false;
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    if (g_alarmCh[ch] != ALARM_NORMAL) { g_alarmActive = true; break; }
  }
}

static void ledApplyLevel(LedState& led, bool on){
  if (&led == &g_ledNet) {
    // NET LED is driven by W5500 INT line; never override it from ESP32
    return;
  }

  if (led.level == on) return;
  digitalWrite(led.pin, on ? HIGH : LOW);
  led.level = on;
}

static void ledSetMode(LedState& led, LedMode mode){
  if (led.mode == mode) return;
  led.mode = mode;
}

static void ledTick(LedState& led, uint32_t now){
  bool on = false;
  switch (led.mode) {
    case LED_OFF:           on = false; break;
    case LED_ON:            on = true;  break;
    case LED_BLINK_SLOW:    on = (now % 1000UL) < 500UL; break;           // 1 Hz, 50% duty
    case LED_BLINK_FAST:    on = (now % 200UL)  < 100UL; break;           // 5 Hz, 50% duty
    case LED_HEARTBEAT:     on = (now % 1000UL) < 180UL; break;           // wider 180 ms flash every 1 s
    case LED_DOUBLE_BLINK: {
      uint32_t phase = now % 1400UL;                                      // two 120 ms pulses / 1.4 s
      on = (phase < 120UL) || (phase >= 260UL && phase < 380UL);
      break;
    }
  }
  ledApplyLevel(led, on);
}

static void statusLedsInit(){
  pinMode(LED_RUN_PIN, OUTPUT);   digitalWrite(LED_RUN_PIN,   LOW);
  pinMode(LED_ERROR_PIN, OUTPUT); digitalWrite(LED_ERROR_PIN, LOW);
  pinMode(LED_MEAS_PIN, OUTPUT);  digitalWrite(LED_MEAS_PIN,  LOW);

  // NET / INT: input only, W5500 controls it (LED via INT pulses)
  pinMode(LED_NET_PIN, INPUT);   // or INPUT_PULLUP if your hardware needs it
}

static void ledSelfTest(){
  const uint16_t onMs  = 180;
  const uint16_t gapMs = 80;
  // NET LED (INT pin) intentionally NOT included here
  LedState* leds[] = { &g_ledRun, &g_ledError, &g_ledMeas };

  for (auto* l : leds) {
    ledApplyLevel(*l, true);
    delay(onMs);
    ledApplyLevel(*l, false);
    delay(gapMs);
  }

  // All on briefly, then off to confirm shared supply strength
  for (auto* l : leds) ledApplyLevel(*l, true);
  delay(onMs);
  for (auto* l : leds) ledApplyLevel(*l, false);
}

static void updateStatusLeds(){
  uint32_t now = millis();

  ledSetMode(g_ledRun, LED_HEARTBEAT);

  // NET LED now purely reflects W5500 INT hardware line.
  // No ledSetMode(...) for g_ledNet here.

  if (g_alarmActive) {
    ledSetMode(g_ledError, LED_ON);
  } else if (!adsReady || g_adsFailCount > 0) {
    ledSetMode(g_ledError, LED_DOUBLE_BLINK);
  } else {
    ledSetMode(g_ledError, LED_OFF);
  }

  ledSetMode(g_ledMeas, g_measActive ? LED_ON : LED_OFF);

  ledTick(g_ledRun,   now);
  // ledTick(g_ledNet,   now);  // <- removed, INT drives this now
  ledTick(g_ledError, now);
  ledTick(g_ledMeas,  now);
}

void handleCloudDiag() {
  String url = cfg.serverUrl;
  String scheme, host, path; uint16_t port = 0;
  bool parsed = parseUrl(url, scheme, host, port, path);

  IPAddress dnsIP = Ethernet.dnsServerIP();
  IPAddress hostIP; bool dnsOk = false, tcpOk = false;

  if (parsed && scheme == "https" && host.length()) {
    dnsOk = resolveHost(host.c_str(), hostIP);
    if (dnsOk) {
      EthernetClient c; c.setTimeout(1500);
      tcpOk = c.connect(hostIP, port);
      c.stop();
    }
  }

  String j = "{";
  j += "\"url\":\"" + url + "\",";
  j += "\"parsed\":"; j += (parsed ? "true" : "false"); j += ",";
  j += "\"scheme\":\"" + scheme + "\",";
  j += "\"host\":\"" + host + "\",";
  j += "\"port\":" + String(port) + ",";
  j += "\"dns\":\"" + dnsIP.toString() + "\",";
  j += "\"resolved\":\""; j += (dnsOk ? hostIP.toString() : ""); j += "\",";
  j += "\"tcp\":"; j += (tcpOk ? "true" : "false");
  j += "}";

  server.sendHeader("Cache-Control","no-store");
  server.send(200, "application/json", j);
}

static void adsNoteError(const String& msg, int code) {
  g_adsFailCount++;
  g_adsLastErr   = msg + " (code " + String(code) + ")";
  g_adsLastErrMs = millis();
}

static int i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();         // 0 = OK (ACK), else error/NACK
}

static String jsonEscape(const String& s){
  String o; o.reserve(s.length()+4);
  for (size_t i=0; i<s.length(); ++i){
    char c = s[i];
    switch (c){
      case '\"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\b': o += "\\b";  break;
      case '\f': o += "\\f";  break;
      case '\n': o += "\\n";  break;
      case '\r': o += "\\r";  break;
      case '\t': o += "\\t";  break;
      default:
        if ((uint8_t)c < 0x20) { char u[7]; snprintf(u, sizeof(u), "\\u%04x", (unsigned char)c); o += u; }
        else o += c;
    }
  }
  return o;
}

// Save ALL fields, every time
// Commit ADS config to NVS atomically and durably
static void adsConfigSave(){
  if (!adsPrefsReady) {
    adsPrefsReady = adsPrefs.begin(ADS_PREF_NS, false);
  }
  if (!adsPrefsReady) {
    Serial.println("[ADS] NVS: open failed");
    return;
  }

  // selection
  adsPrefs.putUChar("sel",   (uint8_t)g_adsSel);

  // per-channel
  for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
    char key[8];
    snprintf(key, sizeof(key), "gain%u", ch); adsPrefs.putUChar(key, gainToCode(g_gainCh[ch]));
    snprintf(key, sizeof(key), "rate%u", ch); adsPrefs.putInt(key, g_rateCh[ch]);
    snprintf(key, sizeof(key), "sh%u",   ch); adsPrefs.putFloat(key, g_shuntCh[ch]);
  }

  // legacy shadows (keep for status/seed)
  adsPrefs.putUChar("gain",  gainToCode(g_gainCh[0]));
  adsPrefs.putInt(  "rate",  g_rateCh[0]);
  adsPrefs.putFloat("shunt", g_shuntCh[0]);

  // engineering units
  for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
    char key[8];
    snprintf(key, sizeof(key), "fs%u",  ch); adsPrefs.putFloat(key, g_engFSmm[ch]);
    snprintf(key, sizeof(key), "off%u", ch); adsPrefs.putFloat(key, g_engOffmm[ch]);
  }

  Serial.println("[ADS] NVS: saved");
}

// LOAD ADS CONFIG (seed NVS on first boot, no NOT_FOUND spam)
static void adsConfigLoad(){
  if (!adsPrefsReady) {
    adsPrefsReady = adsPrefs.begin(ADS_PREF_NS, false);
  }
  if (!adsPrefsReady) {
    Serial.println("[ADS] NVS: open failed (load)");
    return;
  }

  // ---- RAM defaults (these are your requested defaults) ----
  g_adsSel       = ADS_SEL_ALL;
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    g_gainCh[ch]   = GAIN_ONE;    // 4.096 V
    g_rateCh[ch]   = 250;         // 250 SPS
    g_shuntCh[ch]  = 160.0f;      // Ω
    g_engFSmm[ch]  = 40.0f;
    g_engOffmm[ch] = 0.0f;
  }

  // ---- Only read keys that exist to avoid NOT_FOUND logs ----
  if (adsPrefs.isKey("sel"))   g_adsSel       = (AdsSel)adsPrefs.getUChar("sel",  (uint8_t)ADS_SEL_ALL);

  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    char key[8];
    snprintf(key,sizeof(key),"gain%u", ch); if (adsPrefs.isKey(key)) g_gainCh[ch] = codeToGain(adsPrefs.getUChar(key, 1));
    snprintf(key,sizeof(key),"rate%u", ch); if (adsPrefs.isKey(key)) g_rateCh[ch] = adsPrefs.getInt(key, 250);
    snprintf(key,sizeof(key),"sh%u",   ch); if (adsPrefs.isKey(key)) g_shuntCh[ch]= adsPrefs.getFloat(key, 160.0f);

    snprintf(key,sizeof(key),"fs%u",   ch); if (adsPrefs.isKey(key)) g_engFSmm[ch]  = clampf(adsPrefs.getFloat(key, 40.0f), 1.0f, 10000.0f);
    snprintf(key,sizeof(key),"off%u",  ch); if (adsPrefs.isKey(key)) g_engOffmm[ch] = clampf(adsPrefs.getFloat(key, 0.0f), -100000.0f, 100000.0f);
  }

  // ---- Mirror A0 into legacy shadows (for chip seeding / status) ----
  g_adsGain      = g_gainCh[0];
  g_adsRateSps   = g_rateCh[0];
  g_shuntOhms    = g_shuntCh[0];

  // ---- One-time seed if first boot (so future loads don’t log NOT_FOUND) ----
  bool needSeed = false;
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    char key[8];
    snprintf(key,sizeof(key),"gain%u", ch); needSeed |= !adsPrefs.isKey(key);
    snprintf(key,sizeof(key),"rate%u", ch); needSeed |= !adsPrefs.isKey(key);
    snprintf(key,sizeof(key),"sh%u",   ch); needSeed |= !adsPrefs.isKey(key);
  }
  if (needSeed) {
    adsConfigSave();
    Serial.println("[ADS] NVS seeded with defaults");
  }

  String msg = "[ADS] loaded | sel=" + String((unsigned)g_adsSel) + " | ";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    msg += "A" + String(ch) + " gain=" + gainToStr(g_gainCh[ch]) + " rate=" + String(g_rateCh[ch]) + " sh=" + String(g_shuntCh[ch],1) + "Ω";
    if (ch + 1 < NUM_SENSORS) msg += "; ";
  }
  Serial.println(msg);
}

static void alarmsTask(){
  static uint32_t last = 0;
  if (millis() - last < 250) return;  // ~4 Hz
  last = millis();

  /* if (!adsReady) return;

  // Read both channels
  int16_t r; float mv, ma, pct;

  // A0
  adsReadCh(0, r, mv, ma, pct);
  AlarmState next0 = evalWithHyst(g_alarmCh[0], ma);
  if (next0 != g_alarmCh[0]) {
    logLine(String("[ALARM] A0 ") + alarmStr(next0) + String(" @ ") + String(ma,3) + " mA");
    g_alarmCh[0] = next0;
  }

  // A1
  adsReadCh(1, r, mv, ma, pct);
  AlarmState next1 = evalWithHyst(g_alarmCh[1], ma);
  if (next1 != g_alarmCh[1]) {
    logLine(String("[ALARM] A1 ") + alarmStr(next1) + String(" @ ") + String(ma,3) + " mA");
    g_alarmCh[1] = next1;
  }

  updateAlarmGPIO(); */
}

void adsInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!ads.begin(0x48, &Wire)) {
    adsReady = false;
    Serial.println("[ADS] not found @0x48");
    return;
  }

  // Seed with A0 defaults; reads will set per-channel config before sampling
  ads.setGain(g_adsGain);
  adsSetRateSps(ads, g_adsRateSps);

  adsReady = true;
  String msg = "[ADS] ready  ";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    msg += "A" + String(ch) + ": gain=" + gainToStr(g_gainCh[ch]) + " rate=" + String(g_rateCh[ch]) + " sh=" + String(g_shuntCh[ch],1) + "Ω";
    if (ch + 1 < NUM_SENSORS) msg += " | ";
  }
  Serial.println(msg);
}

// Convert one channel to raw / mV / mA / % of 4–20 mA span
static void adsReadCh(uint8_t ch, int16_t &raw, float &mv, float &ma, float &pct){
  if (ch>=NUM_SENSORS) ch=0;
  // Lock the ADS while we touch config+read
  if (g_adsMutex) xSemaphoreTake(g_adsMutex, portMAX_DELAY);

  ads.setGain(g_gainCh[ch]);
  adsSetRateSps(ads, g_rateCh[ch]);
  raw = ads.readADC_SingleEnded(ch);

  if (g_adsMutex) xSemaphoreGive(g_adsMutex);

  mv  = raw * adsLSB_mV(g_gainCh[ch]);          // mV per LSB depends on gain
  float sh = g_shuntCh[ch];
  ma  = (sh > 0.1f) ? (mv / sh) : 0.0f;         // mA = mV / Ω
  pct = ((ma - 4.0f) / 16.0f) * 100.0f;         // 4–20mA → 0–100%
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
}

static String gainStr(){  // returns string for the *default* gain (A0)
  switch(g_adsGain){
    case GAIN_TWOTHIRDS: return "6.144";
    case GAIN_ONE:       return "4.096";
    case GAIN_TWO:       return "2.048";
    case GAIN_FOUR:      return "1.024";
    case GAIN_EIGHT:     return "0.512";
    case GAIN_SIXTEEN:   return "0.256";
  }
  return "4.096";
}

static inline void deselectAll() {
  pinMode(WIZ_CS, OUTPUT); digitalWrite(WIZ_CS, HIGH);
  pinMode(SD_CS,  OUTPUT); digitalWrite(SD_CS,  HIGH);
}

bool parseIP(const String& s, IPAddress& out) {
  int a,b,c,d;
  if (sscanf(s.c_str(), "%d.%d.%d.%d", &a,&b,&c,&d) == 4) {
    if (a<0||b<0||c<0||d<0||a>255||b>255||c>255||d>255) return false;
    out = IPAddress((uint8_t)a,(uint8_t)b,(uint8_t)c,(uint8_t)d);
    return true;
  }
  return false;
}

// Ping ADS @0x48, auto-reinit I²C + ADS if it stops ACKing
static void adsWatchdog() {
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < 1500) return;   // ~1.5 s cadence
  lastCheck = millis();

  int rc = i2cPing(0x48);
  if (rc == 0) {
    // If device is present but adsReady is false, try to recover once
    if (!adsReady) {
      adsInit();              // runs begin() + sets adsReady
      adsApplyHW();           // reapplies gain/rate
      if (adsReady) {
        g_adsLastErr   = "recovered";
        g_adsLastErrMs = millis();
      }
    }
    return;
  }

  // Not ACKing: remember & attempt reinit with a short backoff
  adsNoteError("I2C NACK @0x48", rc);

  if (millis() - g_adsLastReinitMs > 3000) {  // 3 s backoff against thrashing
    g_adsLastReinitMs = millis();

    // Re-kick I²C and device
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    delay(2);
    adsInit();
    adsApplyHW();
  }
}

// Read last N lines from a file into 'out'. Returns true on success.
static bool readLastLines(File &f, int lines, String &out) {
  const size_t CHUNK = 512;
  if (!f) return false;
  size_t size = f.size();
  if (size == 0) { out = ""; return true; }

  int need = lines;
  size_t pos = size;
  char buf[CHUNK];
  int found = 0;
  size_t start = 0;

  while (pos > 0 && found <= need) {
    size_t toread = (pos >= CHUNK ? CHUNK : pos);
    pos -= toread;
    f.seek(pos);
    size_t n = f.readBytes(buf, toread);
    if (n == 0) break;
    for (int i = (int)n - 1; i >= 0; --i) {
      if (buf[i] == '\n') {
        found++;
        if (found > need) { start = pos + i + 1; goto locate_done; }
      }
    }
  }
  start = 0;
locate_done:
  const size_t MAX_BYTES = 20000;
  f.seek(start);
  out.reserve((size - start < MAX_BYTES) ? (size - start) : MAX_BYTES);
  while (f.available() && out.length() < (int)MAX_BYTES) {
    char tmp[256];
    int n = f.readBytes(tmp, sizeof(tmp));
    if (n <= 0) break;
    out.concat(String(tmp).substring(0, n));
  }
  return true;
}

// Encode a text block for SSE ("data: ..." per line)
static String sseEncode(const String& s) {
  String out; out.reserve(s.length() + 16);
  int start = 0;
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] == '\n') {
      out += "data: ";
      out += s.substring(start, i);
      out += "\n";
      start = i + 1;
    }
  }
  out += "data: ";
  out += s.substring(start);
  out += "\n\n";
  return out;
}

void loadCfg() {
  prefs.begin("app", true);
  cfg.devName   = prefs.getString("devName",   cfg.devName);
  cfg.serverUrl = prefs.getString("serverUrl", cfg.serverUrl);
  cfg.apiKey    = prefs.getString("apiKey",    cfg.apiKey);
  cfg.useStatic = prefs.getBool  ("useStatic", cfg.useStatic);

  IPAddress t;
  if (parseIP(prefs.getString("ip",   "0.0.0.0"), t)) cfg.ip=t;
  if (parseIP(prefs.getString("gw",   "0.0.0.0"), t)) cfg.gw=t;
  if (parseIP(prefs.getString("mask", "255.255.255.0"), t)) cfg.mask=t;
  if (parseIP(prefs.getString("dns",  "1.1.1.1"), t)) cfg.dns=t;

  // Cloud
  cfg.cloudEnabled = prefs.getBool   ("cloudEn",   cfg.cloudEnabled);
  cfg.cloudPeriodS = prefs.getUInt   ("cloudPer",  cfg.cloudPeriodS);
  cfg.tlsFp        = prefs.getString ("tlsfp",     cfg.tlsFp);
  prefs.end();
}

void saveCfg() {
  prefs.begin("app", false);
  prefs.putString("devName",   cfg.devName);
  prefs.putString("serverUrl", cfg.serverUrl);
  prefs.putString("apiKey",    cfg.apiKey);
  prefs.putBool  ("useStatic", cfg.useStatic);
  prefs.putString("ip",   cfg.ip.toString());
  prefs.putString("gw",   cfg.gw.toString());
  prefs.putString("mask", cfg.mask.toString());
  prefs.putString("dns",  cfg.dns.toString());
  // Cloud
  prefs.putBool  ("cloudEn",   cfg.cloudEnabled);
  prefs.putUInt  ("cloudPer",  cfg.cloudPeriodS);
  prefs.putString("tlsfp",     cfg.tlsFp);
  prefs.end();
}

String resetReasonStr() {
  switch(esp_reset_reason()){
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT:     return "EXT";
    case ESP_RST_SW:      return "SW";
    case ESP_RST_PANIC:   return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT:return "TASK_WDT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:return "BROWNOUT";
    case ESP_RST_SDIO:    return "SDIO";
    default:              return "OTHER";
  }
}
String uptimeStr() {
  uint64_t ms = millis();
  uint32_t s = ms/1000; uint32_t m = s/60; uint32_t h = m/60; uint32_t d = h/24;
  char buf[64];
  snprintf(buf,sizeof(buf),"%ud %02u:%02u:%02u",(unsigned)d,(unsigned)(h%24),(unsigned)(m%60),(unsigned)(s%60));
  return String(buf);
}

// ---- TIME/NTP ----
void setupTime() {
  // Europe/Budapest (CET/CEST)
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.google.com", "time.cloudflare.com");
}
String isoNow() {
  time_t now = time(nullptr);
  if (now < 1609459200) return "unsynced";  // before 2021 => not set
  struct tm t; localtime_r(&now, &t);
  char buf[40]; strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &t);
  return String(buf);
}
// Add near isoNow()
String isoNowFileSafe() {
  time_t now = time(nullptr);
  struct tm t; localtime_r(&now, &t);
  char buf[32];
  // local time, no timezone => no '+'
  strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &t);
  return String(buf);
}

// NTP over W5500 (UDP). Sets system time + Budapest TZ on success.
bool ntpSyncW5500(const char* host = "pool.ntp.org", uint16_t timeoutMs = 1500) {
  IPAddress ntpIP;
  if (!resolveHost(host, ntpIP)) return false;

  const int NTP_PACKET_SIZE = 48;
  uint8_t pkt[NTP_PACKET_SIZE] = {0};
  pkt[0] = 0x1B;  // LI=0, VN=3, Mode=3 (client)

  static bool udpBegun = false;
  if (!udpBegun) { ntpUDP.begin(12345); udpBegun = true; }  // open a local UDP port once

  if (!ntpUDP.beginPacket(ntpIP, 123)) return false;
  ntpUDP.write(pkt, NTP_PACKET_SIZE);
  ntpUDP.endPacket();

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (ntpUDP.parsePacket() >= NTP_PACKET_SIZE) {
      ntpUDP.read(pkt, NTP_PACKET_SIZE);
      uint32_t secs1900 = (uint32_t)pkt[40]<<24 | (uint32_t)pkt[41]<<16 | (uint32_t)pkt[42]<<8 | (uint32_t)pkt[43];
      const uint32_t seventyYears = 2208988800UL; // 1900->1970
      if (secs1900 >= seventyYears) {
        time_t epoch = secs1900 - seventyYears;
        struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
        settimeofday(&tv, nullptr);

        // Budapest timezone (CET/CEST)
        setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
        tzset();

        g_timeSynced = true;
        return true;
      }
    }
    delay(10);
  }
  return false;
}

// ---- LOGGING (size-based rotation) ----
void ensureLogsDir() {
  if (!sdMounted) return;
  if (!SD.exists("/logs")) SD.mkdir("/logs");
  if (!SD.exists(currentLogPath)) {
    File f = SD.open(currentLogPath, FILE_WRITE);
    if (f) f.close();
  }
}
void rotateLogsIfNeeded() {
  if (!sdMounted) return;
  File cur = SD.open(currentLogPath, FILE_READ); if (!cur) return;
  size_t sz = cur.size(); cur.close();
  if (sz <= MAX_LOG_SIZE) return;

  // delete oldest
  String oldest = "/logs/log" + String(MAX_LOG_FILES-1) + ".log";
  if (SD.exists(oldest)) SD.remove(oldest);
  // shift
  for (int i=MAX_LOG_FILES-2; i>=0; --i) {
    String from = "/logs/log" + String(i) + ".log";
    String to   = "/logs/log" + String(i+1) + ".log";
    if (SD.exists(from)) SD.rename(from, to);
  }
  // new current
  File f = SD.open("/logs/log0.log", FILE_WRITE);
  if (f) f.close();
}
void logLine(const String& msg) {
  Serial.println(msg);
  if (!sdMounted) return;
  ensureLogsDir();
  rotateLogsIfNeeded();
  File f = SD.open(currentLogPath, FILE_APPEND);
  if (!f) return;
  String line = isoNow() + " " + msg + "\n";
  f.print(line);
  f.close();
}

// ---- ETHERNET / QoL ----
bool ethernetDHCP() {
  Serial.println("[ETH] DHCP…");
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("[ETH] Link down — skipping DHCP");
    return false;
  }
  // try DHCP quickly so boot doesn't stall with no cable
  unsigned long start = millis(); bool ok=false;
  while (millis()-start < 3000) {
    if (Ethernet.begin(ethMac)) { ok=true; break; }
    if (Ethernet.linkStatus() == LinkOFF) break;
    delay(250);
  }
  return ok;
}
bool ethernetInit() {
  // reset WIZ
  pinMode(WIZ_RST, OUTPUT);
  digitalWrite(WIZ_RST, LOW);  delay(5);
  digitalWrite(WIZ_RST, HIGH); delay(60);

  SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, WIZ_CS);
  Ethernet.init(WIZ_CS);

  bool ok = ethernetDHCP();
  if (!ok && cfg.useStatic) {
    Serial.printf("[ETH] DHCP failed → static: ip=%s gw=%s mask=%s dns=%s\n",
      cfg.ip.toString().c_str(), cfg.gw.toString().c_str(),
      cfg.mask.toString().c_str(), cfg.dns.toString().c_str());
    Ethernet.begin(ethMac, cfg.ip, cfg.dns, cfg.gw, cfg.mask);
    ok = true;
  }
  if (ok) {
    Serial.print("[ETH] IP: ");      Serial.println(Ethernet.localIP());
    Serial.print("[ETH] Gateway: "); Serial.println(Ethernet.gatewayIP());
    Serial.print("[ETH] DNS: ");     Serial.println(Ethernet.dnsServerIP());
  } else {
    Serial.println("[ETH] No IP.");
  }
  return ok;
}
bool internetOK(uint16_t timeoutMs = 1500) {
  EthernetClient c; c.setTimeout(timeoutMs);
  if (!c.connect(testHost, testPort)) return false;
  c.stop(); return true;
}

static bool refreshInternetState(bool force=false, uint16_t timeoutMs = INTERNET_CHECK_TIMEOUT_MS) {
  uint32_t now = millis();
  if (!force && (now - g_lastInetCheckMs) < INTERNET_CHECK_INTERVAL_MS) {
    return g_internetOk;
  }
  g_lastInetCheckMs = now;
  if (Ethernet.linkStatus() != LinkON) {
    g_internetOk = false;
    return false;
  }
  g_internetOk = internetOK(timeoutMs);
  return g_internetOk;
}

void linkWatchdog() {
  EthernetLinkStatus lk = Ethernet.linkStatus();
  if (lk != lastLink) {
    lastLink = lk;
    logLine(String("[ETH] Link ") + (lk==LinkON?"UP":(lk==LinkOFF?"DOWN":"UNKNOWN")));
    if (lk==LinkON) {
      // got link → (re)acquire IP
      if (cfg.useStatic) {
        Ethernet.begin(ethMac, cfg.ip, cfg.dns, cfg.gw, cfg.mask);
      } else {
        ethernetDHCP();
      }
      refreshInternetState(true);
    } else {
      g_internetOk = false;
    }
  }
}

// ---- SD init ----
bool sdInit() {
  digitalWrite(WIZ_CS, HIGH); // deselect Ethernet
  SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] Mount FAIL");
    sdMounted = false;
    return false;
  }
  sdMounted = true;
  ensureLogsDir();
  return true;
}

// ---- WEB UTILS ----
String safePath(String p) {
  if (p.length()==0) p="/";
  if (p[0] != '/') p = "/" + p;
  p.replace("//","/");
  while (p.indexOf("..") >= 0) { p.replace("..",""); }
  if (p.length()>1 && p.endsWith("/")) p.remove(p.length()-1);
  return p;
}
String urlDecode(const String& in) {
  String out; out.reserve(in.length());
  char a,b; size_t i=0;
  auto hex=[&](char ch){ if(ch>='0'&&ch<='9') return ch-'0';
                         if(ch>='a'&&ch<='f') return ch-'a'+10;
                         if(ch>='A'&&ch<='F') return ch-'A'+10;
                         return 0; };
  while (i<in.length()) {
    char c=in[i++];
    if (c=='+') out+=' ';
    else if (c=='%' && i+1<in.length()) { a=in[i++]; b=in[i++]; out += char((hex(a)<<4)|hex(b)); }
    else out += c;
  }
  return out;
}
// Add next to urlDecode()
String urlDecodePath(const String& in) {
  String out; out.reserve(in.length());
  auto hex=[&](char ch){ if(ch>='0'&&ch<='9') return ch-'0';
                         if(ch>='a'&&ch<='f') return ch-'a'+10;
                         if(ch>='A'&&ch<='F') return ch-'A'+10;
                         return 0; };
  for (size_t i=0; i<in.length();) {
    char c=in[i++];
    if (c=='%' && i+1<in.length()) { char a=in[i++], b=in[i++]; out += char((hex(a)<<4)|hex(b)); }
    else out += c;   // ⬅️ keep '+' as '+'
  }
  return out;
}


// ---- ROUTES: Config & Status ----
void handleRoot(){
  String page(FPSTR(INDEX_HTML));
  page.replace("%DEVNAME%", cfg.devName);
  page.replace("%SERVERURL%", cfg.serverUrl);
  page.replace("%APIKEY%", cfg.apiKey);
  page.replace("%CLOUDCHK%",       cfg.cloudEnabled ? "checked" : "");
  page.replace("%PERIOD%",         String(cfg.cloudPeriodS));
  page.replace("%SHAFINGERPRINT%", cfg.tlsFp);
  page.replace("%DHCPSEL%", cfg.useStatic ? "" : "selected");
  page.replace("%STATICSEL%", cfg.useStatic ? "selected" : "");
  page.replace("%IPBOXDISP%", cfg.useStatic ? "block" : "none");
  page.replace("%IP%",   cfg.ip.toString());
  page.replace("%GW%",   cfg.gw.toString());
  page.replace("%MASK%", cfg.mask.toString());
  page.replace("%DNS%",  cfg.dns.toString());
  page.replace("%ADSRATE%", String(g_adsRateSps));
  page.replace("%ADSTYPE0%", (g_engFSmm[0] >= 80.0f) ? "80" : "40");
  page.replace("%ADSTYPE1%", (g_engFSmm[1] >= 80.0f) ? "80" : "40");
  page.replace("%ADSTYPE2%", (g_engFSmm[2] >= 80.0f) ? "80" : "40");
  page.replace("%ADSTYPE3%", (g_engFSmm[3] >= 80.0f) ? "80" : "40");
  page.replace("%APSSID%", apSsid);
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  server.send(200, "text/html", page);
}
void handleSave(){
  cfg.devName   = server.arg("devName");
  cfg.serverUrl = server.arg("serverUrl");
  cfg.apiKey    = server.arg("apiKey");
  cfg.useStatic = (server.arg("mode")=="static");
  IPAddress t;
  if (cfg.useStatic) {
    if (parseIP(server.arg("ip"), t))   cfg.ip=t;
    if (parseIP(server.arg("gw"), t))   cfg.gw=t;
    if (parseIP(server.arg("mask"), t)) cfg.mask=t;
    if (parseIP(server.arg("dns"), t))  cfg.dns=t;
  }
  // Cloud fields (optional in UI)
  cfg.cloudEnabled = (server.arg("cloud").length() > 0); // checkbox
  if (server.hasArg("period")) {
    uint32_t s = server.arg("period").toInt();
    if (s < 2) s = 2; if (s > 86400) s = 86400;
    cfg.cloudPeriodS = s;
  }
  if (server.hasArg("tlsfp")) cfg.tlsFp = server.arg("tlsfp");
  saveCfg();
  server.send(200,"application/json","{\"ok\":true}");
}
void handleReboot(){ server.send(200,"text/plain","Rebooting…"); delay(200); ESP.restart(); }

void handleStatus() {
  bool link = (Ethernet.linkStatus() == LinkON);
  bool inet = g_internetOk;
  String mdnsName = g_mdnsRunning ? (cfg.devName + ".local") : "off";

  String j = "{";
  j += "\"ethUp\":" + String((lastLink != Unknown) ? "true" : "false") + ",";
  j += "\"link\":"  + String(link ? "true" : "false") + ",";
  j += "\"inet\":" + String(g_internetOk?"true":"false") + ",";
  j += "\"ip\":\""  + Ethernet.localIP().toString() + "\",";
  j += "\"sd\":"    + String(sdMounted ? "true" : "false") + ",";
  j += "\"time\":\""+ isoNow() + "\",";
  j += "\"timesynced\":" + String(g_timeSynced ? "true" : "false") + ",";
  j += "\"uptime\":\"" + uptimeStr() + "\",";
  j += "\"reboot\":\"" + resetReasonStr() + "\",";
  j += "\"mdns\":\"" + mdnsName + "\"";

  // ---- cloud block ----
  j += ",\"cloud\":{";
  j +=   "\"enabled\":" + String(cfg.cloudEnabled?"true":"false") + ",";
  j +=   "\"period\":"  + String(cfg.cloudPeriodS) + ",";
  j +=   "\"lastCode\":"+ String(g_lastHttpCode) + ",";
  j +=   "\"lastAt\":\""+ g_lastPushIso + "\",";
  j +=   "\"err\":\""  + g_lastCloudErr + "\",";
  j +=   "\"nextSec\":"+ String(g_nextPushInS);
  j += "}";

  // ---- ADS reliability + alarms ----
  j += ",\"adsReady\":"      + String(adsReady ? "true":"false");
  j += ",\"adsFail\":"       + String((unsigned)g_adsFailCount);
  j += ",\"adsLastErr\":\""  + jsonEscape(g_adsLastErr) + "\"";
  j += ",\"adsLastErrAgo\":" + String(g_adsLastErrMs ? (millis()-g_adsLastErrMs) : 0);
  j += ",\"alarms\":[";
  bool anyAlarm = false;
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    if (ch) j += ",";
    j += "\"" + jsonEscape(String(alarmStr(g_alarmCh[ch]))) + "\"";
    if (g_alarmCh[ch] != ALARM_NORMAL) anyAlarm = true;
  }
  j += "],\"aAny\":" + String(anyAlarm ? "true":"false");

  j += "}";
  server.sendHeader("Cache-Control","no-store");
  server.send(200, "application/json", j);
}

// ---- ROUTES: SD Browser ----
void handleFsList(){
  String path = safePath(urlDecodePath(server.arg("path")));
  digitalWrite(WIZ_CS, HIGH);
  File dir = SD.open(path);
  if (!dir || !dir.isDirectory()) { server.send(200,"application/json","{\"ok\":false,\"err\":\"not a dir\"}"); return; }
  String out = "{\"ok\":true,\"path\":\""+path+"\",\"items\":[";
  bool first=true;
  File f = dir.openNextFile();
  while (f) {
    if (!first) out += ",";
    first=false;
    String name = f.name();
    if (name.startsWith(path+"/")) name = name.substring(path.length()+1);
    out += "{\"name\":\""+name+"\",\"size\":"+String((uint32_t)f.size())+",\"type\":\""+(f.isDirectory()?"dir":"file")+"\"}";
    f = dir.openNextFile();
  }
  dir.close();
  out += "]}";
  server.send(200,"application/json",out);
}
void handleDownload(){
  String path = safePath(urlDecodePath(server.arg("path")));
  digitalWrite(WIZ_CS, HIGH);
  File f = SD.open(path, FILE_READ);
  if (!f) {
    // fallback: if path had spaces, try '+' (handles old +0200 names)
    if (path.indexOf(' ')>=0) {
      String alt = path; alt.replace(" ","+");
      f = SD.open(alt, FILE_READ);
      if (f) path = alt;
    }
  }
  if (!f || f.isDirectory()) { server.send(404,"text/plain","Not found"); return; }
  server.sendHeader("Content-Disposition","attachment; filename=\""+baseName(path)+"\"");
  server.streamFile(f, "application/octet-stream");
  f.close();
}
bool deleteRecursive(const String& path) {
  File f = SD.open(path);
  if (!f) return false;
  if (!f.isDirectory()) { f.close(); return SD.remove(path); }
  File e = f.openNextFile();
  while (e) {
    String p = String(path) + "/" + e.name();
    e.close();
    if (!deleteRecursive(p)) { f.close(); return false; }
    e = f.openNextFile();
  }
  f.close();
  return SD.rmdir(path);
}
void handleDelete(){
  String path = safePath(urlDecodePath(server.arg("path")));
  bool ok = deleteRecursive(path);
  if (!ok && path.indexOf(' ')>=0) { // try '+' variant
    String alt = path; alt.replace(" ","+");
    ok = deleteRecursive(alt);
  }
  server.send(200,"application/json", ok?"{\"ok\":true}":"{\"ok\":false}");
}
void handleMkdir(){ String path = safePath(urlDecodePath(server.arg("path"))); bool ok = SD.mkdir(path); server.send(200,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}"); }
File _uploadFile;
void handleUploadPost() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    String dir = safePath(urlDecode(server.arg("dir"))); if (dir.length()==0) dir="/";
    String base = up.filename; base.replace("\\","/"); int pos=base.lastIndexOf('/'); if (pos>=0) base=base.substring(pos+1);
    if (dir != "/") SD.mkdir(dir);
    String path = dir + (dir=="/"?"":"/") + (base.length()?base:"upload.bin");
    digitalWrite(WIZ_CS, HIGH);
    _uploadFile = SD.open(path, FILE_WRITE);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (_uploadFile) _uploadFile.write(up.buf, up.currentSize);
  } else if (up.status == UPLOAD_FILE_END) {
    if (_uploadFile) _uploadFile.close();
    server.send(200,"application/json","{\"ok\":true}");
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (_uploadFile) { String n=_uploadFile.name(); _uploadFile.close(); SD.remove(n); }
    server.send(200,"application/json","{\"ok\":false}");
  }
}

// ---- ADS ----
// GET /ads — return current readings and config
void handleAdsGet(){
  server.sendHeader("Cache-Control","no-store");
  const bool ready = adsReady;

  // Determine which channel(s) to read (skip reads entirely if ADS is offline)
  uint8_t chans[NUM_SENSORS] = {0,1,2,3};
  size_t chanCount = ready ? NUM_SENSORS : 0;
  auto setSingle = [&](uint8_t ch){ chans[0] = ch; chanCount = 1; };

  if (ready) {
    if (server.hasArg("sel")){
      String s = server.arg("sel"); s.toLowerCase();
      if (s=="both") { chans[0]=0; chans[1]=1; chanCount=2; }
      else if (s=="all") { chanCount = NUM_SENSORS; }
      else {
        int c = s.toInt(); if (c<0) c=0; if (c >= (int)NUM_SENSORS) c = NUM_SENSORS-1;
        setSingle((uint8_t)c);
      }
    } else if (server.hasArg("ch")){
      int c = server.arg("ch").toInt(); if (c<0) c=0; if (c >= (int)NUM_SENSORS) c = NUM_SENSORS-1;
      setSingle((uint8_t)c);
    } else {
      switch(g_adsSel){
        case ADS_SEL_A1:   setSingle(1); break;
        case ADS_SEL_BOTH: chans[0]=0; chans[1]=1; chanCount=2; break;
        case ADS_SEL_ALL:  chanCount = NUM_SENSORS; break;
        case ADS_SEL_A0: default: setSingle(0); break;
      }
    }
  }

  String j = "{";
  j += "\"ok\":true,\"ready\":"; j += (ready ? "true," : "false,");
  j += "\"sel\":" + String((int)g_adsSel) + ",";
  j += "\"gain\":\""+gainToStr(g_adsGain)+"\",";            // legacy A0 view
  j += "\"rate\":" + String(g_adsRateSps) + ",";
  j += "\"shunt\":"+ String(g_shuntOhms,3) + ",";
  j += "\"units\":{\"fsmm\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_engFSmm[ch],1); }
  j += "],\"offmm\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_engOffmm[ch],3); }
  j += "]},";

  // Echo per-channel configuration as well (used by the UI to fill dropdowns)
  j += "\"cfg\":{";
  j +=   "\"gain\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += "\\\"" + gainToStr(g_gainCh[ch]) + "\\\""; }
  j +=   "],\"rate\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_rateCh[ch]); }
  j +=   "],\"shunt\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_shuntCh[ch],3); }
  j +=   "]";
  j += "},";

  bool useCached = g_measActive && ready;  // do not poke the ADS mid-session

  String mode = ready ? ((chanCount == NUM_SENSORS) ? "all" : ((chanCount>1)?"multi":"single")) : "none";
  j += "\"mode\":\"" + mode + "\",\"readings\":[";
  if (ready) {
    for (size_t idx=0; idx<chanCount; ++idx) {
      uint8_t ch = chans[idx];
      int16_t r=0; float mv=0,ma=0,p=0;
      if (useCached) {
        mv = g_lastMv[ch]; ma = g_lastmA[ch]; p = g_lastPct[ch];
      } else {
        adsReadCh(ch,r,mv,ma,p);
      }
      float mm = (p/100.0f)*g_engFSmm[ch] + g_engOffmm[ch];
      if (idx) j += ",";
      j += "{\"ch\":"+String(ch)+",\"raw\":"+String(r)+",\"mv\":"+String(mv,3)+",\"ma\":"+String(ma,3)+",\"pct\":"+String(p,1)+",\"mm\":"+String(mm,2)+"}";
    }
  }
  j += "]";

  server.send(200,"application/json", j);
}


static bool parseGainStr(const String& s, adsGain_t& out){
  String t=s; t.trim();
  if (t=="6.144") { out=GAIN_TWOTHIRDS; return true; }
  if (t=="4.096") { out=GAIN_ONE;       return true; }
  if (t=="2.048") { out=GAIN_TWO;       return true; }
  if (t=="1.024") { out=GAIN_FOUR;      return true; }
  if (t=="0.512") { out=GAIN_EIGHT;     return true; }
  if (t=="0.256") { out=GAIN_SIXTEEN;   return true; }
  return false;
}

static String gainToStr(adsGain_t g){
  switch(g){
    case GAIN_TWOTHIRDS: return "6.144";
    case GAIN_ONE:       return "4.096";
    case GAIN_TWO:       return "2.048";
    case GAIN_FOUR:      return "1.024";
    case GAIN_EIGHT:     return "0.512";
    default:             return "0.256";
  }
}

// POST /adsconf — set selection/gain/rate/shunt
void handleAdsConf(){
  server.sendHeader("Cache-Control","no-store");

  if (g_measActive) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"measuring\"}");
    return;
  }

  bool touchedSel = false;
  bool touchedElec = false;
  bool touchedUnits = false;

  // ----- parse selection -----
  if (server.hasArg("sel")) {
    String s = server.arg("sel"); s.trim(); s.toLowerCase();
    if      (s=="all")  g_adsSel = ADS_SEL_ALL;
    else if (s=="both") g_adsSel = ADS_SEL_BOTH;
    else if (s=="1" || s=="a1" || s=="ch1") g_adsSel = ADS_SEL_A1;
    else g_adsSel = ADS_SEL_A0;
    touchedSel = true;
  }

  // ----- per-channel electrical -----
  adsGain_t gt;
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    String k = String("gain") + ch;
    if (server.hasArg(k) && parseGainStr(server.arg(k), gt)) { g_gainCh[ch]=gt; touchedElec = true; }
  }
  if (server.hasArg("gain")  && parseGainStr(server.arg("gain"),  gt)) {
    for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) g_gainCh[ch]=gt; touchedElec = true;
  }

  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    String k = String("rate") + ch;
    if (server.hasArg(k)) { int s=server.arg(k).toInt(); if(validSps(s)) { g_rateCh[ch]=s; touchedElec = true; } }
  }
  if (server.hasArg("rate"))  { int s=server.arg("rate").toInt();  if(validSps(s)) { for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) g_rateCh[ch]=s; touchedElec = true; } }

  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    String k = String("shunt") + ch;
    if (server.hasArg(k)) { float v=server.arg(k).toFloat(); if(v>0.1f&&v<10000.0f) { g_shuntCh[ch]=v; touchedElec = true; } }
  }
  if (server.hasArg("shunt"))  { float v=server.arg("shunt").toFloat();  if(v>0.1f&&v<10000.0f) { for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) g_shuntCh[ch]=v; touchedElec = true; } }

  // ----- units -----
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    String tkey = String("type") + ch;
    String okey = String("off") + ch;
    if (server.hasArg(tkey)) { g_engFSmm[ch] = (server.arg(tkey).indexOf("80")>=0) ? 80.0f : 40.0f; touchedUnits = true; }
    if (server.hasArg(okey))  { g_engOffmm[ch]= clampf(server.arg(okey).toFloat(), -100000.0f, 100000.0f); touchedUnits = true; }
  }

  // mirror A0 into legacy shadows (used by seed/status)
  g_adsGain    = g_gainCh[0];
  g_adsRateSps = g_rateCh[0];
  g_shuntOhms  = g_shuntCh[0];

  // ----- persist to NVS (atomic) -----
  adsConfigSave();

  // ----- optionally touch hardware if present -----
  if (adsReady) adsApplyHW();

  // ----- verify by reloading from NVS and echo back -----
  adsConfigLoad();

  String logMsg = "[ADS] saved";
  bool havePart = false;
  if (touchedElec){
    logMsg += ": ";
    for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
      logMsg += "A" + String(ch) + " gain=" + gainToStr(g_gainCh[ch]) + " rate=" + String(g_rateCh[ch]) + " sh=" + String(g_shuntCh[ch],1) + "Ω";
      if (ch + 1 < NUM_SENSORS) logMsg += "; ";
    }
    havePart = true;
  }
  if (touchedUnits){
    logMsg += havePart ? " | " : ": ";
    logMsg += "units fs=[";
    for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) logMsg += ","; logMsg += String(g_engFSmm[ch],1); }
    logMsg += "] off=[";
    for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) logMsg += ","; logMsg += String(g_engOffmm[ch],3); }
    logMsg += "]";
    havePart = true;
  }
  if (touchedSel){
    logMsg += havePart ? " | " : ": ";
    logMsg += "sel=" + String((int)g_adsSel);
    havePart = true;
  }
  if (!havePart) logMsg += ": no changes";
  logLine(logMsg);

  String j="{";
  j += "\"ok\":true,";
  j += "\"sel\":" + String((int)g_adsSel) + ",";
  j += "\"cfg\":{\"gain\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += "\""+gainToStr(g_gainCh[ch])+"\""; }
  j += "],\"rate\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_rateCh[ch]); }
  j += "],\"shunt\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_shuntCh[ch],3); }
  j += "]}";
  j += "}";
  server.send(200,"application/json",j);
}


// Parse https://host[:port]/path  → scheme, host, port, path
static bool parseUrl(const String& url, String& scheme, String& host, uint16_t& port, String& path){
  scheme = host = path = ""; port = 0;
  int p = url.indexOf("://"); if (p < 0) return false;
  scheme = url.substring(0,p); String rest = url.substring(p+3);
  int slash = rest.indexOf('/'); String hostport = (slash<0)?rest:rest.substring(0,slash);
  path = (slash<0)?"/":rest.substring(slash);
  int colon = hostport.indexOf(':');
  if (colon>=0){ host = hostport.substring(0,colon); port = (uint16_t)hostport.substring(colon+1).toInt(); }
  else { host = hostport; port = 0; }
  if (port==0) port = (scheme=="https")?443:80;
  return true;
}

// Parse "AA:BB:..:ZZ" → 20 bytes (SHA1)
static bool parseSha1Fp(const String& s, uint8_t out[20]){
  int n=0; int i=0; while (i < (int)s.length() && n<20){
    while (i<(int)s.length() && (s[i]==':' || s[i]==' ')) i++;
    if (i+1 >= (int)s.length()) break;
    auto hex = [](char c)->int{
      if (c>='0'&&c<='9') return c-'0';
      if (c>='a'&&c<='f') return 10+(c-'a');
      if (c>='A'&&c<='F') return 10+(c-'A');
      return -1;
    };
    int hi = hex(s[i++]); int lo = hex(s[i++]); if (hi<0||lo<0) return false;
    out[n++] = (uint8_t)((hi<<4)|lo);
  }
  return n==20;
}

static String jsonSnapshot(bool withReadings=true){
  String j = "{";
  j += "\"dev\":\""+cfg.devName+"\",";
  j += "\"ip\":\""+Ethernet.localIP().toString()+"\",";
  j += "\"time\":\""+isoNow()+"\",";
  j += "\"uptime\":\""+uptimeStr()+"\",";
  j += "\"inet\":" + String(internetOK()?"true":"false") + ",";
  // alarms (if you have these globals)
  // expose a0/a1 if present in your code
  // Readings (both channels)
  if (withReadings && adsReady){
    int16_t r0=0,r1=0; float mv0=0,mv1=0, ma0=0,ma1=0, p0=0,p1=0;
    adsReadCh(0,r0,mv0,ma0,p0);
    adsReadCh(1,r1,mv1,ma1,p1);
    float mm0 = mapToMM(0,p0), mm1 = mapToMM(1,p1);
    j += "\"ads\":{";
    j += "\"c0\":{\"raw\":"+String(r0)+",\"mv\":"+String(mv0,3)+",\"ma\":"+String(ma0,3)+",\"pct\":"+String(p0,1)+",\"mm\":"+String(mm0,2)+"},";
    j += "\"c1\":{\"raw\":"+String(r1)+",\"mv\":"+String(mv1,3)+",\"ma\":"+String(ma1,3)+",\"pct\":"+String(p1,1)+",\"mm\":"+String(mm1,2)+"}";
    j += "},";
  }
  // measurement
  j += "\"meas\":{\"active\":"+String(g_measActive?"true":"false")+",\"id\":\""+g_measId+"\"}";
  j += "}";
  return j;
}

void handleAdsDump(){
  server.sendHeader("Cache-Control","no-store");
  String j = "{";
  j += "\"sel\":" + String((int)g_adsSel) + ",";
  j += "\"gain\":\"" + gainToStr(g_adsGain) + "\",";
  j += "\"rate\":" + String(g_adsRateSps) + ",";
  j += "\"shunt\":" + String(g_shuntOhms,3) + ",";
  j += "\"fsmm\":[" + String(g_engFSmm[0],1) + "," + String(g_engFSmm[1],1) + "],";
  j += "\"offmm\":[" + String(g_engOffmm[0],3) + "," + String(g_engOffmm[1],3) + "]";
  j += "}";
  server.send(200,"application/json", j);
}

// ---- ROUTES: Logs ----

// POST JSON to cfg.serverUrl over HTTPS
// ---- helpers ----
static bool isRedirect(int code){ return code==301 || code==302 || code==303 || code==307 || code==308; }

static bool readStatusAndHeaders(SSLClient& tls, int& code, String& location, uint32_t firstByteTimeoutMs=6000){
  code = -1; location = "";
  uint32_t start = millis();
  // status line
  String line="";
  while (tls.connected() && millis()-start < firstByteTimeoutMs) {
    if (tls.available()) { line = tls.readStringUntil('\n'); break; }
    delay(1);
  }
  if (!line.startsWith("HTTP/1.1 ")) return false;
  code = line.substring(9, 12).toInt();

  // headers
  while (tls.connected()) {
    String h = tls.readStringUntil('\n');
    if (h.length()==0 || h=="\r") break;        // end of headers
    if (h.startsWith("Location:") || h.startsWith("location:")) {
      int i = h.indexOf(':'); String v = h.substring(i+1);
      v.trim(); if (v.endsWith("\r")) v.remove(v.length()-1);
      location = v;
    }
  }
  return true;
}

static String absolutizeLocation(const String& baseUrl, const String& loc) {
  if (loc.startsWith("http://") || loc.startsWith("https://")) return loc;   // absolute
  // relative → use scheme/host/port from base
  String scheme, host, path; uint16_t port;
  if (!parseUrl(baseUrl, scheme, host, port, path)) return baseUrl;
  String p = loc;
  if (!p.startsWith("/")) { // relative path (no leading slash) — resolve against current dir
    int cut = path.lastIndexOf('/'); String dir = (cut>=0? path.substring(0,cut+1) : "/");
    p = dir + p;
  }
  return scheme + "://" + host + (port==443 && scheme=="https" ? "" : (String(":")+port)) + p;
}

// ---- POST JSON with redirect support ----
static bool httpsPostJson(const String& urlIn, const String& bearer, const String& body,
                          int& outCode, String& outResp, String& outErr)
{
  outCode=-1; outResp=""; outErr="";
  String nextUrl = urlIn;

  for (int hops=0; hops<3; ++hops) {                 // follow up to 3 redirects
    String scheme, host, path; uint16_t port;
    if (!parseUrl(nextUrl, scheme, host, port, path) || scheme!="https") { outErr="bad url"; return false; }

    if (_tls.connected()) _tls.stop();
    if (_tcp.connected()) _tcp.stop();
    tlsPrepare(host);
    if (!_tls.connect(host.c_str(), port)) { outErr="connect fail"; return false; }

    String req;
    req.reserve(256 + body.length());
    req += "POST " + path + " HTTP/1.1\r\n";
    req += "Host: " + host + "\r\n";
    req += "User-Agent: ESP32-W5500\r\n";
    req += "Content-Type: application/json\r\n";
    req += "X-DEV: " + cfg.devName + "\r\n";
    if (bearer.length()) req += "X-API-KEY: " + bearer + "\r\n";
    req += "Content-Length: " + String(body.length()) + "\r\n";
    req += "Connection: close\r\n\r\n";

    _tls.print(req);
    _tls.print(body);

    String loc;
    if (!readStatusAndHeaders(_tls, outCode, loc)) { outErr="no status"; _tls.stop(); return false; }

    // read body (even on redirects; some servers send helpful text)
    for (;;) {
      int b = _tls.read();
      if (b < 0) break;
    }
    _tls.stop();

    if (isRedirect(outCode)) {
      if (loc.length()==0) { outErr="redirect w/o Location"; return false; }
      nextUrl = absolutizeLocation(nextUrl, loc);
      continue; // follow
    }
    return (outCode>=200 && outCode<300);
  }
  outErr="too many redirects";
  return false;
}

// Upload a file as raw octet-stream to <url>?upload=1&name=<base>
static bool httpsUploadFile(const String& urlIn, const String& bearer, const String& filePath,
                            int& outCode, String& outResp, String& outErr)
{
  outCode=-1; outResp=""; outErr="";
  if (!SD.exists(filePath)) { outErr="no file"; return false; }

  String nextUrl = urlIn;
  for (int hops=0; hops<3; ++hops) {
    File f = SD.open(filePath, FILE_READ); if (!f) { outErr="open fail"; return false; }

    String scheme, host, path; uint16_t port;
    if (!parseUrl(nextUrl, scheme, host, port, path) || scheme!="https") { f.close(); outErr="bad url"; return false; }

    // append upload query
    String fullPath = path + (path.indexOf('?')>=0?"&":"?") + "upload=1&name=" + baseName(filePath);

    if (_tls.connected()) _tls.stop();
    if (_tcp.connected()) _tcp.stop();
    tlsPrepare(host);
    if (!_tls.connect(host.c_str(), port)) { f.close(); outErr="connect fail"; return false; }

    uint32_t len = f.size();
    String hdr;
    hdr.reserve(256);
    hdr += "POST " + fullPath + " HTTP/1.1\r\n";
    hdr += "Host: " + host + "\r\n";
    hdr += "User-Agent: ESP32-W5500\r\n";
    hdr += "Content-Type: application/octet-stream\r\n";
    hdr += "X-DEV: " + cfg.devName + "\r\n";
    if (bearer.length()) hdr += "X-API-KEY: " + bearer + "\r\n";
    hdr += "Content-Length: " + String(len) + "\r\n";
    hdr += "Connection: close\r\n\r\n";

    _tls.print(hdr);

    uint8_t buf[1024];
    while (f.available()) {
      int n = f.read(buf, sizeof(buf));
      if (n>0) _tls.write(buf, n);
      yield();
    }
    f.close();

    String loc;
    if (!readStatusAndHeaders(_tls, outCode, loc)) { outErr="no status"; _tls.stop(); return false; }

    for (;;) {
      int b = _tls.read();
      if (b < 0) break;
    }
    _tls.stop();

    if (isRedirect(outCode)) {
      if (loc.length()==0) { outErr="redirect w/o Location"; return false; }
      nextUrl = absolutizeLocation(nextUrl, loc);
      continue; // follow
    }
    return (outCode>=200 && outCode<300);
  }
  outErr="too many redirects";
  return false;
}

static bool pushCloudNow(bool includeReadings=true){
  if (!cfg.cloudEnabled) { g_lastCloudErr="disabled"; return false; }
  if (Ethernet.linkStatus() != LinkON || !g_internetOk) {
    g_lastHttpCode = -1;
    g_lastCloudErr = "offline";
    g_lastPushIso  = isoNow();
    return false;
  }
  String body = jsonSnapshot(includeReadings);
  int code; String resp, err;
  bool ok = httpsPostJson(cfg.serverUrl, cfg.apiKey, body, code, resp, err);
  g_lastHttpCode = code; g_lastCloudErr = err; g_lastPushIso = isoNow();
  logLine(String("[CLOUD] POST ")+ (ok?"OK ":"FAIL ") + "code="+String(code) + (err.length()?(" err="+err):""));
  if (!ok) {
    g_internetOk = false;
    g_lastInetCheckMs = millis();
  }
  return ok;
}

// Try upload of a session log to the same base URL (server should accept it)
static bool uploadLastSession(){
  if (g_measFile.length()==0) return false;
  if (Ethernet.linkStatus() != LinkON || !g_internetOk) {
    g_lastHttpCode = -1;
    g_lastCloudErr = "offline";
    g_lastPushIso  = isoNow();
    return false;
  }
  int code; String resp, err;
  bool ok = httpsUploadFile(cfg.serverUrl, cfg.apiKey, g_measFile, code, resp, err);
  g_lastHttpCode = code; g_lastCloudErr = err; g_lastPushIso = isoNow();
  logLine(String("[CLOUD] UPLOAD ")+ baseName(g_measFile) + " → " + (ok?"OK ":"FAIL ") + "code="+String(code));
  if (!ok) {
    g_internetOk = false;
    g_lastInetCheckMs = millis();
  }
  return ok;
}

// --- Cloud test ---
void handleCloudTest(){
  // Allow overriding URL/API via query/body for quick tests:
  String url = server.hasArg("url") ? urlDecode(server.arg("url")) : cfg.serverUrl;
  String key = server.hasArg("api") ? server.arg("api") : cfg.apiKey;

  // Validate URL
  String scheme, host, path; uint16_t port;
  bool parsed = parseUrl(url, scheme, host, port, path);
  if (!parsed || scheme != "https" || host.length()==0) {
    String j = "{\"ok\":false,\"err\":\"set a valid https URL in serverUrl or pass ?url=...\",\"url\":\""+url+"\"}";
    server.sendHeader("Cache-Control","no-store");
    server.send(200, "application/json", j);
    return;
  }

  // Build a tiny test payload (no readings)
  String body = jsonSnapshot(false);

  int code=-1; String resp, err;
  bool ok = httpsPostJson(url, key, body, code, resp, err);

  String j = "{";
  j += "\"ok\":" + String(ok?"true":"false") + ",";
  j += "\"code\":" + String(code) + ",";
  j += "\"err\":\"" + err + "\",";
  j += "\"url\":\"" + url + "\"";
  j += "}";
  server.sendHeader("Cache-Control","no-store");
  server.send(200, "application/json", j);
}

// --- Measurement control ---
void handleMeasStart(){
  if (g_measActive) { server.send(200,"application/json","{\"ok\":true,\"already\":true}"); return; }

  // Optional rate override (so web UI "Rate" affects the next run without a separate save)
  if (server.hasArg("rate")) {
    int sps = server.arg("rate").toInt();
    if (validSps(sps)) {
      for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) g_rateCh[ch] = sps;
      g_adsRateSps = sps;
      adsConfigSave();
      adsConfigLoad();
      if (adsReady) adsApplyHW();
    }
  }

  // snapshot SPS → dt (for status)
  int usedChannels = 0;
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    g_measSps[ch] = g_rateCh[ch];
    if (g_rateCh[ch] > 0) usedChannels++;
  }
  if (usedChannels == 0) usedChannels = 1;
  const float usedChannelsF = float(usedChannels);
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    g_measDtMs[ch] = (g_measSps[ch]>0)? ((1000.0f * usedChannelsF)/float(g_measSps[ch])) : 0.0f;
  }

  g_measId     = isoNowFileSafe();
  g_measDir    = "/meas/sess_" + g_measId;
  g_measFile   = "";
  g_measFileIndex = 0xFFFFFFFFu;
  g_measFileIdxOffset = 0;
  g_measHaveRawIdx = false;
  g_measLastRawIdx = 0;
  g_frameCount = 0;
  g_measBytes  = 0;
  g_batchFill  = 0;
  if (!switchToMeasFile(0)) {
    g_measId        = "";
    g_measDir       = "";
    g_measFile      = "";
    g_measFileIndex = 0xFFFFFFFFu;
    g_measBytes     = 0;
    logLine("[MEAS] start failed: cannot open first file");
    server.send(200,"application/json","{\"ok\":false,\"err\":\"sd write fail\"}");
    return;
  }

  g_measActive = true;
  g_pairHz     = 0.0f;

  // High-ish priority, pin to core 0 so server() can breathe on the other core
  TaskHandle_t taskHandle = nullptr;
 BaseType_t ok = xTaskCreatePinnedToCore(meas_task_bin, "meas_bin", 6144, nullptr, 2, &g_measTask, 0);
if (ok != pdPASS) {
  g_measActive = false;
  server.send(200,"application/json","{\"ok\":false,\"err\":\"task create fail\"}");
  return;
}

  String spsLog = "[MEAS] start BIN: " + g_measFile + " | SPS=";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) spsLog += "/"; spsLog += String(g_measSps[ch]); }
  logLine(spsLog);
  server.send(200,"application/json","{\"ok\":true}");
}

void handleMeasStop(){
  if (!g_measActive) { server.send(200,"application/json","{\"ok\":true,\"already\":true}"); return; }
  g_measActive = false;

  // Wait briefly for the task to exit & flush
  uint32_t t0 = millis();
  while (g_measTask && millis() - t0 < 800) { delay(10); }

  logLine("[MEAS] stop BIN: " + g_measFile);

  bool upOK = false;
  if (cfg.cloudEnabled) upOK = uploadLastSession();

  String j = String("{\"ok\":true,\"uploaded\":") + (upOK?"true":"false") + ",\"file\":\""+g_measFile+"\"}";
  server.send(200,"application/json", j);
}

void handleMeasDebug(){
  String j;
  j.reserve(120);
  j += "{\"active\":";
  j += g_measActive ? "true" : "false";
  j += ",\"task\":\"";
  j += g_measTask ? "yes" : "no";
  j += "\",\"batch\":";
  j += String((unsigned)g_batchFill);
  j += ",\"frames\":";
  j += String((unsigned)g_frameCount);
  j += ",\"frame_hz\":";
  j += String(g_pairHz,1);
  j += "}";
  server.send(200,"application/json", j);
}

void handleMeasStatus(){
  String j = "{";
  j += "\"active\":" + String(g_measActive?"true":"false") + ",";
  j += "\"id\":\"" + g_measId + "\",\"file\":\"" + g_measFile + "\",";
  j += "\"frames\":" + String((unsigned)g_frameCount) + ",";
  j += "\"bytes\":" + String((unsigned long long)g_measBytes) + ",";

  j += "\"sps\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    if (ch) j += ",";
    j += String(g_measSps[ch]);
  }
  j += "],";

  j += "\"dt_ms\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    if (ch) j += ",";
    j += String(g_measDtMs[ch],3);
  }
  j += "],";

  j += "\"frame_hz\":" + String(g_pairHz,1);
  j += "}";
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json", j);
}

// ---- /export_csv?path=/meas/xxx.am1[&cols=full|raw|rawmv] ----
// full  = t_s,raw0..rawN,mV0..mVN,mA0..mAN,mm0..mmN  (default)
// raw   = t_s,raw0..rawN
// rawmv = t_s,raw0..rawN,mV0..mVN
void handleExportCsv(){
  String path = safePath(urlDecodePath(server.arg("path")));
  digitalWrite(WIZ_CS, HIGH);
  File f = SD.open(path, FILE_READ);
  if (!f) { server.send(404,"text/plain","Not found"); return; }

  // --- read header ---
  struct __attribute__((packed)) BaseHeader {
    char     magic[4]; uint16_t ver, reserved;
    uint32_t start_epoch, time_scale_us;
  } base{};
  struct __attribute__((packed)) MeasHeaderV1 {
    BaseHeader base; uint16_t sps0, sps1; uint8_t gain0_code, gain1_code;
    float    sh0, sh1, fs0, fs1, off0, off1;
  };
  struct __attribute__((packed)) MeasHeaderV2 {
    BaseHeader base; uint16_t sps[NUM_SENSORS]; uint8_t gain_code[NUM_SENSORS];
    float    sh[NUM_SENSORS], fs[NUM_SENSORS], off[NUM_SENSORS];
  };

  if (f.read((uint8_t*)&base, sizeof(base)) != sizeof(base) || memcmp(base.magic,"AM01",4)!=0) {
    f.close(); server.send(400,"text/plain","Bad header"); return;
  }

  uint16_t ver = base.ver;
  uint8_t channels = (ver <= 1) ? 2 : NUM_SENSORS;
  uint16_t sps[NUM_SENSORS] = {0};
  uint8_t  gainCode[NUM_SENSORS] = {0};
  float    sh[NUM_SENSORS] = {0};
  float    fs[NUM_SENSORS] = {0};
  float    off[NUM_SENSORS]= {0};

  f.seek(0);
  if (ver <= 1) {
    MeasHeaderV1 h{};
    if (f.read((uint8_t*)&h, sizeof(h)) != sizeof(h)) { f.close(); server.send(400,"text/plain","Bad header"); return; }
    sps[0]=h.sps0; sps[1]=h.sps1;
    gainCode[0]=h.gain0_code; gainCode[1]=h.gain1_code;
    sh[0]=h.sh0; sh[1]=h.sh1; fs[0]=h.fs0; fs[1]=h.fs1; off[0]=h.off0; off[1]=h.off1;
  } else {
    MeasHeaderV2 h{};
    if (f.read((uint8_t*)&h, sizeof(h)) != sizeof(h)) { f.close(); server.send(400,"text/plain","Bad header"); return; }
    for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { sps[ch]=h.sps[ch]; gainCode[ch]=h.gain_code[ch]; sh[ch]=h.sh[ch]; fs[ch]=h.fs[ch]; off[ch]=h.off[ch]; }
  }

  // Decide columns
  String cols = server.hasArg("cols") ? server.arg("cols") : "full";
  bool wantMV    = (cols=="rawmv" || cols=="full");
  bool wantFULL  = (cols=="full");

  // Prepare response: force download with .csv filename
  String baseNm = baseName(path);
  int dot = baseNm.lastIndexOf('.'); if (dot>0) baseNm = baseNm.substring(0,dot);
  server.sendHeader("Cache-Control","no-store");
  server.sendHeader("Content-Disposition", "attachment; filename=\""+baseNm+".csv\"");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"text/csv","");

  // CSV header line
  String head = "t_s";
  for (uint8_t ch=0; ch<channels; ++ch) head += ",raw" + String(ch);
  if (wantMV)   for (uint8_t ch=0; ch<channels; ++ch) head += ",mV" + String(ch);
  if (wantFULL) for (uint8_t ch=0; ch<channels; ++ch) head += ",mA" + String(ch) + ",mm" + String(ch);
  head += "\n";
  server.sendContent(head);

  float lsb[NUM_SENSORS];
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) lsb[ch] = adsLSB_mV(codeToGain(gainCode[ch]));

  struct __attribute__((packed)) FrameV1 { uint32_t t_10us; int16_t raw0, raw1; };
  size_t frameSize = (ver <= 1) ? sizeof(FrameV1) : sizeof(MeasFrame);

   // stream frames using global scratch buffers (avoids large stack use)
  size_t csvFill = 0;
  WiFiClient client = server.client();
  bool aborted = false;
  uint32_t frameCounter = 0;

  auto flushCsvBuffer = [&]() -> bool {
    if (csvFill == 0) return true;
    server.sendContent(g_csvBuf, csvFill);
    if (!client.connected()) {
      return false;
    }
    csvFill = 0;
    yield();
    return true;
  };

  while (!aborted) {
    if (!client.connected()) { aborted = true; break; }
    int n = f.read(g_csvFrameBuf, frameSize * CSV_EXPORT_FRAME_CHUNK);
    if (n <= 0) break;
    int frames = n / (int)frameSize;
    for (int i=0;i<frames;i++){
      if (!client.connected()) { aborted = true; break; }
      const uint8_t* basePtr = g_csvFrameBuf + (i * frameSize);
      uint32_t t10 = 0; int16_t rawVals[NUM_SENSORS] = {0};
      if (ver <= 1) {
        const FrameV1 *fr = reinterpret_cast<const FrameV1*>(basePtr);
        t10 = fr->t_10us; rawVals[0]=fr->raw0; rawVals[1]=fr->raw1;
      } else {
        const MeasFrame *fr = reinterpret_cast<const MeasFrame*>(basePtr);
        t10 = fr->t_10us; for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) rawVals[ch] = fr->raw[ch];
      }
      float t = (t10 * (base.time_scale_us / 1e6f)); // seconds

      size_t len = 0;
      int wrote = snprintf(g_csvRowBuf, sizeof(g_csvRowBuf), "%.6f", (double)t);
      if (wrote < 0) wrote = 0; len = (size_t)wrote; if (len >= sizeof(g_csvRowBuf)) len = sizeof(g_csvRowBuf)-1;

      for (uint8_t ch=0; ch<channels && len < sizeof(g_csvRowBuf)-1; ++ch){
        wrote = snprintf(g_csvRowBuf + len, sizeof(g_csvRowBuf) - len, ",%d", (int)rawVals[ch]);
        if (wrote < 0) wrote = 0; len += (size_t)wrote; if (len >= sizeof(g_csvRowBuf)) len = sizeof(g_csvRowBuf)-1;
      }

      float mv[NUM_SENSORS];
      for (uint8_t ch=0; ch<channels; ++ch) mv[ch] = rawVals[ch] * lsb[ch];

      if (wantMV && len < sizeof(g_csvRowBuf) - 1){
        for (uint8_t ch=0; ch<channels && len < sizeof(g_csvRowBuf)-1; ++ch){
          wrote = snprintf(g_csvRowBuf + len, sizeof(g_csvRowBuf) - len, ",%.3f", (double)mv[ch]);
          if (wrote < 0) wrote = 0; len += (size_t)wrote; if (len >= sizeof(g_csvRowBuf)) len = sizeof(g_csvRowBuf)-1;
        }
      }
      if (wantFULL && len < sizeof(g_csvRowBuf) - 1){
        for (uint8_t ch=0; ch<channels && len < sizeof(g_csvRowBuf)-1; ++ch){
          float ma = (sh[ch]>0.1f)? (mv[ch]/sh[ch]) : 0.0f;
          float pct = ((ma-4.0f)/16.0f)*100.0f; if(pct<0)pct=0; if(pct>100)pct=100;
          float mm = (pct/100.0f)*fs[ch] + off[ch];
          wrote = snprintf(g_csvRowBuf + len, sizeof(g_csvRowBuf) - len, ",%.3f,%.2f", (double)ma, (double)mm);
          if (wrote < 0) wrote = 0; len += (size_t)wrote; if (len >= sizeof(g_csvRowBuf)) len = sizeof(g_csvRowBuf)-1;
        }
      }
      if (len > sizeof(g_csvRowBuf) - 2) { len = sizeof(g_csvRowBuf) - 2; }
      g_csvRowBuf[len++] = '\n';

      if (len > sizeof(g_csvBuf)) {
        server.sendContent(g_csvRowBuf, len);
        if (!client.connected()) { aborted = true; break; }
        yield();
      } else {
        while (!aborted && (csvFill + len > sizeof(g_csvBuf))) {
          if (!flushCsvBuffer()) { aborted = true; break; }
        }

        if (aborted) break;
        memcpy(g_csvBuf + csvFill, g_csvRowBuf, len);
        csvFill += len;
      }

      frameCounter++;
      if ((frameCounter & 0x3F) == 0) yield();
    }
    if (aborted) break;
    yield();
  }
  if (!aborted && csvFill > 0) {
    if (!flushCsvBuffer()) {
      aborted = true;
    }
  }
  if (!aborted) {
    server.sendContent(""); // terminate chunked response
  }
  if (aborted) {
    csvFill = 0;
  }
  f.close();
  client.stop();
}


void handleLogStream() {
  // Params
  String nm = baseName(urlDecode(server.arg("file")));
  if (nm.length() == 0) nm = "log0.log";
  int lines = server.hasArg("lines") ? server.arg("lines").toInt() : 200;
  if (lines < 10) lines = 10;
  if (lines > 1000) lines = 1000;

  // SSE headers (chunked) — browser reconnects automatically
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection",    "keep-alive");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/event-stream", "");
  server.sendContent("retry: 4000\n\n");    // suggest 4s reconnect delay

  // Read last N lines and push as a single SSE event, then return immediately
  digitalWrite(WIZ_CS, HIGH);               // free SPI from W5500

  String path = "/logs/" + nm;
  if (!SD.exists(path)) {
    if (SD.exists("/logs/log0.log")) path = "/logs/log0.log";
    else { server.sendContent("data: (no log)\n\n"); return; }
  }

  File f = SD.open(path, FILE_READ);
  if (!f) { server.sendContent("data: (open failed)\n\n"); return; }

  String tail;
  if (!readLastLines(f, lines, tail)) {
    f.close(); server.sendContent("data: (read failed)\n\n"); return;
  }
  f.close();

  // One event; EventSource will show it and reconnect later
  server.sendContent(sseEncode(tail));
  // handler returns now — no long blocking loop
}

void handleLogsList() {
  server.sendHeader("Cache-Control","no-store");
  digitalWrite(WIZ_CS, HIGH);                 // ⬅️ free the SPI bus

  if (!sdMounted) { server.send(200,"application/json","{\"items\":[]}"); return; }
  if (!SD.exists("/logs")) SD.mkdir("/logs");

  File dir = SD.open("/logs");                // some cores need "/logs/" — we try plain "/logs"
  if (!dir) { server.send(200,"application/json","{\"items\":[]}"); return; }

  String out = "{\"items\":[";
  bool first = true;

  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    String nm = baseName(String(f.name()));
    uint32_t sz = (uint32_t)f.size();
    if (!first) out += ",";
    first = false;
    out += "{\"name\":\"" + nm + "\",\"size\":" + String(sz) + "}";
    f.close();                                // always close file entries
  }
  dir.close();

  out += "]}";
  server.send(200,"application/json", out);
}

void handleLogDownload() {
  server.sendHeader("Cache-Control","no-store");
  digitalWrite(WIZ_CS, HIGH);                 // ⬅️ free the SPI bus

  String nm = baseName(urlDecode(server.arg("file")));
  String path = "/logs/" + nm;

  // Fallback: if a stale link asked for something odd, serve the first log
  if (!SD.exists(path) && SD.exists("/logs/log0.log")) path = "/logs/log0.log";

  File f = SD.open(path, FILE_READ);
  if (!f) { server.send(404,"text/plain","Not found"); return; }
  server.sendHeader("Content-Disposition","attachment; filename=\"" + baseName(path) + "\"");
  server.streamFile(f, "text/plain");
  f.close();
}
void handleLogClear() {
  server.sendHeader("Cache-Control","no-store");
  digitalWrite(WIZ_CS, HIGH);                 // ⬅️ free the SPI bus

  if (!sdMounted) { server.send(200,"text/plain","ok"); return; }
  if (!SD.exists("/logs")) { server.send(200,"text/plain","ok"); return; }

  File dir = SD.open("/logs");
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    String nm = baseName(String(f.name()));
    f.close();
    SD.remove(String("/logs/") + nm);
  }
  dir.close();

  // Optional: immediately recreate an empty log so the UI never looks blank after clear
  File touch = SD.open("/logs/log0.log", FILE_WRITE);
  if (touch) touch.close();

  server.send(200,"text/plain","ok");
}

void handleLogTail() {
  server.sendHeader("Cache-Control", "no-store");
  digitalWrite(WIZ_CS, HIGH);  // free SPI from W5500

  if (!sdMounted) { server.send(503, "text/plain", "SD not mounted"); return; }

  String nm = baseName(urlDecode(server.arg("file")));
  if (nm.length() == 0) nm = "log0.log";
  int lines = server.hasArg("lines") ? server.arg("lines").toInt() : 200;
  if (lines < 10) lines = 10;
  if (lines > 1000) lines = 1000;

  String path = "/logs/" + nm;
  if (!SD.exists(path)) {
    // Fallback: try log0.log so viewer never looks dead
    path = "/logs/log0.log";
    if (!SD.exists(path)) { server.send(404, "text/plain", "No log file"); return; }
  }

  File f = SD.open(path, FILE_READ);
  if (!f) { server.send(404, "text/plain", "Open failed"); return; }

  String out;
  if (!readLastLines(f, lines, out)) {
    f.close();
    server.send(500, "text/plain", "Read failed");
    return;
  }
  f.close();

  server.send(200, "text/plain", out);  // plain text for easy display
}

void handleNotFound() {
  Serial.printf("[HTTP] 404 %s %s\n",
    server.method()==HTTP_GET?"GET":
    server.method()==HTTP_POST?"POST":
    server.method()==HTTP_OPTIONS?"OPTIONS":
    server.method()==HTTP_HEAD?"HEAD":"OTHER",
    server.uri().c_str());
  // nicer UX than a blank 404:
  handleRoot();
}

// ---- ROUTES: OTA ----
void handleOTAUpload() {
  // Optional expected MD5 in query
  bool hasMd5Arg = server.hasArg("md5");
  String expected = hasMd5Arg ? urlDecode(server.arg("md5")) : String();
  bool expectedProvided = false;
  bool expectedInvalid = false;
  if (hasMd5Arg) {
    if (expected.length() == 0) {
      // treat empty strings as "no MD5"
      expectedProvided = false;
    } else if (normalizeMd5Hex(expected)) {
      expectedProvided = true;
    } else {
      expectedInvalid = true;
    }
  }
  HTTPUpload &up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    otaInProgress = true;
    logLine("[OTA] Start: " + up.filename);
    if (expectedInvalid) {
      logLine(String("[OTA] Rejecting invalid MD5 parameter: ") + expected);
      server.send(400,"application/json","{\"ok\":false,\"err\":\"md5-format\",\"hint\":\"md5 must be 32 hex chars\"}");
      otaInProgress = false;
      return;
    }
    if (expectedProvided) {
      // If known ahead, we set it; Update will verify at end.
      Update.setMD5(expected.c_str());
    } else if (hasMd5Arg) {
      logLine("[OTA] No MD5 provided; proceeding without pre-check");
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // choose next OTA slot
      logLine("[OTA] Begin failed: " + String(Update.errorString()));
      server.send(200,"application/json","{\"ok\":false,\"err\":\"begin\"}");
      otaInProgress = false; return;
    }
    mbedtls_md5_init(&md5ctx); mbedtls_md5_starts_ret(&md5ctx); md5ctxActive = true;

  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      logLine("[OTA] Write failed: " + String(Update.errorString()));
    }
    mbedtls_md5_update_ret(&md5ctx, up.buf, up.currentSize);

  } else if (up.status == UPLOAD_FILE_END) {
    unsigned char digest[16];
    int md5res = mbedtls_md5_finish_ret(&md5ctx, digest);
    if (md5ctxActive) { mbedtls_md5_free(&md5ctx); md5ctxActive = false; }
    if (md5res != 0) {
      logLine(String("[OTA] MD5 finalize failed: ") + String(md5res));
      Update.abort();
      server.send(200,"application/json","{\"ok\":false,\"err\":\"md5\"}");
      otaInProgress = false;
      return;
    }
    // If expected MD5 was set via Update.setMD5, Update.end(true) checks it.
    bool ok = Update.end(true);
    char md5hex[33]; // also compute self-MD5 for reporting
    for (size_t i = 0; i < sizeof(digest); ++i) {
      snprintf(md5hex + (i * 2), 3, "%02x", static_cast<unsigned int>(digest[i]));
    }
    md5hex[32] = '\0';
    if (!ok) {
      logLine("[OTA] End failed: " + String(Update.errorString()));
      server.send(200,"application/json", String("{\"ok\":false,\"err\":\"") + Update.errorString() + "\"}");
      otaInProgress = false; return;
    }
    logLine(String("[OTA] Success, MD5=") + md5hex);
    server.send(200,"application/json", String("{\"ok\":true,\"md5\":\"") + md5hex + "\"}");
    otaInProgress = false;

  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    if (md5ctxActive) { mbedtls_md5_free(&md5ctx); md5ctxActive = false; }
    logLine("[OTA] Aborted");
    server.send(200,"application/json","{\"ok\":false,\"err\":\"aborted\"}");
    otaInProgress = false;
  }
}

void sendNoContent(){
  server.sendHeader("Cache-Control","no-store");
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.sendHeader("Access-Control-Allow-Methods","GET,POST,OPTIONS,HEAD");
  server.send(204);
}

// ---- AP & Server ----
void startApAndPortal() {
  uint8_t macWifi[6]; esp_read_mac(macWifi, ESP_MAC_WIFI_SOFTAP);
  char suf[5]; snprintf(suf,sizeof(suf),"%02X%02X", macWifi[4], macWifi[5]);
  apSsid = "ESP32-Config-" + String(suf);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid.c_str(), apPass);
  delay(100);
  Serial.print("[AP] SSID: "); Serial.println(apSsid);
  Serial.print("[AP] IP: ");   Serial.println(WiFi.softAPIP());

  dns.start(53, "*", WiFi.softAPIP());

  // --- mDNS on the AP interface (after AP exists) ---
  if (g_mdnsRunning) { MDNS.end(); g_mdnsRunning = false; }
  String host = makeHostname(cfg.devName);
  if (MDNS.begin(host.c_str())) {
    MDNS.addService("http", "tcp", 80);
    g_mdnsRunning = true;
    Serial.printf("[mDNS] %s.local on AP\n", host.c_str());
  } else {
    Serial.println("[mDNS] start failed (AP)");
  }
  // --------------------------------------------------
  server.on("/chat", HTTP_ANY, [](){ server.send(204); });
  server.on("/favicon.ico", HTTP_GET, [](){
    server.send(204); // No Content
  });
  server.on("/hotspot-detect.html", HTTP_GET, [](){ // Apple
    server.send(200, "text/plain", "OK");
  });
  server.on("/generate_204", HTTP_GET, [](){        // Android
    server.send(204);
  });
  server.on("/ncsi.txt", HTTP_GET, [](){            // Windows
    server.send(200, "text/plain", "Microsoft NCSI");
  });

  // Config + status
  server.on("/",       HTTP_GET, handleRoot);
  server.on("/save",   HTTP_POST, handleSave);
  server.on("/reboot", HTTP_GET,  handleReboot);
  server.on("/status", HTTP_GET,  handleStatus);

  // SD
  server.on("/fs",     HTTP_GET,  handleFsList);
  server.on("/dl",     HTTP_GET,  handleDownload);
  server.on("/rm",     HTTP_POST, handleDelete);
  server.on("/mkdir",  HTTP_POST, handleMkdir);
  server.on("/upload", HTTP_POST, [](){}, handleUploadPost);

  // Logs
  server.on("/logs",     HTTP_GET,  handleLogsList);
  server.on("/logdl",    HTTP_GET,  handleLogDownload);
  server.on("/logclear", HTTP_POST, handleLogClear);

  // OTA
  server.on("/ota", HTTP_POST, [](){}, handleOTAUpload);

  server.on("/logtail", HTTP_GET, handleLogTail);

  server.on("/logstream", HTTP_GET, handleLogStream);

  server.on("/ads",     HTTP_GET,  handleAdsGet);
  server.on("/adsconf", HTTP_POST, handleAdsConf);
  server.on("/adsregs", HTTP_GET, handleAdsRegs);

  // register:
  server.on("/adsdump", HTTP_GET, handleAdsDump);

  server.on("/cloudtest", HTTP_ANY, handleCloudTest);
  server.on("/clouddiag", HTTP_GET, handleCloudDiag);
  server.on("/measure/start", HTTP_POST, handleMeasStart);
  server.on("/measure/stop",  HTTP_POST, handleMeasStop);
  server.on("/measure/status",HTTP_GET,  handleMeasStatus);
  server.on("/measure/debug", HTTP_GET, handleMeasDebug);

  server.on("/export_csv", HTTP_GET, handleExportCsv);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("[WEB] Portal ready");
}

// ---- SETUP/LOOP ----
void setup() {
  Serial.begin(115200);
  g_adsMutex = xSemaphoreCreateMutex();
  delay(250);
  Serial.println("\n=== Boot ===");

  seedRNG_noADC();

  statusLedsInit();
  ledSelfTest();

  deselectAll();
  loadCfg();

  // Ethernet + SD
  bool ethOk = ethernetInit();
  lastLink = Ethernet.linkStatus();
  sdMounted = sdInit();
  logLine(String("[SUM] ETH=") + (ethOk?"OK":"FAIL") + " LINK=" + (lastLink==LinkON?"UP":"DOWN") + " SD=" + (sdMounted?"OK":"FAIL"));

  // NTP time
  // set TZ even before sync, so localtime uses Budapest after sync
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();

  refreshInternetState(true, 1000);
  if (Ethernet.linkStatus() == LinkON && g_internetOk) {
    if (ntpSyncW5500("pool.ntp.org") || ntpSyncW5500("time.google.com")) {
      logLine("[TIME] NTP sync OK: " + isoNow());
    } else {
      logLine("[TIME] NTP sync FAILED; DNS=" + Ethernet.dnsServerIP().toString());
    }
  }
  setupTime();

  adsPrefsReady = adsPrefs.begin(ADS_PREF_NS, false);  // 1) open NVS
  adsConfigLoad();                     // 2) pull config from NVS into globals
  adsInit();                           // 3) init ADS (ads.begin(...) sets adsReady)
  adsApplyHW();                        // 4) program gain/rate to chip from loaded globals

  // AP + web
  startApAndPortal();

  // LEDs
  updateStatusLeds();
}

void loop() {
  dns.processNextRequest();
  server.handleClient();

  // Link watchdog every ~1s
  static uint32_t t=0;
  if (millis()-t>1000) {
    t = millis();
    linkWatchdog();
  }
  refreshInternetState();
  adsWatchdog();
  alarmsTask();
  static uint32_t lastNtp=0;
  if (millis()-lastNtp > 3600000UL) {
    lastNtp = millis();
    if (Ethernet.linkStatus()==LinkON && g_internetOk) ntpSyncW5500("pool.ntp.org");
  }
    // ---- Periodic cloud push ----
  if (cfg.cloudEnabled) {
    uint32_t now = millis();
    uint32_t due = g_lastPushMs + (cfg.cloudPeriodS * 1000UL);
    if (now - g_lastPushMs > cfg.cloudPeriodS * 1000UL) {
      if (Ethernet.linkStatus()==LinkON && g_internetOk) {
        pushCloudNow(true);     // includes readings
        g_lastPushMs = now;
      }
    }
    // next-in seconds for /status
    if (now <= due) g_nextPushInS = (due - now) / 1000UL;
    else g_nextPushInS = 0;
  } else {
    g_nextPushInS = 0;
  }

  // Measurement sampling is now handled entirely by meas_task_bin().

  updateStatusLeds();
}