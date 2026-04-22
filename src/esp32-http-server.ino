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
#include "mbedtls/md.h"
#include <EthernetUdp.h>
#include <Dns.h>
#include <sys/time.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SSLClient.h>
#include <trust_anchors.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#if defined(ESP32)
#include "esp_timer.h"
#include "esp_heap_caps.h"
#endif

#include "index_html.h"   // UI page
#include "alarm_types.h"
#include "measurement_safety_logic.h"
#include "meas_autocycle_logic.h"
#include "storage_fault_logic.h"
#include "upload_retry_state.h"

#if defined(ESP32)
SET_LOOP_TASK_STACK_SIZE(16 * 1024);
#endif

#define ADS_PREF_NS "ads"   // make sure you use this same namespace everywhere

// Forward declarations so auto-generated prototypes can reference the type.
struct MeasFrame;
enum LedMode : uint8_t;
struct LedState;
struct RemoteCommand {
  String id;
  String action;
  String params;
  String issuedAt;
  String nonce;
  String sig;
};
static inline bool isEpochSane(time_t now);
static bool startMeasurementCore(int rateOverride, String& outErr);
static bool stopMeasurementCore(bool doUpload, bool persistAutoCyclePending,
                                bool& outUploaded, String& outFile, String& outErr);
static void scheduleDeferredReboot(const String& reason, uint32_t delayMs = 1500);
static void noteNetOp(const String& op);
static void noteNetResult(bool ok, const String& err = "");
static void noteRuntimeStage(const char* stage);
static void runtimeHealthTick();
static void runtimeCpuSampleTick();
static bool storageCanUseSd();
static bool storageIsFaulted();
static bool storageIsDegraded();
static const char* storageStateName();
static String storageLogSource();
static bool spiBusLock(TickType_t waitTicks = portMAX_DELAY);
static void spiBusUnlock();
static inline void prepareSdSpi();
static inline void prepareEthernetSpi();
static void storageRecordSuccess();
static void storageRecordFailure(const String& path, const String& err, uint8_t severity = 1u);
static void handlePersistentStorageFault(const String& path, const String& err);
static void storageClearHealthy();
static bool runSdSelfTest(String& outErr, String& outPath);
static bool measurementNetworkDeferralActive();
static uint32_t measurementDeferredIntervalMs(uint32_t baseMs);
static void measurementNetworkPolicyTick();
static void serviceLocalPortal();
static bool localPortalInteractiveActive();
static size_t measurementBufferedFrameCount();
static bool measurementResetBuffers();
static bool measurementEnsureActiveBufferWritable();
static bool measurementAppendFrame(const MeasFrame& fr);
static bool measurementFlushPendingBufferIfAny();
static bool measurementFlushAllBuffersBlocking();
String isoNow();
static void logLine(const String& msg);
void saveCfg();

struct ScopedSpiBusLock {
  bool locked;
  explicit ScopedSpiBusLock(TickType_t waitTicks = portMAX_DELAY)
      : locked(spiBusLock(waitTicks)) {}
  ~ScopedSpiBusLock() {
    if (locked) spiBusUnlock();
  }
  explicit operator bool() const { return locked; }
};

static const size_t NUM_SENSORS = 4;

namespace measurement_safety = logic::measurement_safety;
namespace meas_autocycle = logic::meas_autocycle;
namespace storage_fault = logic::storage_fault;
namespace upload_retry = logic::upload_retry;

static SemaphoreHandle_t g_adsMutex = nullptr;
static SemaphoreHandle_t g_spiMutex = nullptr;

static volatile float g_lastMv[NUM_SENSORS]  = {0,0,0,0};
static volatile float g_lastmA[NUM_SENSORS]  = {0,0,0,0};
static volatile float g_lastPct[NUM_SENSORS] = {0,0,0,0};

// Debounce for alarms (min dwell before state change)
static uint32_t g_alarmLastChangeMs[NUM_SENSORS] = {0,0,0,0};
static const uint32_t ALARM_MIN_DWELL_MS = 200; // ms
static float    g_alarmFiltered[NUM_SENSORS]    = {0,0,0,0};
static bool     g_alarmFilterInit[NUM_SENSORS]  = {false,false,false,false};
static AlarmState g_alarmPendingState[NUM_SENSORS] = { ALARM_NORMAL, ALARM_NORMAL, ALARM_NORMAL, ALARM_NORMAL };
static uint32_t g_alarmPendingSinceMs[NUM_SENSORS] = {0,0,0,0};
static const float   ALARM_FILTER_ALPHA = 0.18f;  // EMA weight for alarm smoothing

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
    case 128: case 250: case 490: case 920: case 1600: case 2400: case 3300:
      return true;
    default:
      return false;
  }
}

static constexpr int ADS_DEFAULT_SPS = 920;

Adafruit_ADS1015 ads;
Preferences adsPrefs;            // NVS namespace handle for ADS config
static bool adsPrefsReady = false;
static bool adsReady = false;

#if !(defined(RATE_ADS1015_128SPS) && defined(RATE_ADS1015_250SPS) && \
      defined(RATE_ADS1015_490SPS) && defined(RATE_ADS1015_920SPS) && \
      defined(RATE_ADS1015_1600SPS) && defined(RATE_ADS1015_2400SPS) && \
      defined(RATE_ADS1015_3300SPS))
  #error "Adafruit ADS1015 data-rate macros missing; update the ADS1X15 library"
#endif

// --- Data-rate helper: requires ADS1015 rate enums ---
// Call ads.setDataRate(...) using ADS1015-specific rate macros. This will
// only skip work when the ADS chip has been marked unavailable.
static void adsSetRateSps(Adafruit_ADS1015& ads, int sps) {
  if (!adsReady) return;

  const int clampedSps = validSps(sps) ? sps : ADS_DEFAULT_SPS;

  switch (clampedSps) {
    case 128:  ads.setDataRate(RATE_ADS1015_128SPS);  break;
    case 250:  ads.setDataRate(RATE_ADS1015_250SPS);  break;
    case 490:  ads.setDataRate(RATE_ADS1015_490SPS);  break;
    case 920:  ads.setDataRate(RATE_ADS1015_920SPS);  break;
    case 1600: ads.setDataRate(RATE_ADS1015_1600SPS); break;
    case 2400: ads.setDataRate(RATE_ADS1015_2400SPS); break;
    case 3300: ads.setDataRate(RATE_ADS1015_3300SPS); break;
  }
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

// --- Per-channel configuration (NEW) ---
static adsGain_t g_gainCh[NUM_SENSORS]  = { GAIN_ONE, GAIN_ONE, GAIN_ONE, GAIN_ONE }; // ±4.096 V
static int       g_rateCh[NUM_SENSORS]  = { 920, 920, 920, 920 };                     // SPS
static float     g_shuntCh[NUM_SENSORS] = { 160.0f, 160.0f, 160.0f, 160.0f };         // Ω
static bool      g_adsActive[NUM_SENSORS] = { true, true, true, true };               // enable/disable per-channel

// --- Legacy single-channel shadows (mirror A0 so older code compiles) ---
static adsGain_t g_adsGain    = GAIN_ONE;
static int       g_adsRateSps = 920;
static float     g_shuntOhms  = 160.0f;

// Per-channel engineering units config for mm
static float g_engFSmm[NUM_SENSORS]  = {40.0f, 40.0f, 40.0f, 40.0f};  // full-scale in mm — selectable 40/80
static float g_engOffmm[NUM_SENSORS] = {0.0f,  0.0f,  0.0f,  0.0f };  // offset added after scaling (mm)

// I2C pins
static const int I2C_SDA = 21, I2C_SCL = 22;

// Single definition only (remove any duplicates elsewhere!)
static inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

// LSB (mV per code) for ADS1015 by gain
static float adsLSB_mV(adsGain_t g){
  switch(g){
    case GAIN_TWOTHIRDS: return 3.0000f;
    case GAIN_ONE:       return 2.0000f;
    case GAIN_TWO:       return 1.0000f;
    case GAIN_FOUR:      return 0.5000f;
    case GAIN_EIGHT:     return 0.2500f;
    case GAIN_SIXTEEN:   return 0.1250f;
  }
  return 2.0000f;
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
static uint32_t g_lastTimeSyncMs = 0;

static bool     g_linkOk           = false;
static bool     g_dnsOk            = false;
static bool     g_internetOk       = false;
static bool     g_cloudOk          = false;
static uint32_t g_lastInetCheckMs  = 0;
static const uint32_t INTERNET_CHECK_INTERVAL_MS = 5000;
static const uint32_t INTERNET_CHECK_OFFLINE_MAX_MS = 60000UL;
static const uint16_t INTERNET_CHECK_TIMEOUT_MS  = 750;
static uint32_t g_internetCheckBackoffMs = INTERNET_CHECK_INTERVAL_MS;
static uint32_t g_lastAuthFailMs   = 0;
static const uint32_t AUTH_COOLDOWN_MS = 500;
static uint32_t g_lastNtpAttemptMs = 0;
static uint32_t g_ntpRetryMs       = 30000;
static bool     g_forceNtpSync     = false;
static uint32_t g_lastDhcpMaintainMs = 0;
static bool     g_measurementNetDeferredPrev = false;

String apSsid;

byte ethMac[6] = { 0,0,0,0,0,0 };
IPAddress testHost(1,1,1,1);
const uint16_t testPort = 53;

struct AppCfg {
  String devName   = "sensor-prototype";
  String serverUrl = "https://example.com/ingest";
  String apiKey    = "";
  String deviceId  = "";
  String cmdSecret = "";
  String localAuthToken = "";
  String apPass    = "";
  bool commissioningMode = true;
  bool remoteEnabled = false;
  String lastCommandId = "";
  bool useStatic   = false;
  IPAddress ip     = IPAddress(0,0,0,0);
  IPAddress gw     = IPAddress(0,0,0,0);
  IPAddress mask   = IPAddress(255,255,255,0);
  IPAddress dns    = IPAddress(1,1,1,1);
  bool cloudEnabled = false;
  uint32_t cloudPeriodS = 30;
  bool uploadOnStop = true;
  String tlsFp = "";
} cfg;

bool sdMounted = false;
static bool g_sdBootMounted = false;
bool ethUpOnce = false;
EthernetLinkStatus lastLink = Unknown;

static storage_fault::Status g_storageStatus = {};
static constexpr const char* STORAGE_RECOVERY_PREF_KEY = "sdRecover";
static constexpr size_t RAM_LOG_LINE_COUNT = 128;
static constexpr size_t RAM_LOG_LINE_BYTES = 224;
static char g_ramLogLines[RAM_LOG_LINE_COUNT][RAM_LOG_LINE_BYTES] = {};
static uint16_t g_ramLogNext = 0;
static uint16_t g_ramLogCount = 0;
static portMUX_TYPE g_ramLogMux = portMUX_INITIALIZER_UNLOCKED;

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
static uint32_t g_lastPortalHttpActivityMs = 0;
static bool g_portalMeasurementLockHold = false;

// ----- Cloud push state -----
static uint32_t g_lastPushMs   = 0;
static int      g_lastHttpCode = -1;
static String   g_lastCloudErr = "";
static String   g_lastPushIso  = "";
static uint32_t g_lastCloudSkipLogMs = 0;
// For periodic "next in X s" status
static uint32_t g_nextPushInS  = 0;
static uint32_t g_lastCloudOkMs = 0;

// ----- Remote command state -----
static const uint32_t REMOTE_POLL_INTERVAL_MS = 2000;
static const uint32_t REMOTE_POLL_PORTAL_INTERVAL_MS = 30000;
static const uint32_t CLOUD_PUSH_PORTAL_INTERVAL_MS = 120000;
static const uint32_t LOCAL_PORTAL_ACTIVITY_GRACE_MS = 15000UL;
static const uint32_t REMOTE_STARTSTOP_COOLDOWN_MS = 1500;
static const uint32_t REMOTE_REBOOT_COOLDOWN_MS = 60000;
static const uint32_t REMOTE_POLL_MAX_BACKOFF_MS = 30000;
static const uint32_t MEASUREMENT_NET_DEFER_INTERVAL_MS =
  measurement_safety::kDeferredNetworkIntervalMs;
static uint32_t g_lastRemotePollMs = 0;
static uint32_t g_lastRemotePollOkMs = 0;
static String   g_lastRemoteCmdId = "";
static String   g_lastRemoteCmdResult = "";
static uint8_t  g_remotePollFailStreak = 0;
static uint32_t g_remotePollIntervalMs = REMOTE_POLL_INTERVAL_MS;
static bool     g_pendingRemoteReboot = false;
static uint32_t g_pendingRemoteRebootAtMs = 0;
static String   g_pendingRebootReason = "";
static uint32_t g_lastRemoteStartStopMs = 0;
static uint32_t g_lastRemoteRebootMs = 0;

// ----- Runtime diagnostics -----
static TaskHandle_t g_loopTaskHandle = nullptr;
static uint32_t g_loopLastMs = 0;
static uint32_t g_loopPrevMs = 0;
static uint32_t g_loopMaxBlockMs = 0;
static uint32_t g_lastLoopStallLogMs = 0;
static uint32_t g_lastHealthLogMs = 0;
static uint32_t g_lastBreadcrumbMs = 0;
static String   g_lastNetOp = "idle";
static String   g_lastNetErr = "";
static uint32_t g_lastNetErrMs = 0;
static char     g_lastRuntimeStage[24] = "boot";
static uint32_t g_lastRuntimeStageMs = 0;

struct RuntimeCpuSample {
  bool valid;
  uint32_t sampledAtMs;
  uint16_t taskCount;
  uint8_t cpuBusyPct;
  uint8_t loopCpuPct;
  uint8_t measCpuPct;
};

static RuntimeCpuSample g_runtimeCpu = {};

struct RuntimeBreadcrumb {
  uint32_t magic;
  uint32_t version;
  uint32_t uptimeMs;
  uint32_t freeHeap;
  uint32_t minFreeHeap;
  uint32_t largestFreeBlock;
  uint32_t loopMaxBlockMs;
  uint32_t flags;
  char lastNetOp[16];
  char lastStage[24];
  char lastNetErr[48];
  char measFile[80];
};

static constexpr uint32_t RUNTIME_BREADCRUMB_MAGIC = 0x52554E32u; // "RUN2"
static constexpr uint32_t RUNTIME_BREADCRUMB_VERSION = 2u;

#if defined(ESP32)
RTC_DATA_ATTR static RuntimeBreadcrumb g_rtcBreadcrumb = {};
#else
static RuntimeBreadcrumb g_rtcBreadcrumb = {};
#endif
static RuntimeBreadcrumb g_prevBreadcrumb = {};
static bool g_havePrevBreadcrumb = false;

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
static float    g_measSps[NUM_SENSORS]   = {920.0f, 920.0f, 920.0f, 920.0f};
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
static const uint32_t MEAS_AUTOCYCLE_MARGIN_TICKS = 10UL * 60UL * 100000UL; // 10 minutes
static const uint32_t MEAS_AUTOCYCLE_TARGET_LIMIT_TICKS = 4UL * 60UL * 60UL * 100000UL; // 4 hours
static const uint32_t MEAS_AUTOCYCLE_LIMIT_TICKS =
  meas_autocycle::effectiveLimitTicks(MEAS_AUTOCYCLE_TARGET_LIMIT_TICKS);
static const uint32_t MEAS_AUTOCYCLE_UPLOAD_RETRY_MS = 30000UL;
static const uint32_t MEAS_AUTOCYCLE_RESTART_RETRY_MS = 5000UL;
static bool     g_measAutoRestartPending = false;
static bool     g_measAutoRestartWaitingUpload = false;
static uint32_t g_measAutoRestartLastAttemptMs = 0;
static uint32_t g_measAutoRestartLastLogMs = 0;
static const char* const MEAS_PENDING_AUTOCYCLE_PATH = "/meas/.pending_autocycle.bin";
static const char* const MEAS_PENDING_AUTOCYCLE_TMP_PATH = "/meas/.pending_autocycle.tmp";

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
static const size_t BATCH_BUFFER_COUNT = 2;
static const int8_t BATCH_PENDING_NONE = -1;
static const int8_t BATCH_PENDING_FLUSHING = -2;
static MeasFrame*  g_batch[BATCH_BUFFER_COUNT] = {nullptr, nullptr};
static volatile size_t   g_batchCounts[BATCH_BUFFER_COUNT] = {0, 0};
static volatile uint8_t  g_batchActiveIndex = 0;
static volatile int8_t   g_batchPendingIndex = BATCH_PENDING_NONE;
static volatile size_t   g_batchPendingOffset = 0;
static portMUX_TYPE      g_batchMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t          g_batchWaitStartMs = 0;

static uint64_t g_startUs      = 0;             // monotonic microseconds at session start
static uint64_t g_nextDueUs    = 0;             // next pair due time
static uint32_t g_pairPeriodUs = 0;             // ~2200 us @ 920+920

// samples/bytes for status
static volatile uint32_t g_frameCount   = 0;             // number of MeasFrame written
static uint64_t g_measBytes    = 0;             // total bytes on disk

static size_t measurementBufferedFrameCount() {
  portENTER_CRITICAL(&g_batchMux);
  size_t total = g_batchCounts[0] + g_batchCounts[1];
  portEXIT_CRITICAL(&g_batchMux);
  return total;
}

static bool measurementEnsureBuffersAllocated() {
  for (size_t i = 0; i < BATCH_BUFFER_COUNT; ++i) {
    if (g_batch[i]) continue;
    size_t bytes = BATCH_FRAMES * sizeof(MeasFrame);
#if defined(ESP32)
    g_batch[i] = static_cast<MeasFrame*>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
#else
    g_batch[i] = static_cast<MeasFrame*>(malloc(bytes));
#endif
    if (!g_batch[i]) return false;
    memset(g_batch[i], 0, bytes);
  }
  return true;
}

static bool measurementResetBuffers() {
  if (!measurementEnsureBuffersAllocated()) return false;
  portENTER_CRITICAL(&g_batchMux);
  for (size_t i = 0; i < BATCH_BUFFER_COUNT; ++i) {
    memset(g_batch[i], 0, BATCH_FRAMES * sizeof(MeasFrame));
  }
  g_batchCounts[0] = 0;
  g_batchCounts[1] = 0;
  g_batchActiveIndex = 0;
  g_batchPendingIndex = BATCH_PENDING_NONE;
  g_batchPendingOffset = 0;
  portEXIT_CRITICAL(&g_batchMux);
  g_batchWaitStartMs = 0;
  return true;
}

static void measurementNoteBackpressure(bool waiting) {
  if (waiting) {
    if (g_batchWaitStartMs == 0) {
      g_batchWaitStartMs = millis();
      logLine(String("[MEAS] batch backpressure: waiting for flush active=")
              + String((unsigned)g_batchActiveIndex));
    }
    return;
  }

  if (g_batchWaitStartMs != 0) {
    uint32_t waitedMs = millis() - g_batchWaitStartMs;
    g_batchWaitStartMs = 0;
    logLine(String("[MEAS] batch backpressure cleared after ") + String(waitedMs) + " ms");
  }
}

static bool measurementEnsureActiveBufferWritable() {
  while (g_measActive) {
    bool ready = false;
    bool waiting = false;
    portENTER_CRITICAL(&g_batchMux);
    uint8_t activeIdx = g_batchActiveIndex;
    if (g_batchCounts[activeIdx] < BATCH_FRAMES) {
      ready = true;
    } else if (g_batchPendingIndex == BATCH_PENDING_NONE) {
      g_batchPendingIndex = static_cast<int8_t>(activeIdx);
      g_batchPendingOffset = 0;
      g_batchActiveIndex = static_cast<uint8_t>((activeIdx + 1U) % BATCH_BUFFER_COUNT);
      g_batchCounts[g_batchActiveIndex] = 0;
      ready = true;
    } else {
      waiting = true;
    }
    portEXIT_CRITICAL(&g_batchMux);

    if (ready) {
      measurementNoteBackpressure(false);
      return true;
    }

    measurementNoteBackpressure(waiting);
    delay(1);
    yield();
  }

  measurementNoteBackpressure(false);
  return false;
}

static bool measurementAppendFrame(const MeasFrame& fr) {
  if (!measurementEnsureActiveBufferWritable()) return false;

  bool needRotate = false;
  portENTER_CRITICAL(&g_batchMux);
  uint8_t activeIdx = g_batchActiveIndex;
  if (g_batchCounts[activeIdx] < BATCH_FRAMES) {
    g_batch[activeIdx][g_batchCounts[activeIdx]++] = fr;
    needRotate = (g_batchCounts[activeIdx] >= BATCH_FRAMES);
  }
  portEXIT_CRITICAL(&g_batchMux);

  if (needRotate) return measurementEnsureActiveBufferWritable();
  return true;
}

// ---------- Raw ADS1015 register access (fast) ----------
/* static inline void adsWriteReg(uint8_t reg, uint16_t val){
  Wire.beginTransmission(0x48); Wire.write(reg);
  Wire.write(uint8_t(val>>8)); Wire.write(uint8_t(val)); Wire.endTransmission();
} */

// ---------- Raw ADS1015 register access (fast + error-aware) ----------
static inline bool adsWriteReg(uint8_t reg, uint16_t val){
  Wire.beginTransmission(0x48);
  Wire.write(reg);
  Wire.write(uint8_t(val >> 8));
  Wire.write(uint8_t(val & 0xFF));
  int rc = Wire.endTransmission();    // 0 = OK
  if (rc != 0) {
    adsNoteError(String("i2c write 0x48 reg ") + reg, rc);
  }
  return rc == 0;
}

static inline bool adsReadRegRaw(uint8_t reg, uint16_t &out){
  out = 0;
  Wire.beginTransmission(0x48);
  Wire.write(reg);
  int rc = Wire.endTransmission();    // 0 = OK
  if (rc != 0) {
    adsNoteError(String("i2c write(ptr) 0x48 reg ") + reg, rc);
    return false;
  }

  if (Wire.requestFrom((int)0x48, 2) != 2) {
    adsNoteError(String("i2c short read 0x48 reg ") + reg, -1);
    return false;
  }

  out = (uint16_t(Wire.read()) << 8) | Wire.read();
  return true;
}

static inline uint16_t adsPgaBits(adsGain_t g){
  switch(g){ case GAIN_TWOTHIRDS:return 0<<9; case GAIN_ONE:return 1<<9; case GAIN_TWO:return 2<<9;
    case GAIN_FOUR:return 3<<9; case GAIN_EIGHT:return 4<<9; default:return 5<<9; }
}

// DR code helper (0–7) for the bits in the ADS config register
static inline uint8_t adsDrCode(int sps){
  switch(sps){
    case 128:  return 0;  // ADS1015: 128 SPS
    case 250:  return 1;  // ADS1015: 250 SPS
    case 490:  return 2;  // ADS1015: 490 SPS
    case 920:  return 3;  // ADS1015: 920 SPS
    case 1600: return 4;  // ADS1015: 1600 SPS
    case 2400: return 5;  // ADS1015: 2400 SPS
    case 3300: return 6;  // ADS1015: 3300 SPS
    default:   return 7;  // ADS1015: 3300 SPS
  }
}

static inline uint16_t adsDrBits(uint8_t drCode){
  return (uint16_t)(drCode & 0x07u) << 5;
}

// Conversion time (µs) by DR code for ADS1015 timings
static inline uint32_t adsConvTimeUsFromDr(uint8_t drCode){
  // ADS1015 nominal conversion times (µs), with light margin.
  static const uint32_t ADS1015_US[8] = {
    8000,  // 128 SPS   (~7.8 ms)
    4000,  // 250 SPS   (~4.0 ms)
    2000,  // 490 SPS   (~2.0 ms)
    1100,  // 920 SPS   (~1.09 ms)
    500,   // 1600 SPS  (~0.625 ms)
    350,   // 2400 SPS  (~0.417 ms)
    250,   // 3300 SPS  (~0.303 ms)
    250
  };
  return ADS1015_US[drCode & 0x07u];
}

// ADS1015 single-shot read: start -> wait ~t_conv -> OS check -> read.
// Pattern ported from your stable ADS1115 2-channel firmware.
static bool adsSingleReadRaw_timed(uint8_t ch, adsGain_t gain, int rateSps, int16_t &raw){
  ch &= 0x03; // 0..3
  noteRuntimeStage("ads_start");

  // Clamp SPS to a valid ADS1015 value
  const int sps    = validSps(rateSps) ? rateSps : ADS_DEFAULT_SPS;
  const uint8_t drCode  = adsDrCode(sps);
  const uint16_t drBits = adsDrBits(drCode);

  // MUX bits [14:12]: AINx vs GND → 100,101,110,111 for A0..A3
  const uint16_t mux = (uint16_t(4u + ch) << 12);

  // Build config: OS=1, MUX, PGA, MODE=single-shot, DR, comparator off
  uint16_t cfg = 0;
  cfg |= 1u << 15;          // OS = 1 (start conversion)
  cfg |= mux;               // MUX
  cfg |= adsPgaBits(gain);  // gain
  cfg |= 1u << 8;           // MODE=1 (single-shot)
  cfg |= drBits;            // data rate
  cfg |= 0x0003;            // comparator disabled

  // Conversion time from ADS1015 table + small safety margin.
  const uint32_t convUs = adsConvTimeUsFromDr(drCode);
  uint32_t waitUs = convUs + convUs / 10;   // +10%
  if (waitUs < 300u) waitUs = 300u;         // never <0.3 ms

  if (g_adsMutex) xSemaphoreTake(g_adsMutex, portMAX_DELAY);

  // Start conversion
  if (!adsWriteReg(0x01, cfg)) {
    if (g_adsMutex) xSemaphoreGive(g_adsMutex);
    g_adsLastErr   = "adc write cfg failed";
    g_adsLastErrMs = millis();
    return false;
  }

  // Busy-wait most of the conversion time (no I2C bus traffic)
  const uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < waitUs) {
    // spin
  }

  // Single OS-bit loop with a bounded timeout so ADS stalls cannot block forever.
  uint16_t c = 0;
  const uint32_t osWaitStartUs = micros();
  const uint32_t osWaitTimeoutUs = waitUs + 5000u;
  noteRuntimeStage("ads_wait_os");
  do {
    if (!adsReadRegRaw(0x01, c)) {
      if (g_adsMutex) xSemaphoreGive(g_adsMutex);
      g_adsLastErr   = "adc read cfg failed";
      g_adsLastErrMs = millis();
      return false;
    }
    if ((uint32_t)(micros() - osWaitStartUs) > osWaitTimeoutUs) {
      if (g_adsMutex) xSemaphoreGive(g_adsMutex);
      adsNoteError(String("adc os timeout ch ") + ch, -2);
      return false;
    }
  } while (!(c & 0x8000u));  // wait until OS=1 (ready)

  // Read conversion result
  uint16_t u = 0;
  noteRuntimeStage("ads_read");
  if (!adsReadRegRaw(0x00, u)) {
    if (g_adsMutex) xSemaphoreGive(g_adsMutex);
    g_adsLastErr   = "adc read conv failed";
    g_adsLastErrMs = millis();
    return false;
  }

  if (g_adsMutex) xSemaphoreGive(g_adsMutex);

  // ADS1015: 12-bit result left-justified in 16 bits -> shift right by 4
  raw = (int16_t)(u >> 4);
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

    // Use the same proven ADS path as the status/idle readings
    for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
      if (!g_adsActive[ch]) {
        raw[ch] = 0;
        mv[ch] = ma[ch] = pct[ch] = 0.0f;
        g_lastMv[ch]  = 0.0f;
        g_lastmA[ch]  = 0.0f;
        g_lastPct[ch] = 0.0f;
        continue;
      }

      bool ok = adsSingleReadRaw_timed(ch, g_gainCh[ch], g_rateCh[ch], raw[ch]);
      if (!ok) {
        raw[ch] = 0;
      }
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

    // Ensure inactive channels never hold stale alarm states from prior sessions
    // (especially when the active mask changes between runs).
    for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
      if (!g_adsActive[ch]) {
        g_alarmCh[ch] = ALARM_NORMAL;
        g_alarmLastChangeMs[ch] = nowMs;
        g_alarmPendingState[ch] = ALARM_NORMAL;
        g_alarmPendingSinceMs[ch] = 0;
        g_alarmFilterInit[ch] = false;
        g_alarmFiltered[ch] = 0.0f;
      }
    }

    auto evalSmoothedDebounced = [&](uint8_t ch, float sampleMa){
      if (!g_alarmFilterInit[ch]) {
        g_alarmFiltered[ch] = sampleMa;
        g_alarmFilterInit[ch] = true;
      } else {
        g_alarmFiltered[ch] += ALARM_FILTER_ALPHA * (sampleMa - g_alarmFiltered[ch]);
      }

      AlarmState desired = evalWithHyst(g_alarmCh[ch], g_alarmFiltered[ch]);
      if (desired == g_alarmCh[ch]) {
        g_alarmPendingSinceMs[ch] = 0;
        g_alarmPendingState[ch] = desired;
        return;
      }

      if (g_alarmPendingSinceMs[ch] == 0 || desired != g_alarmPendingState[ch]) {
        g_alarmPendingState[ch] = desired;
        g_alarmPendingSinceMs[ch] = nowMs;
      }

      if (nowMs - g_alarmPendingSinceMs[ch] >= ALARM_MIN_DWELL_MS) {
        g_alarmLastChangeMs[ch] = nowMs;
        g_alarmCh[ch] = desired;
        g_alarmPendingSinceMs[ch] = 0;
        logLine(String("[ALARM] A") + ch + " " + alarmStr(desired) + " @ " + String(g_alarmFiltered[ch],3) + " mA");
        updateAlarmGPIO();
      }
    };

    for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
      if (!g_adsActive[ch]) continue;
      evalSmoothedDebounced(ch, ma[ch]);
    }

    uint32_t total = g_frameCount + measurementBufferedFrameCount();
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
    if (!measurementAppendFrame(fr)) break;

    // (No delay; the conversions fully pace the loop)
  }

  // Session ending: the loop task / stop path drains any pending and partial buffers.

  // task exits
  g_measTask = nullptr;
  vTaskDelete(nullptr);
}

// --- Read ADS1015 config reg (0x01) and decode data rate ---
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
    case 0: return 128;
    case 1: return 250;
    case 2: return 490;
    case 3: return 920;
    case 4: return 1600;
    case 5: return 2400;
    case 6: return 3300;
    case 7: return 3300;
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
  if (!storageCanUseSd()) return;
  ScopedSpiBusLock lock;
  if (!lock) return;
  prepareSdSpi();
  if (!SD.exists("/meas") && !SD.mkdir("/meas")) {
    storageRecordFailure("/meas", "mkdir_fail");
    return;
  }
  if (g_measDir.length() && !SD.exists(g_measDir.c_str()) && !SD.mkdir(g_measDir.c_str())) {
    storageRecordFailure(g_measDir, "mkdir_fail");
    return;
  }
  storageRecordSuccess();
}

static String sessionIdFromDir(const String& dir) {
  String leaf = baseName(dir);
  if (leaf.startsWith("sess_")) return leaf.substring(5);
  return leaf;
}

static String sessionFilePathForDir(const String& sessionDir, uint32_t idx){
  char buf[32];
  snprintf(buf, sizeof(buf), "/part_%04lu.am1", (unsigned long)idx);
  String path = sessionDir;
  path += buf;
  return path;
}

static String sessionFilePath(uint32_t idx){
  return sessionFilePathForDir(g_measDir, idx);
}

static String uploadStatePathForSession(const String& sessionDir) {
  return sessionDir + "/.upload_state.bin";
}

static String uploadStateTmpPathForSession(const String& sessionDir) {
  return sessionDir + "/.upload_state.tmp";
}

static bool readBinaryFileExact(const String& path, uint8_t* data, size_t len) {
  if (!storageCanUseSd()) return false;
  ScopedSpiBusLock lock;
  if (!lock) return false;
  prepareSdSpi();
  if (!SD.exists(path)) return false;
  File f = SD.open(path, FILE_READ);
  if (!f) {
    storageRecordFailure(path, "open_fail");
    return false;
  }
  size_t got = f.read(data, len);
  f.close();
  if (got != len) {
    storageRecordFailure(path, "read_short");
    return false;
  }
  storageRecordSuccess();
  return got == len;
}

static bool writeBinaryFileAtomically(const String& finalPath, const String& tempPath,
                                      const uint8_t* data, size_t len) {
  if (!storageCanUseSd()) return false;
  ScopedSpiBusLock lock;
  if (!lock) return false;
  prepareSdSpi();
  if (SD.exists(tempPath)) SD.remove(tempPath);
  File f = SD.open(tempPath, FILE_WRITE);
  if (!f) {
    storageRecordFailure(tempPath, "open_fail");
    return false;
  }
  size_t written = f.write(data, len);
  f.close();
  if (written != len) {
    SD.remove(tempPath);
    storageRecordFailure(tempPath, "write_short");
    return false;
  }
  if (SD.exists(finalPath)) SD.remove(finalPath);
  if (!SD.rename(tempPath, finalPath)) {
    SD.remove(tempPath);
    storageRecordFailure(finalPath, "rename_fail");
    return false;
  }
  storageRecordSuccess();
  return true;
}

static bool saveUploadManifest(const String& sessionDir, const upload_retry::Manifest& manifest) {
  if (!storageCanUseSd() || sessionDir.length() == 0) return false;
  ensureMeasDir();
  return writeBinaryFileAtomically(
    uploadStatePathForSession(sessionDir),
    uploadStateTmpPathForSession(sessionDir),
    reinterpret_cast<const uint8_t*>(&manifest),
    sizeof(manifest)
  );
}

static bool loadUploadManifest(const String& sessionDir, upload_retry::Manifest& manifest) {
  if (!readBinaryFileExact(uploadStatePathForSession(sessionDir),
                           reinterpret_cast<uint8_t*>(&manifest),
                           sizeof(manifest))) {
    return false;
  }
  return upload_retry::validateManifest(manifest) &&
         strcmp(manifest.sessionDir, sessionDir.c_str()) == 0;
}

static void removeUploadManifest(const String& sessionDir) {
  if (!storageCanUseSd() || sessionDir.length() == 0) return;
  ScopedSpiBusLock lock;
  if (!lock) return;
  prepareSdSpi();
  String path = uploadStatePathForSession(sessionDir);
  String temp = uploadStateTmpPathForSession(sessionDir);
  if (SD.exists(path)) SD.remove(path);
  if (SD.exists(temp)) SD.remove(temp);
  storageRecordSuccess();
}

static bool loadOrInitUploadManifest(const String& sessionDir, uint32_t finalFileIndex,
                                     upload_retry::Manifest& manifest) {
  if (loadUploadManifest(sessionDir, manifest) &&
      upload_retry::sessionMatches(manifest, sessionDir.c_str(), finalFileIndex)) {
    return true;
  }
  upload_retry::initManifest(manifest, sessionDir.c_str(), finalFileIndex);
  return saveUploadManifest(sessionDir, manifest);
}

static bool savePendingAutoCycleState(const String& sessionDir, uint32_t finalFileIndex,
                                      bool waitingUpload, bool pendingRestart) {
  if (!storageCanUseSd() || sessionDir.length() == 0) return false;
  ensureMeasDir();
  upload_retry::PendingState state{};
  upload_retry::initPendingState(state, sessionDir.c_str(), finalFileIndex,
                                 waitingUpload, pendingRestart);
  return writeBinaryFileAtomically(
    String(MEAS_PENDING_AUTOCYCLE_PATH),
    String(MEAS_PENDING_AUTOCYCLE_TMP_PATH),
    reinterpret_cast<const uint8_t*>(&state),
    sizeof(state)
  );
}

static bool loadPendingAutoCycleState(upload_retry::PendingState& state) {
  if (!readBinaryFileExact(String(MEAS_PENDING_AUTOCYCLE_PATH),
                           reinterpret_cast<uint8_t*>(&state),
                           sizeof(state))) {
    return false;
  }
  return upload_retry::validatePendingState(state);
}

static void clearPendingAutoCycleStateFile() {
  if (!storageCanUseSd()) return;
  ScopedSpiBusLock lock;
  if (!lock) return;
  prepareSdSpi();
  if (SD.exists(MEAS_PENDING_AUTOCYCLE_PATH)) SD.remove(MEAS_PENDING_AUTOCYCLE_PATH);
  if (SD.exists(MEAS_PENDING_AUTOCYCLE_TMP_PATH)) SD.remove(MEAS_PENDING_AUTOCYCLE_TMP_PATH);
  storageRecordSuccess();
}

static bool hasPendingSessionUpload() {
  if (g_measDir.length() == 0 || g_measFileIndex == 0xFFFFFFFFu) return false;
  if (g_measAutoRestartPending || g_measAutoRestartWaitingUpload) return true;
  if (!storageCanUseSd()) return false;
  ScopedSpiBusLock lock;
  if (!lock) return false;
  prepareSdSpi();
  return SD.exists(uploadStatePathForSession(g_measDir)) || SD.exists(MEAS_PENDING_AUTOCYCLE_PATH);
}

static void restorePendingAutoCycleState() {
  if (!storageCanUseSd()) return;
  upload_retry::PendingState state{};
  if (!loadPendingAutoCycleState(state)) return;

  String sessionDir = String(state.sessionDir);
  bool sessionExists = false;
  {
    ScopedSpiBusLock lock;
    if (lock) {
      prepareSdSpi();
      sessionExists = SD.exists(sessionDir);
    }
  }
  if (!sessionExists) {
    logLine(String("[MEAS] auto cycle: dropping stale resume state dir=") + sessionDir);
    clearPendingAutoCycleStateFile();
    removeUploadManifest(sessionDir);
    return;
  }

  g_measId = sessionIdFromDir(sessionDir);
  g_measDir = sessionDir;
  g_measFileIndex = state.finalFileIndex;
  g_measFile = sessionFilePathForDir(sessionDir, state.finalFileIndex);
  const bool pendingRestart = upload_retry::pendingRestart(state);
  const bool waitingUpload = upload_retry::pendingWaitingUpload(state);
  g_measAutoRestartPending = pendingRestart;
  g_measAutoRestartWaitingUpload = pendingRestart ? waitingUpload : false;
  g_measAutoRestartLastAttemptMs = millis() - (g_measAutoRestartWaitingUpload
                                                 ? MEAS_AUTOCYCLE_UPLOAD_RETRY_MS
                                                 : MEAS_AUTOCYCLE_RESTART_RETRY_MS);
  g_measAutoRestartLastLogMs = 0;

  if (!pendingRestart && waitingUpload) {
    logLine(String("[MEAS] pending manual upload restored in standby dir=") + sessionDir);
    return;
  }

  logLine(String("[MEAS] auto cycle: restored pending resume dir=") + sessionDir
          + " waiting_upload=" + String(g_measAutoRestartWaitingUpload ? "true" : "false"));
}

static uint32_t frameToFileIndex(const MeasFrame& fr){
  uint32_t rawIdx = (MEAS_FILE_SPAN_TICKS == 0) ? 0u : (fr.t_10us / MEAS_FILE_SPAN_TICKS);
  return measurement_safety::clampFileIndexForAutoCycle(
    rawIdx,
    MEAS_AUTOCYCLE_LIMIT_TICKS,
    MEAS_FILE_SPAN_TICKS
  );
}

static bool writeBinHeader(const String& path){
  if (!storageCanUseSd() || path.length() == 0) return false;
  ensureMeasDir();
  ScopedSpiBusLock lock;
  if (!lock) return false;
  prepareSdSpi();
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    storageRecordFailure(path, "open_fail");
    return false;
  }

  MeasHeader h{};
  memcpy(h.magic, "AM01", 4);
  h.ver          = MEAS_HEADER_VER;
  time_t nowEpoch = time(nullptr);
  h.start_epoch  = isEpochSane(nowEpoch) ? (uint32_t)nowEpoch : 0u;
  h.time_scale_us= 10;                      // 10 µs ticks in frames
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    h.sps[ch]       = (uint16_t)g_measSps[ch];
    h.gain_code[ch] = gainToCode(g_gainCh[ch]);
    h.sh[ch]        = g_shuntCh[ch];
    h.fs[ch]        = g_engFSmm[ch];
    h.off[ch]       = g_engOffmm[ch];
  }

  size_t wrote = f.write((uint8_t*)&h, sizeof(h));
  f.close();
  if (wrote != sizeof(h)) {
    storageRecordFailure(path, "header_write_fail");
    return false;
  }
  storageRecordSuccess();
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

static size_t flushMeasurementFrames(const MeasFrame* frames, size_t count, size_t startPos = 0) {
  if (count == 0) return startPos;
  if (!storageCanUseSd()) return startPos;

  size_t pos = startPos;
  while (pos < count) {
    uint32_t idx = frameToFileIndex(frames[pos]);
    if (!switchToMeasFile(idx)) break;

    size_t end = pos + 1;
    while (end < count && frameToFileIndex(frames[end]) == idx) {
      ++end;
    }

    ScopedSpiBusLock lock;
    if (!lock) {
      storageRecordFailure(g_measFile, "spi_lock_fail");
      break;
    }
    prepareSdSpi();
    File f = SD.open(g_measFile, FILE_APPEND);
    if (!f) {
      storageRecordFailure(g_measFile, "open_fail");
      break;
    }

    size_t framesToWrite = end - pos;
    size_t bytes = framesToWrite * sizeof(MeasFrame);
    size_t wrote = f.write((const uint8_t*)&frames[pos], bytes);
    f.close();
    if (wrote != bytes) {
      storageRecordFailure(g_measFile, "frame_write_fail");
      break;
    }
    storageRecordSuccess();

    g_measBytes += bytes;
    g_frameCount += framesToWrite;
    pos = end;
  }

  return pos;
}

static bool measurementClaimPendingBuffer(uint8_t& bufIdx, size_t& count, size_t& offset) {
  bool claimed = false;
  portENTER_CRITICAL(&g_batchMux);
  if (g_batchPendingIndex >= 0) {
    bufIdx = static_cast<uint8_t>(g_batchPendingIndex);
    count = g_batchCounts[bufIdx];
    offset = g_batchPendingOffset;
    g_batchPendingIndex = BATCH_PENDING_FLUSHING;
    claimed = true;
  }
  portEXIT_CRITICAL(&g_batchMux);
  return claimed;
}

static void measurementRestorePendingBuffer(uint8_t bufIdx, size_t offset, bool completed) {
  portENTER_CRITICAL(&g_batchMux);
  if (completed || offset >= g_batchCounts[bufIdx]) {
    g_batchCounts[bufIdx] = 0;
    g_batchPendingOffset = 0;
    g_batchPendingIndex = BATCH_PENDING_NONE;
  } else {
    g_batchPendingOffset = offset;
    g_batchPendingIndex = static_cast<int8_t>(bufIdx);
  }
  portEXIT_CRITICAL(&g_batchMux);
}

static bool measurementFlushPendingBufferIfAny() {
  uint8_t bufIdx = 0;
  size_t count = 0;
  size_t offset = 0;
  if (!measurementClaimPendingBuffer(bufIdx, count, offset)) return true;

  if (offset >= count) {
    measurementRestorePendingBuffer(bufIdx, count, true);
    return true;
  }

  size_t nextOffset = flushMeasurementFrames(g_batch[bufIdx], count, offset);
  bool completed = (nextOffset >= count);
  measurementRestorePendingBuffer(bufIdx, nextOffset, completed);
  return completed || nextOffset > offset;
}

static bool measurementFlushAllBuffersBlocking() {
  bool ok = true;

  while (true) {
    bool hasPending = false;
    portENTER_CRITICAL(&g_batchMux);
    hasPending = (g_batchPendingIndex != BATCH_PENDING_NONE);
    portEXIT_CRITICAL(&g_batchMux);
    if (!hasPending) break;
    if (!measurementFlushPendingBufferIfAny()) {
      ok = false;
      break;
    }
  }

  uint8_t activeIdx = 0;
  size_t activeCount = 0;
  portENTER_CRITICAL(&g_batchMux);
  activeIdx = g_batchActiveIndex;
  activeCount = g_batchCounts[activeIdx];
  portEXIT_CRITICAL(&g_batchMux);

  if (activeCount == 0) return ok;

  size_t nextOffset = flushMeasurementFrames(g_batch[activeIdx], activeCount, 0);
  portENTER_CRITICAL(&g_batchMux);
  if (nextOffset >= activeCount) {
    g_batchCounts[activeIdx] = 0;
  } else if (nextOffset > 0) {
    size_t remaining = activeCount - nextOffset;
    memmove(g_batch[activeIdx], g_batch[activeIdx] + nextOffset, remaining * sizeof(MeasFrame));
    g_batchCounts[activeIdx] = remaining;
    ok = false;
  } else {
    ok = false;
  }
  portEXIT_CRITICAL(&g_batchMux);

  return ok;
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

static String randomHex(size_t bytesLen) {
  static const char kHex[] = "0123456789abcdef";
  String out;
  out.reserve(bytesLen * 2);
  for (size_t i = 0; i < bytesLen; ++i) {
    uint8_t b = uint8_t(esp_random() & 0xFFu);
    out += kHex[(b >> 4) & 0x0F];
    out += kHex[b & 0x0F];
  }
  return out;
}

static void deriveEthMacFromEfuse() {
  uint64_t efuse = ESP.getEfuseMac();
  ethMac[0] = 0x02; // locally administered, unicast
  ethMac[1] = (efuse >> 40) & 0xFF;
  ethMac[2] = (efuse >> 32) & 0xFF;
  ethMac[3] = (efuse >> 24) & 0xFF;
  ethMac[4] = (efuse >> 16) & 0xFF;
  ethMac[5] = (efuse >> 8)  & 0xFF;
}

static String defaultDeviceId() {
  char buf[24];
  uint64_t efuse = ESP.getEfuseMac();
  snprintf(buf, sizeof(buf), "esp32-%08lx", (unsigned long)(efuse & 0xFFFFFFFFu));
  return String(buf);
}

static String defaultApPassword() {
  uint64_t efuse = ESP.getEfuseMac();
  char suf[7];
  snprintf(suf, sizeof(suf), "%06lX", (unsigned long)(efuse & 0xFFFFFFu));
  return String("cfg-") + String(suf);
}

static void ensureSecurityDefaults() {
  bool dirty = false;
  if (cfg.deviceId.length() == 0) {
    cfg.deviceId = defaultDeviceId();
    dirty = true;
  }
  if (cfg.cmdSecret.length() < 32) {
    cfg.cmdSecret = randomHex(32);
    dirty = true;
  }
  if (cfg.localAuthToken.length() < 24) {
    cfg.localAuthToken = randomHex(24);
    dirty = true;
  }
  if (cfg.apPass.length() < 8) {
    cfg.apPass = defaultApPassword();
    dirty = true;
  }
  if (dirty) {
    saveCfg();
  }
}

static bool authMatches(const String& token) {
  if (cfg.localAuthToken.length() == 0) return false;
  String t = token;
  t.trim();
  return t.length() && t == cfg.localAuthToken;
}

static bool isAuthorizedRequest() {
  String token;

  String auth = server.header("Authorization");
  auth.trim();
  if (auth.startsWith("Bearer ")) token = auth.substring(7);
  if (!token.length() && auth.startsWith("bearer ")) token = auth.substring(7);
  if (!token.length() && server.hasArg("auth")) token = server.arg("auth");
  return authMatches(token);
}

static bool requireAuth() {
  if (isAuthorizedRequest()) return true;
  uint32_t now = millis();
  if (now - g_lastAuthFailMs > AUTH_COOLDOWN_MS) {
    g_lastAuthFailMs = now;
    logLine(String("[AUTH] denied ") + server.uri());
  }
  server.sendHeader("Cache-Control","no-store");
  server.send(401, "application/json", "{\"ok\":false,\"err\":\"unauthorized\"}");
  return false;
}

// Resolve hostname using the Ethernet DNS server. Falls back to cfg.dns or 1.1.1.1.
bool resolveHost(const char* host, IPAddress& out) {
  ScopedSpiBusLock lock;
  if (!lock) {
    g_dnsOk = false;
    return false;
  }
  prepareEthernetSpi();
  DNSClient dns;
  IPAddress dnsIP = ethDnsIpSafe();
  if (dnsIP == IPAddress(0,0,0,0)) dnsIP = cfg.dns;            // your saved static DNS
  if (dnsIP == IPAddress(0,0,0,0)) dnsIP = IPAddress(1,1,1,1); // last resort
  dns.begin(dnsIP);
  bool ok = (dns.getHostByName(host, out) == 1);
  g_dnsOk = ok;
  return ok;
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
    if (!g_adsActive[ch]) continue;
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
  String apiPrefix = cloudApiPrefixFromServerUrl();
  String ingestUrl = cloudIngestUrlFromServerUrl();
  String scheme, host, path; uint16_t port = 0;
  bool parsed = parseUrl(url, scheme, host, port, path);

  IPAddress dnsIP = Ethernet.dnsServerIP();
  IPAddress dnsEff = effectiveDnsServer();
  IPAddress hostIP; bool dnsOk = false, tcpOk = false;

  if (parsed && scheme == "https" && host.length()) {
    dnsOk = resolveHost(host.c_str(), hostIP);
    if (dnsOk) {
      ScopedSpiBusLock lock;
      if (!lock) {
        dnsOk = false;
      } else {
        prepareEthernetSpi();
        EthernetClient c; c.setTimeout(1500);
        tcpOk = c.connect(hostIP, port);
        c.stop();
      }
    }
  }

  String j = "{";
  j += "\"url\":\"" + url + "\",";
  j += "\"parsed\":"; j += (parsed ? "true" : "false"); j += ",";
  j += "\"scheme\":\"" + scheme + "\",";
  j += "\"host\":\"" + host + "\",";
  j += "\"port\":" + String(port) + ",";
  j += "\"dns\":\"" + dnsIP.toString() + "\",";
  j += "\"dns_effective\":\"" + dnsEff.toString() + "\",";
  j += "\"resolved\":\""; j += (dnsOk ? hostIP.toString() : ""); j += "\",";
  j += "\"tcp\":"; j += (tcpOk ? "true" : "false"); j += ",";
  j += "\"api_prefix\":\"" + apiPrefix + "\",";
  j += "\"ingest_url\":\"" + ingestUrl + "\"";
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

static String htmlEscape(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      case '\'': o += "&#39;"; break;
      default: o += c; break;
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

  uint8_t activeMask = 0;
  for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) { if (g_adsActive[ch]) activeMask |= (1u<<ch); }
  adsPrefs.putUChar("act", activeMask);

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
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    g_gainCh[ch]   = GAIN_ONE;    // 4.096 V
    g_rateCh[ch]   = 920;         // 920 SPS
    g_shuntCh[ch]  = 160.0f;      // Ω
    g_engFSmm[ch]  = 40.0f;
    g_engOffmm[ch] = 0.0f;
  }

  // ---- Only read keys that exist to avoid NOT_FOUND logs ----
  uint8_t activeMask = adsPrefs.getUChar("act", 0x0F);
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) g_adsActive[ch] = (activeMask & (1u<<ch)) != 0;

  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    char key[8];
    snprintf(key,sizeof(key),"gain%u", ch); if (adsPrefs.isKey(key)) g_gainCh[ch] = codeToGain(adsPrefs.getUChar(key, 1));
    snprintf(key,sizeof(key),"rate%u", ch); if (adsPrefs.isKey(key)) g_rateCh[ch] = adsPrefs.getInt(key, 920);
    snprintf(key,sizeof(key),"sh%u",   ch); if (adsPrefs.isKey(key)) g_shuntCh[ch]= adsPrefs.getFloat(key, 160.0f);

    snprintf(key,sizeof(key),"fs%u",   ch); if (adsPrefs.isKey(key)) g_engFSmm[ch]  = clampf(adsPrefs.getFloat(key, 40.0f), 1.0f, 10000.0f);
    snprintf(key,sizeof(key),"off%u",  ch); if (adsPrefs.isKey(key)) g_engOffmm[ch] = clampf(adsPrefs.getFloat(key, 0.0f), -100000.0f, 100000.0f);
  }

  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    if (!validSps(g_rateCh[ch])) g_rateCh[ch] = 920;
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
  needSeed |= !adsPrefs.isKey("act");
  if (needSeed) {
    adsConfigSave();
    Serial.println("[ADS] NVS seeded with defaults");
  }

  String msg = "[ADS] loaded | ";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    msg += "A" + String(ch) + " gain=" + gainToStr(g_gainCh[ch]) + " rate=" + String(g_rateCh[ch]) + " sh=" + String(g_shuntCh[ch],1) + "Ω";
    if (ch + 1 < NUM_SENSORS) msg += "; ";
  }
  msg += " | active=0b" + String(activeMask, BIN);
  Serial.println(msg);
}

static void alarmsTask(){
  static uint32_t last = 0;
  if (millis() - last < 250) return;  // ~4 Hz
  last = millis();
}

void adsInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!ads.begin(0x48, &Wire)) {
    adsReady = false;
    Serial.println("[ADS] not found @0x48");
    return;
  }

  adsReady = true;  // mark device present before applying rate/gain

  // Seed with A0 defaults; reads will set per-channel config before sampling
  ads.setGain(g_adsGain);
  adsSetRateSps(ads, g_adsRateSps);

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
  if (!g_adsActive[ch]) { raw = 0; mv = ma = pct = 0.0f; return; }
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

static bool spiBusLock(TickType_t waitTicks) {
  if (!g_spiMutex) return true;
  return xSemaphoreTakeRecursive(g_spiMutex, waitTicks) == pdTRUE;
}

static void spiBusUnlock() {
  if (g_spiMutex) xSemaphoreGiveRecursive(g_spiMutex);
}

static inline void prepareSdSpi() {
  digitalWrite(WIZ_CS, HIGH);
}

static inline void prepareEthernetSpi() {
  digitalWrite(SD_CS, HIGH);
}

static void appendRamLogLine(const String& line) {
  char buf[RAM_LOG_LINE_BYTES];
  size_t n = line.length();
  if (n >= sizeof(buf)) n = sizeof(buf) - 1;
  memcpy(buf, line.c_str(), n);
  buf[n] = '\0';

  portENTER_CRITICAL(&g_ramLogMux);
  memcpy(g_ramLogLines[g_ramLogNext], buf, n + 1);
  g_ramLogNext = static_cast<uint16_t>((g_ramLogNext + 1U) % RAM_LOG_LINE_COUNT);
  if (g_ramLogCount < RAM_LOG_LINE_COUNT) ++g_ramLogCount;
  portEXIT_CRITICAL(&g_ramLogMux);
}

static void clearRamLogBuffer() {
  portENTER_CRITICAL(&g_ramLogMux);
  g_ramLogNext = 0;
  g_ramLogCount = 0;
  memset(g_ramLogLines, 0, sizeof(g_ramLogLines));
  portEXIT_CRITICAL(&g_ramLogMux);
}

static String ramLogText(int requestedLines = -1) {
  uint16_t count = 0;
  uint16_t next = 0;
  portENTER_CRITICAL(&g_ramLogMux);
  count = g_ramLogCount;
  next = g_ramLogNext;
  portEXIT_CRITICAL(&g_ramLogMux);

  if (count == 0) return "";

  uint16_t take = count;
  if (requestedLines > 0 && requestedLines < take) take = static_cast<uint16_t>(requestedLines);
  uint16_t start = static_cast<uint16_t>((next + count - take) % RAM_LOG_LINE_COUNT);

  String out;
  out.reserve(static_cast<size_t>(take) * 96U);
  for (uint16_t i = 0; i < take; ++i) {
    char line[RAM_LOG_LINE_BYTES];
    uint16_t idx = static_cast<uint16_t>((start + i) % RAM_LOG_LINE_COUNT);
    portENTER_CRITICAL(&g_ramLogMux);
    memcpy(line, g_ramLogLines[idx], sizeof(line));
    portEXIT_CRITICAL(&g_ramLogMux);
    if (line[0] == '\0') continue;
    out += line;
    out += '\n';
  }
  return out;
}

static void emitSerialAndRamOnly(const String& msg) {
  Serial.println(msg);
  appendRamLogLine(isoNow() + " " + msg);
}

static bool loadStorageRecoveryMarker() {
  prefs.begin("app", true);
  bool value = prefs.getBool(STORAGE_RECOVERY_PREF_KEY, false);
  prefs.end();
  return value;
}

static void saveStorageRecoveryMarker(bool value) {
  prefs.begin("app", false);
  prefs.putBool(STORAGE_RECOVERY_PREF_KEY, value);
  prefs.end();
}

static bool storageIsFaulted() {
  return storage_fault::isFaulted(g_storageStatus);
}

static bool storageIsDegraded() {
  return storage_fault::isDegraded(g_storageStatus);
}

static bool storageCanUseSd() {
  return sdMounted && !storageIsFaulted();
}

static const char* storageStateName() {
  return storage_fault::stateName(g_storageStatus.state);
}

static String storageLogSource() {
  return storageCanUseSd() ? "sd" : "ram";
}

static void storageRecordSuccess() {
  if (storageIsFaulted()) return;
  storage_fault::noteSuccess(g_storageStatus, millis());
}

static void storageClearHealthy() {
  storage_fault::clear(g_storageStatus);
}

static void handlePersistentStorageFault(const String& path, const String& err) {
  bool alreadyFaulted = storageIsFaulted();
  if (!alreadyFaulted) {
    storage_fault::markFaulted(g_storageStatus, millis(), path.c_str(), err.c_str());
  } else {
    g_storageStatus.state = storage_fault::State::FAULTED;
    storage_fault::copyStringCapped(err.c_str(), g_storageStatus.lastError, sizeof(g_storageStatus.lastError));
    storage_fault::copyStringCapped(path.c_str(), g_storageStatus.lastPath, sizeof(g_storageStatus.lastPath));
  }
  storage_fault::setUploadBlocked(g_storageStatus, true);
  sdMounted = false;
  g_measAutoRestartPending = false;
  g_measAutoRestartWaitingUpload = false;
  g_measAutoRestartLastAttemptMs = 0;
  g_measAutoRestartLastLogMs = 0;

  bool wasMeasuring = g_measActive;
  if (g_measActive) {
    g_measActive = false;
    if (g_measTask && xTaskGetCurrentTaskHandle() != g_measTask) {
      uint32_t t0 = millis();
      while (g_measTask && millis() - t0 < 800UL) {
        delay(10);
      }
    }
  }

  emitSerialAndRamOnly(String("[SD] STORAGE FAULT err=") + err + " path=" + path);
  if (wasMeasuring) {
    emitSerialAndRamOnly("[MEAS] storage fault: measurement stopped");
  }

  if (!g_storageStatus.recoveryRebootAttempted) {
    storage_fault::markRecoveryAttempted(g_storageStatus, true);
    saveStorageRecoveryMarker(true);
    scheduleDeferredReboot("[SD] storage fault: reboot scheduled for recovery");
  } else if (!alreadyFaulted) {
    emitSerialAndRamOnly("[SD] storage fault still active after recovery reboot; staying in standby");
  }
}

static void storageRecordFailure(const String& path, const String& err, uint8_t severity) {
  bool wasFaulted = storageIsFaulted();
  storage_fault::noteFailure(g_storageStatus, millis(), path.c_str(), err.c_str(), severity);
  if (!wasFaulted && storageIsFaulted()) {
    handlePersistentStorageFault(path, err);
  }
}

static bool runSdSelfTest(String& outErr, String& outPath) {
  outErr = "";
  outPath = "";
  if (!sdMounted) {
    outErr = "mount_fail";
    return false;
  }

  ScopedSpiBusLock lock;
  if (!lock) {
    outErr = "spi_lock_fail";
    return false;
  }
  prepareSdSpi();

  if (!SD.exists("/logs") && !SD.mkdir("/logs")) {
    outErr = "mkdir_fail";
    outPath = "/logs";
    return false;
  }
  if (!SD.exists("/meas") && !SD.mkdir("/meas")) {
    outErr = "mkdir_fail";
    outPath = "/meas";
    return false;
  }

  const String probePath = "/meas/.sd_selftest.tmp";
  if (SD.exists(probePath)) SD.remove(probePath);
  File f = SD.open(probePath, FILE_WRITE);
  if (!f) {
    outErr = "open_fail";
    outPath = probePath;
    return false;
  }
  static const uint8_t probeData[] = {'o', 'k', '\n'};
  size_t written = f.write(probeData, sizeof(probeData));
  f.close();
  if (written != sizeof(probeData)) {
    SD.remove(probePath);
    outErr = "write_fail";
    outPath = probePath;
    return false;
  }
  if (!SD.remove(probePath)) {
    outErr = "remove_fail";
    outPath = probePath;
    return false;
  }

  storageRecordSuccess();
  return true;
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

static EthernetLinkStatus ethLinkStatusSafe() {
  ScopedSpiBusLock lock;
  if (!lock) return Unknown;
  prepareEthernetSpi();
  return Ethernet.linkStatus();
}

static IPAddress ethLocalIpSafe() {
  ScopedSpiBusLock lock;
  if (!lock) return IPAddress(0, 0, 0, 0);
  prepareEthernetSpi();
  return Ethernet.localIP();
}

static IPAddress ethGatewayIpSafe() {
  ScopedSpiBusLock lock;
  if (!lock) return IPAddress(0, 0, 0, 0);
  prepareEthernetSpi();
  return Ethernet.gatewayIP();
}

static IPAddress ethDnsIpSafe() {
  ScopedSpiBusLock lock;
  if (!lock) return IPAddress(0, 0, 0, 0);
  prepareEthernetSpi();
  return Ethernet.dnsServerIP();
}

// Ping ADS @0x48, auto-reinit I²C + ADS if it stops ACKing
static void adsWatchdog() {
  if (g_measActive) return;   // don't poke ADC mid-session
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
  cfg.deviceId  = prefs.getString("deviceId",  cfg.deviceId);
  cfg.cmdSecret = prefs.getString("cmdSecret", cfg.cmdSecret);
  cfg.localAuthToken = prefs.getString("localTok", cfg.localAuthToken);
  cfg.apPass    = prefs.getString("apPass",    cfg.apPass);
  cfg.commissioningMode = prefs.getBool("commMode", cfg.commissioningMode);
  cfg.remoteEnabled = prefs.getBool("remoteEn", cfg.remoteEnabled);
  cfg.lastCommandId = prefs.getString("lastCmdId", cfg.lastCommandId);
  cfg.useStatic = prefs.getBool  ("useStatic", cfg.useStatic);

  IPAddress t;
  if (parseIP(prefs.getString("ip",   "0.0.0.0"), t)) cfg.ip=t;
  if (parseIP(prefs.getString("gw",   "0.0.0.0"), t)) cfg.gw=t;
  if (parseIP(prefs.getString("mask", "255.255.255.0"), t)) cfg.mask=t;
  if (parseIP(prefs.getString("dns",  "1.1.1.1"), t)) cfg.dns=t;

  // Cloud
  cfg.cloudEnabled = prefs.getBool   ("cloudEn",   cfg.cloudEnabled);
  cfg.cloudPeriodS = prefs.getUInt   ("cloudPer",  cfg.cloudPeriodS);
  cfg.uploadOnStop = prefs.getBool   ("uplOnStop", cfg.uploadOnStop);
  cfg.tlsFp        = prefs.getString ("tlsfp",     cfg.tlsFp);
  prefs.end();
}

void saveCfg() {
  prefs.begin("app", false);
  prefs.putString("devName",   cfg.devName);
  prefs.putString("serverUrl", cfg.serverUrl);
  prefs.putString("apiKey",    cfg.apiKey);
  prefs.putString("deviceId",  cfg.deviceId);
  prefs.putString("cmdSecret", cfg.cmdSecret);
  prefs.putString("localTok",  cfg.localAuthToken);
  prefs.putString("apPass",    cfg.apPass);
  prefs.putBool  ("commMode",  cfg.commissioningMode);
  prefs.putBool  ("remoteEn",  cfg.remoteEnabled);
  prefs.putString("lastCmdId", cfg.lastCommandId);
  prefs.putBool  ("useStatic", cfg.useStatic);
  prefs.putString("ip",   cfg.ip.toString());
  prefs.putString("gw",   cfg.gw.toString());
  prefs.putString("mask", cfg.mask.toString());
  prefs.putString("dns",  cfg.dns.toString());
  // Cloud
  prefs.putBool  ("cloudEn",   cfg.cloudEnabled);
  prefs.putUInt  ("cloudPer",  cfg.cloudPeriodS);
  prefs.putBool  ("uplOnStop", cfg.uploadOnStop);
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

static uint32_t runtimeFreeHeap() {
  return ESP.getFreeHeap();
}

static uint32_t runtimeMinFreeHeap() {
  return ESP.getMinFreeHeap();
}

static uint32_t runtimeLargestFreeBlock() {
#if defined(ESP32)
  return heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
#else
  return 0;
#endif
}

static uint8_t runtimePct(uint32_t part, uint32_t total) {
  if (total == 0) return 0;
  uint64_t pct = (static_cast<uint64_t>(part) * 100ULL) / static_cast<uint64_t>(total);
  if (pct > 100ULL) pct = 100ULL;
  return static_cast<uint8_t>(pct);
}

static uint8_t runtimeHeapFragPct() {
  uint32_t freeHeap = runtimeFreeHeap();
  uint32_t largest = runtimeLargestFreeBlock();
  if (freeHeap == 0 || largest >= freeHeap) return 0;
  return static_cast<uint8_t>(100UL - ((largest * 100UL) / freeHeap));
}

static uint32_t stackWatermarkBytes(TaskHandle_t task) {
  if (!task) return 0;
  return static_cast<uint32_t>(uxTaskGetStackHighWaterMark(task)) * sizeof(StackType_t);
}

static void copyStringToFixed(char* dst, size_t len, const String& src) {
  if (!dst || len == 0) return;
  size_t n = src.length();
  if (n >= len) n = len - 1;
  memcpy(dst, src.c_str(), n);
  dst[n] = '\0';
}

static void captureRuntimeBreadcrumb() {
  RuntimeBreadcrumb b{};
  b.magic = RUNTIME_BREADCRUMB_MAGIC;
  b.version = RUNTIME_BREADCRUMB_VERSION;
  b.uptimeMs = millis();
  b.freeHeap = runtimeFreeHeap();
  b.minFreeHeap = runtimeMinFreeHeap();
  b.largestFreeBlock = runtimeLargestFreeBlock();
  b.loopMaxBlockMs = g_loopMaxBlockMs;
  if (g_measActive) b.flags |= (1u << 0);
  if (g_linkOk) b.flags |= (1u << 1);
  if (g_internetOk) b.flags |= (1u << 2);
  if (cfg.cloudEnabled) b.flags |= (1u << 3);
  if (cfg.remoteEnabled) b.flags |= (1u << 4);
  copyStringToFixed(b.lastNetOp, sizeof(b.lastNetOp), g_lastNetOp);
  copyStringToFixed(b.lastStage, sizeof(b.lastStage), String(g_lastRuntimeStage));
  copyStringToFixed(b.lastNetErr, sizeof(b.lastNetErr), g_lastNetErr);
  copyStringToFixed(b.measFile, sizeof(b.measFile), g_measFile);
  g_rtcBreadcrumb = b;
}

static String previousBreadcrumbSummary() {
  if (!g_havePrevBreadcrumb) return "";
  uint8_t fragPct = 0;
  if (g_prevBreadcrumb.freeHeap > 0 && g_prevBreadcrumb.largestFreeBlock < g_prevBreadcrumb.freeHeap) {
    fragPct = static_cast<uint8_t>(100UL - ((g_prevBreadcrumb.largestFreeBlock * 100UL) / g_prevBreadcrumb.freeHeap));
  }
  String s;
  s.reserve(256);
  s += "uptime_ms=" + String(g_prevBreadcrumb.uptimeMs);
  s += " heap=" + String(g_prevBreadcrumb.freeHeap);
  s += " min_heap=" + String(g_prevBreadcrumb.minFreeHeap);
  s += " largest=" + String(g_prevBreadcrumb.largestFreeBlock);
  s += " frag=" + String((unsigned)fragPct) + "%";
  s += " loop_max=" + String(g_prevBreadcrumb.loopMaxBlockMs);
  s += " meas=" + String((g_prevBreadcrumb.flags & (1u << 0)) ? "1" : "0");
  if (g_prevBreadcrumb.lastNetOp[0]) s += " net_op=" + String(g_prevBreadcrumb.lastNetOp);
  if (g_prevBreadcrumb.lastStage[0]) s += " stage=" + String(g_prevBreadcrumb.lastStage);
  if (g_prevBreadcrumb.lastNetErr[0]) s += " net_err=" + String(g_prevBreadcrumb.lastNetErr);
  if (g_prevBreadcrumb.measFile[0]) s += " file=" + String(g_prevBreadcrumb.measFile);
  return s;
}

static void prepareBootBreadcrumb() {
  g_havePrevBreadcrumb =
    g_rtcBreadcrumb.magic == RUNTIME_BREADCRUMB_MAGIC &&
    g_rtcBreadcrumb.version == RUNTIME_BREADCRUMB_VERSION;
  if (g_havePrevBreadcrumb) {
    g_prevBreadcrumb = g_rtcBreadcrumb;
  } else {
    memset(&g_prevBreadcrumb, 0, sizeof(g_prevBreadcrumb));
  }
  memset(&g_rtcBreadcrumb, 0, sizeof(g_rtcBreadcrumb));
  captureRuntimeBreadcrumb();
}

static void noteNetOp(const String& op) {
  g_lastNetOp = op;
}

static void noteNetResult(bool ok, const String& err) {
  if (ok) {
    g_lastNetErr = "";
    g_lastNetErrMs = 0;
    return;
  }
  g_lastNetErr = err;
  g_lastNetErrMs = millis();
}

static void noteRuntimeStage(const char* stage) {
  if (!stage || !stage[0]) return;
  snprintf(g_lastRuntimeStage, sizeof(g_lastRuntimeStage), "%s", stage);
  g_lastRuntimeStageMs = millis();
}

static void runtimeCpuSampleTick() {
#if defined(ESP32) && (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1)
  static uint32_t s_lastSampleMs = 0;
  static uint32_t s_prevTotalRuntime = 0;
  static uint32_t s_prevIdleRuntime = 0;
  static uint32_t s_prevLoopRuntime = 0;
  static uint32_t s_prevMeasRuntime = 0;

  uint32_t now = millis();
  if (g_runtimeCpu.sampledAtMs != 0 && (now - s_lastSampleMs) < 5000UL) return;
  s_lastSampleMs = now;

  UBaseType_t taskCapacity = uxTaskGetNumberOfTasks();
  if (taskCapacity < 8) taskCapacity = 8;
  taskCapacity += 4;

  TaskStatus_t* tasks = static_cast<TaskStatus_t*>(malloc(sizeof(TaskStatus_t) * taskCapacity));
  if (!tasks) {
    g_runtimeCpu.valid = false;
    g_runtimeCpu.sampledAtMs = now;
    return;
  }

  uint32_t totalRuntime = 0;
  UBaseType_t taskCount = uxTaskGetSystemState(tasks, taskCapacity, &totalRuntime);
  uint32_t idleRuntime = 0;
  uint32_t loopRuntime = 0;
  uint32_t measRuntime = 0;

  for (UBaseType_t i = 0; i < taskCount; ++i) {
    const char* name = tasks[i].pcTaskName;
    if (name && strncmp(name, "IDLE", 4) == 0) {
      idleRuntime += tasks[i].ulRunTimeCounter;
    }
    if (tasks[i].xHandle == g_loopTaskHandle) {
      loopRuntime = tasks[i].ulRunTimeCounter;
    }
    if (tasks[i].xHandle == g_measTask) {
      measRuntime = tasks[i].ulRunTimeCounter;
    }
  }
  free(tasks);

  g_runtimeCpu.sampledAtMs = now;
  g_runtimeCpu.taskCount = static_cast<uint16_t>(taskCount);

  if (s_prevTotalRuntime != 0 && totalRuntime > s_prevTotalRuntime) {
    uint32_t totalDelta = totalRuntime - s_prevTotalRuntime;
    uint32_t idleDelta = (idleRuntime >= s_prevIdleRuntime) ? (idleRuntime - s_prevIdleRuntime) : 0;
    uint32_t loopDelta = (loopRuntime >= s_prevLoopRuntime) ? (loopRuntime - s_prevLoopRuntime) : 0;
    uint32_t measDelta = (measRuntime >= s_prevMeasRuntime) ? (measRuntime - s_prevMeasRuntime) : 0;
    uint8_t idlePct = runtimePct(idleDelta, totalDelta);
    g_runtimeCpu.cpuBusyPct = static_cast<uint8_t>(idlePct >= 100 ? 0 : (100 - idlePct));
    g_runtimeCpu.loopCpuPct = runtimePct(loopDelta, totalDelta);
    g_runtimeCpu.measCpuPct = runtimePct(measDelta, totalDelta);
    g_runtimeCpu.valid = true;
  } else {
    g_runtimeCpu.valid = false;
    g_runtimeCpu.cpuBusyPct = 0;
    g_runtimeCpu.loopCpuPct = 0;
    g_runtimeCpu.measCpuPct = 0;
  }

  s_prevTotalRuntime = totalRuntime;
  s_prevIdleRuntime = idleRuntime;
  s_prevLoopRuntime = loopRuntime;
  s_prevMeasRuntime = measRuntime;
#else
  g_runtimeCpu.valid = false;
  g_runtimeCpu.sampledAtMs = millis();
  g_runtimeCpu.taskCount = 0;
  g_runtimeCpu.cpuBusyPct = 0;
  g_runtimeCpu.loopCpuPct = 0;
  g_runtimeCpu.measCpuPct = 0;
#endif
}

static void runtimeHealthTick() {
  uint32_t now = millis();
  runtimeCpuSampleTick();
  if ((now - g_lastBreadcrumbMs) >= 5000UL) {
    g_lastBreadcrumbMs = now;
    captureRuntimeBreadcrumb();
  }
  if ((now - g_lastHealthLogMs) >= 300000UL) {
    g_lastHealthLogMs = now;
    String msg = String("[HEALTH] heap=") + String(runtimeFreeHeap())
                 + " min=" + String(runtimeMinFreeHeap())
                 + " largest=" + String(runtimeLargestFreeBlock())
                 + " frag=" + String((unsigned)runtimeHeapFragPct()) + "%"
                 + " loop_max=" + String(g_loopMaxBlockMs)
                 + " loop_stack=" + String(stackWatermarkBytes(g_loopTaskHandle))
                 + " meas_stack=" + String(stackWatermarkBytes(g_measTask))
                 + " meas=" + String(g_measActive ? "1" : "0")
                 + " stage=" + String(g_lastRuntimeStage)
                 + " net_op=" + g_lastNetOp;
    if (g_runtimeCpu.valid) {
      msg += String(" cpu=") + String((unsigned)g_runtimeCpu.cpuBusyPct) + "%"
             + " loop_cpu=" + String((unsigned)g_runtimeCpu.loopCpuPct) + "%"
             + " meas_cpu=" + String((unsigned)g_runtimeCpu.measCpuPct) + "%"
             + " tasks=" + String((unsigned)g_runtimeCpu.taskCount);
    } else {
      msg += " cpu=na";
    }
    if (g_lastNetErr.length()) msg += String(" net_err=") + g_lastNetErr;
    logLine(msg);
  }
}

// ---- TIME/NTP ----
static bool measurementNetworkDeferralActive() {
  return g_measActive;
}

static uint32_t measurementDeferredIntervalMs(uint32_t baseMs) {
  return measurement_safety::deferredIntervalMs(
    measurementNetworkDeferralActive(),
    baseMs,
    MEASUREMENT_NET_DEFER_INTERVAL_MS
  );
}

static void measurementNetworkPolicyTick() {
  bool deferred = measurementNetworkDeferralActive();
  if (deferred == g_measurementNetDeferredPrev) return;
  g_measurementNetDeferredPrev = deferred;
  if (!deferred) {
    g_lastInetCheckMs = 0;
    g_lastDhcpMaintainMs = 0;
    g_lastRemotePollMs = 0;
    g_lastPushMs = 0;
  }
}

void setupTime() {
  // Europe/Budapest (CET/CEST)
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.google.com", "time.cloudflare.com");
}
String isoNow() {
  time_t now = time(nullptr);
  if (now < 1609459200) {
    uint64_t ms = millis();
    char buf[40];
    snprintf(buf, sizeof(buf), "uptime+%lu.%03lus", (unsigned long)(ms/1000u), (unsigned long)(ms%1000u));
    return String(buf);
  }
  struct tm t; localtime_r(&now, &t);
  char buf[40]; strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &t);
  return String(buf);
}
static inline bool isEpochSane(time_t now) {
  return now >= 1609459200; // 2021-01-01
}

String isoNowFileSafe() {
  time_t now = time(nullptr);
  if (!isEpochSane(now)) {
    char uns[32];
    snprintf(uns, sizeof(uns), "unsynced_%010lu", (unsigned long)millis());
    return String(uns);
  }
  struct tm t; localtime_r(&now, &t);
  char buf[32];
  // local time, no timezone => no '+'
  strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &t);
  return String(buf);
}

// NTP over W5500 (UDP). Sets system time + Budapest TZ on success.
bool ntpSyncW5500(const char* host = "pool.ntp.org", uint16_t timeoutMs = 1500) {
  ScopedSpiBusLock lock;
  if (!lock) return false;
  prepareEthernetSpi();
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
        g_lastTimeSyncMs = millis();
        g_ntpRetryMs = 3600000UL;
        return true;
      }
    }
    delay(10);
  }
  return false;
}

// ---- LOGGING (size-based rotation) ----
void ensureLogsDir() {
  if (!storageCanUseSd()) return;
  ScopedSpiBusLock lock;
  if (!lock) return;
  prepareSdSpi();
  if (!SD.exists("/logs") && !SD.mkdir("/logs")) {
    storageRecordFailure("/logs", "mkdir_fail");
    return;
  }
  if (!SD.exists(currentLogPath)) {
    File f = SD.open(currentLogPath, FILE_WRITE);
    if (!f) {
      storageRecordFailure(currentLogPath, "open_fail");
      return;
    }
    f.close();
  }
  storageRecordSuccess();
}
void rotateLogsIfNeeded() {
  if (!storageCanUseSd()) return;
  ScopedSpiBusLock lock;
  if (!lock) return;
  prepareSdSpi();
  File cur = SD.open(currentLogPath, FILE_READ);
  if (!cur) {
    storageRecordFailure(currentLogPath, "open_fail");
    return;
  }
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
  if (!f) {
    storageRecordFailure("/logs/log0.log", "open_fail");
    return;
  }
  f.close();
  storageRecordSuccess();
}
static void logLine(const String& msg) {
  Serial.println(msg);
  String line = isoNow() + " " + msg;
  appendRamLogLine(line);
  if (!storageCanUseSd()) return;
  noteRuntimeStage("log_sd");
  ensureLogsDir();
  if (!storageCanUseSd()) return;
  rotateLogsIfNeeded();
  if (!storageCanUseSd()) return;
  ScopedSpiBusLock lock;
  if (!lock) return;
  prepareSdSpi();
  File f = SD.open(currentLogPath, FILE_APPEND);
  if (!f) {
    storageRecordFailure(currentLogPath, "open_fail");
    return;
  }
  line += "\n";
  size_t wrote = f.print(line);
  f.close();
  if (wrote != line.length()) {
    storageRecordFailure(currentLogPath, "write_short");
    return;
  }
  storageRecordSuccess();
}

// ---- ETHERNET / QoL ----
bool ethernetDHCP() {
  Serial.println("[ETH] DHCP…");
  if (ethLinkStatusSafe() == LinkOFF) {
    Serial.println("[ETH] Link down — skipping DHCP");
    return false;
  }
  // try DHCP quickly so boot doesn't stall with no cable
  unsigned long start = millis(); bool ok=false;
  while (millis()-start < 3000) {
    {
      ScopedSpiBusLock lock;
      if (!lock) break;
      prepareEthernetSpi();
      if (Ethernet.begin(ethMac)) { ok=true; break; }
    }
    if (ethLinkStatusSafe() == LinkOFF) break;
    delay(250);
  }
  if (ok) {
    g_dnsOk = ethDnsIpSafe() != IPAddress(0,0,0,0);
  }
  return ok;
}
bool ethernetInit() {
  // reset WIZ
  pinMode(WIZ_RST, OUTPUT);
  digitalWrite(WIZ_RST, LOW);  delay(5);
  digitalWrite(WIZ_RST, HIGH); delay(60);

  {
    ScopedSpiBusLock lock;
    if (!lock) return false;
    prepareEthernetSpi();
    SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, WIZ_CS);
    Ethernet.init(WIZ_CS);
    g_linkOk = (Ethernet.linkStatus() == LinkON);
  }

  bool ok = ethernetDHCP();
  if (!ok && cfg.useStatic) {
    Serial.printf("[ETH] DHCP failed → static: ip=%s gw=%s mask=%s dns=%s\n",
      cfg.ip.toString().c_str(), cfg.gw.toString().c_str(),
      cfg.mask.toString().c_str(), cfg.dns.toString().c_str());
    ScopedSpiBusLock lock;
    if (lock) {
      prepareEthernetSpi();
      Ethernet.begin(ethMac, cfg.ip, cfg.dns, cfg.gw, cfg.mask);
    }
    ok = true;
  }
  if (ok) {
    Serial.print("[ETH] IP: ");      Serial.println(ethLocalIpSafe());
    Serial.print("[ETH] Gateway: "); Serial.println(ethGatewayIpSafe());
    Serial.print("[ETH] DNS: ");     Serial.println(ethDnsIpSafe());
    g_dnsOk = (ethDnsIpSafe() != IPAddress(0,0,0,0));
  } else {
    Serial.println("[ETH] No IP.");
    g_dnsOk = false;
  }
  return ok;
}
bool internetOK(uint16_t timeoutMs = 1500) {
  ScopedSpiBusLock lock;
  if (!lock) return false;
  prepareEthernetSpi();
  EthernetClient c; c.setTimeout(timeoutMs);
  if (!c.connect(testHost, testPort)) return false;
  c.stop(); return true;
}

static bool refreshInternetState(bool force=false, uint16_t timeoutMs = INTERNET_CHECK_TIMEOUT_MS) {
  uint32_t now = millis();
  uint32_t baseIntervalMs = measurementDeferredIntervalMs(INTERNET_CHECK_INTERVAL_MS);
  uint32_t intervalMs = g_internetOk ? baseIntervalMs : max(baseIntervalMs, g_internetCheckBackoffMs);
  if (!force && (now - g_lastInetCheckMs) < intervalMs) {
    return g_internetOk;
  }
  g_lastInetCheckMs = now;
  g_linkOk = (ethLinkStatusSafe() == LinkON);
  if (!g_linkOk) {
    g_dnsOk = false;
    g_internetOk = false;
    return false;
  }
  g_internetOk = internetOK(timeoutMs);
  if (g_internetOk) {
    g_internetCheckBackoffMs = baseIntervalMs;
  } else {
    uint32_t next = g_internetCheckBackoffMs;
    if (next < baseIntervalMs) next = baseIntervalMs;
    if (next >= (INTERNET_CHECK_OFFLINE_MAX_MS / 2UL)) next = INTERNET_CHECK_OFFLINE_MAX_MS;
    else next *= 2UL;
    if (next > INTERNET_CHECK_OFFLINE_MAX_MS) next = INTERNET_CHECK_OFFLINE_MAX_MS;
    g_internetCheckBackoffMs = next;
  }
  return g_internetOk;
}

static void dhcpMaintainTick() {
  if (cfg.useStatic) return;
  uint32_t now = millis();
  if (now - g_lastDhcpMaintainMs < measurementDeferredIntervalMs(10000UL)) return;
  g_lastDhcpMaintainMs = now;

  int rc = 0;
  {
    ScopedSpiBusLock lock;
    if (!lock) return;
    prepareEthernetSpi();
    rc = Ethernet.maintain();
  }
  if (rc == 1 || rc == 3) {
    logLine(String("[ETH] DHCP renewed IP=") + ethLocalIpSafe().toString());
    g_dnsOk = (ethDnsIpSafe() != IPAddress(0,0,0,0));
  } else if (rc == 2 || rc == 4) {
    logLine(String("[ETH] DHCP renew/rebind failed rc=") + String(rc));
  }
}

static bool tryNtpNow() {
  if (!g_linkOk || !g_internetOk) return false;
  g_lastNtpAttemptMs = millis();
  noteNetOp("ntp_sync");
  if (ntpSyncW5500("pool.ntp.org") || ntpSyncW5500("time.google.com") || ntpSyncW5500("time.cloudflare.com")) {
    noteNetResult(true);
    logLine("[TIME] NTP sync OK: " + isoNow());
    g_ntpRetryMs = 3600000UL;
    return true;
  }
  noteNetResult(false, "ntp_sync_failed");
  logLine("[TIME] NTP sync FAILED; DNS=" + ethDnsIpSafe().toString());
  if (!g_timeSynced) {
    uint32_t next = g_ntpRetryMs * 2UL;
    if (next < 30000UL) next = 30000UL;
    if (next > 900000UL) next = 900000UL;
    g_ntpRetryMs = next;
  } else {
    g_ntpRetryMs = 3600000UL;
  }
  return false;
}

static void ntpSyncTick() {
  if (measurementNetworkDeferralActive()) return;
  uint32_t now = millis();
  if (!g_linkOk || !g_internetOk) return;
  if (g_forceNtpSync || (now - g_lastNtpAttemptMs >= g_ntpRetryMs)) {
    g_forceNtpSync = false;
    tryNtpNow();
  }
}

void linkWatchdog() {
  EthernetLinkStatus lk = ethLinkStatusSafe();
  if (lk != lastLink) {
    lastLink = lk;
    logLine(String("[ETH] Link ") + (lk==LinkON?"UP":(lk==LinkOFF?"DOWN":"UNKNOWN")));
    if (lk==LinkON) {
      g_internetCheckBackoffMs = measurementDeferredIntervalMs(INTERNET_CHECK_INTERVAL_MS);
      // got link → (re)acquire IP
      if (cfg.useStatic) {
        ScopedSpiBusLock lock;
        if (lock) {
          prepareEthernetSpi();
          Ethernet.begin(ethMac, cfg.ip, cfg.dns, cfg.gw, cfg.mask);
        }
      } else {
        ethernetDHCP();
      }
      refreshInternetState(true);
      g_forceNtpSync = true;
    } else {
      g_internetOk = false;
      g_dnsOk = false;
      g_cloudOk = false;
      g_internetCheckBackoffMs = measurementDeferredIntervalMs(INTERNET_CHECK_INTERVAL_MS);
    }
  }
}

// ---- SD init ----
bool sdInit() {
  ScopedSpiBusLock lock;
  if (!lock) {
    Serial.println("[SD] SPI lock FAIL");
    sdMounted = false;
    return false;
  }
  prepareSdSpi(); // deselect Ethernet
  SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] Mount FAIL");
    sdMounted = false;
    return false;
  }
  sdMounted = true;
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
static bool portalMeasurementLockActive() {
  return cfg.commissioningMode && (g_measActive || g_portalMeasurementLockHold);
}

static void sendPortalMeasurementLockedJson() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(423, "application/json", "{\"ok\":false,\"err\":\"measurement_active_portal_locked\"}");
}

static void sendPortalMeasurementLockedText() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(423, "text/plain", "measurement_active_portal_locked");
}

static bool rejectPortalDuringMeasurementJson() {
  if (!portalMeasurementLockActive()) return false;
  sendPortalMeasurementLockedJson();
  return true;
}

static bool rejectPortalDuringMeasurementText() {
  if (!portalMeasurementLockActive()) return false;
  sendPortalMeasurementLockedText();
  return true;
}

static void handleMeasurementLockRoot() {
  String page(FPSTR(MEASUREMENT_RUNNING_HTML));
  page.replace("%MEASACTIVE%", g_measActive ? "true" : "false");
  page.replace("%MEASID%", htmlEscape(g_measId.length() ? g_measId : String("(unknown)")));
  page.replace("%MEASFILE%", htmlEscape(g_measFile.length() ? g_measFile : String("(none)")));
  page.replace("%MEASFRAMES%", String((unsigned long)g_frameCount));
  page.replace("%MEASBYTES%", String((unsigned long long)g_measBytes));
  page.replace("%NOW%", htmlEscape(isoNow()));
  page.replace("%UPTIME%", htmlEscape(uptimeStr()));
  page.replace("%AUTHTOKEN%", jsonEscape(cfg.localAuthToken));
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(200, "text/html", page);
}

void handleRoot(){
  if (cfg.commissioningMode && !g_measActive && g_portalMeasurementLockHold &&
      server.hasArg("full") && server.arg("full") == "1") {
    g_portalMeasurementLockHold = false;
  }
  if (portalMeasurementLockActive()) {
    handleMeasurementLockRoot();
    return;
  }
  String page(FPSTR(INDEX_HTML));
  page.replace("%DEVNAME%", cfg.devName);
  page.replace("%SERVERURL%", cfg.serverUrl);
  page.replace("%APIKEY%", cfg.apiKey);
  page.replace("%DEVICEID%", cfg.deviceId);
  page.replace("%CMDSECRET%", cfg.cmdSecret);
  page.replace("%AUTHTOKEN%", cfg.localAuthToken);
  page.replace("%REMOTECHK%", cfg.remoteEnabled ? "checked" : "");
  page.replace("%CLOUDCHK%",       cfg.cloudEnabled ? "checked" : "");
  page.replace("%UPLOADSTOPCHK%",  cfg.uploadOnStop ? "checked" : "");
  page.replace("%COMMISSIONCHK%", cfg.commissioningMode ? "checked" : "");
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
  page.replace("%APPASS%", cfg.apPass);
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  server.send(200, "text/html", page);
}
void handleSave(){
  if (server.hasArg("devName")) cfg.devName = server.arg("devName");
  if (server.hasArg("serverUrl")) cfg.serverUrl = server.arg("serverUrl");
  if (server.hasArg("apiKey")) cfg.apiKey = server.arg("apiKey");
  if (server.hasArg("deviceId")) cfg.deviceId = server.arg("deviceId");
  if (server.hasArg("cmdSecret")) cfg.cmdSecret = server.arg("cmdSecret");
  if (server.hasArg("mode")) cfg.useStatic = (server.arg("mode")=="static");
  cfg.remoteEnabled = server.hasArg("remoteEnabled");
  cfg.commissioningMode = server.hasArg("commissioning");
  if (server.hasArg("apPass")) {
    String p = server.arg("apPass");
    p.trim();
    if (p.length() >= 8) cfg.apPass = p;
  }
  IPAddress t;
  if (cfg.useStatic) {
    if (parseIP(server.arg("ip"), t))   cfg.ip=t;
    if (parseIP(server.arg("gw"), t))   cfg.gw=t;
    if (parseIP(server.arg("mask"), t)) cfg.mask=t;
    if (parseIP(server.arg("dns"), t))  cfg.dns=t;
  }
  // Cloud fields (optional in UI)
  cfg.cloudEnabled = server.hasArg("cloud");
  cfg.uploadOnStop = server.hasArg("uploadOnStop");
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
  bool link = g_linkOk;
  bool inet = g_internetOk;
  String mdnsName = g_mdnsRunning ? (cfg.devName + ".local") : "off";
  String ipStr = ethLocalIpSafe().toString();
  uint32_t freeHeap = runtimeFreeHeap();
  uint32_t minFreeHeap = runtimeMinFreeHeap();
  uint32_t largestFreeBlock = runtimeLargestFreeBlock();
  uint8_t heapFragPct = runtimeHeapFragPct();
  uint32_t loopStackFree = stackWatermarkBytes(g_loopTaskHandle);
  uint32_t measStackFree = stackWatermarkBytes(g_measTask);
  uint32_t loopAgeMs = g_loopLastMs ? (millis() - g_loopLastMs) : 0;

  String j = "{";
  j += "\"ethUp\":" + String((lastLink != Unknown) ? "true" : "false") + ",";
  j += "\"link\":"  + String(link ? "true" : "false") + ",";
  j += "\"link_ok\":" + String(link ? "true" : "false") + ",";
  j += "\"dns_ok\":" + String(g_dnsOk ? "true" : "false") + ",";
  j += "\"internet_ok\":" + String(inet ? "true" : "false") + ",";
  j += "\"cloud_ok\":" + String(g_cloudOk ? "true" : "false") + ",";
  j += "\"inet\":" + String(inet?"true":"false") + ",";
  j += "\"ip\":\""  + ipStr + "\",";
  j += "\"sd\":"    + String(storageCanUseSd() ? "true" : "false") + ",";
  j += "\"time\":\""+ isoNow() + "\",";
  j += "\"timesynced\":" + String(g_timeSynced ? "true" : "false") + ",";
  j += "\"clock_sync_age_ms\":" + String(g_lastTimeSyncMs ? (millis()-g_lastTimeSyncMs) : 0) + ",";
  j += "\"uptime\":\"" + uptimeStr() + "\",";
  j += "\"reboot\":\"" + resetReasonStr() + "\",";
  j += "\"mdns\":\"" + mdnsName + "\",";
  j += "\"loop_age_ms\":" + String(loopAgeMs) + ",";
  j += "\"loop_max_block_ms\":" + String(g_loopMaxBlockMs) + ",";
  j += "\"last_stage\":\"" + jsonEscape(String(g_lastRuntimeStage)) + "\",";
  j += "\"last_net_op\":\"" + jsonEscape(g_lastNetOp) + "\",";
  j += "\"last_net_err\":\"" + jsonEscape(g_lastNetErr) + "\"";

  j += ",\"runtime\":{";
  j +=   "\"heap_free\":" + String(freeHeap) + ",";
  j +=   "\"heap_min\":" + String(minFreeHeap) + ",";
  j +=   "\"heap_largest\":" + String(largestFreeBlock) + ",";
  j +=   "\"heap_frag_pct\":" + String((unsigned)heapFragPct) + ",";
  j +=   "\"task_count\":" + String((unsigned)g_runtimeCpu.taskCount) + ",";
  j +=   "\"cpu_busy_pct\":";
  j +=   g_runtimeCpu.valid ? String((unsigned)g_runtimeCpu.cpuBusyPct) : String("null");
  j +=   ",";
  j +=   "\"loop_cpu_pct\":";
  j +=   g_runtimeCpu.valid ? String((unsigned)g_runtimeCpu.loopCpuPct) : String("null");
  j +=   ",";
  j +=   "\"meas_cpu_pct\":";
  j +=   g_runtimeCpu.valid ? String((unsigned)g_runtimeCpu.measCpuPct) : String("null");
  j +=   ",";
  j +=   "\"loop_stack_free\":" + String(loopStackFree) + ",";
  j +=   "\"meas_stack_free\":" + String(measStackFree) + ",";
  j +=   "\"loop_age_ms\":" + String(loopAgeMs) + ",";
  j +=   "\"loop_max_block_ms\":" + String(g_loopMaxBlockMs) + ",";
  j +=   "\"last_stage\":\"" + jsonEscape(String(g_lastRuntimeStage)) + "\",";
  j +=   "\"last_stage_age_ms\":" + String(g_lastRuntimeStageMs ? (millis() - g_lastRuntimeStageMs) : 0) + ",";
  j +=   "\"last_net_op\":\"" + jsonEscape(g_lastNetOp) + "\",";
  j +=   "\"last_net_err\":\"" + jsonEscape(g_lastNetErr) + "\",";
  j +=   "\"last_net_err_age_ms\":" + String(g_lastNetErrMs ? (millis() - g_lastNetErrMs) : 0);
  j += "}";

  j += ",\"storage\":{";
  j +=   "\"state\":\"" + jsonEscape(String(storageStateName())) + "\",";
  j +=   "\"fault\":" + String(storageIsFaulted() ? "true" : "false") + ",";
  j +=   "\"degraded\":" + String(storageIsDegraded() ? "true" : "false") + ",";
  j +=   "\"boot_mounted\":" + String(g_sdBootMounted ? "true" : "false") + ",";
  j +=   "\"last_err\":\"" + jsonEscape(String(g_storageStatus.lastError)) + "\",";
  j +=   "\"last_path\":\"" + jsonEscape(String(g_storageStatus.lastPath)) + "\",";
  j +=   "\"fault_count\":" + String(g_storageStatus.faultCount) + ",";
  j +=   "\"recovery_reboot_attempted\":" + String(g_storageStatus.recoveryRebootAttempted ? "true" : "false") + ",";
  j +=   "\"log_source\":\"" + storageLogSource() + "\",";
  j +=   "\"upload_blocked\":" + String(g_storageStatus.uploadBlocked ? "true" : "false");
  j += "}";

  j += ",\"bootdiag\":{";
  j +=   "\"prev_valid\":" + String(g_havePrevBreadcrumb ? "true" : "false");
  if (g_havePrevBreadcrumb) {
    uint8_t prevFragPct = 0;
    if (g_prevBreadcrumb.freeHeap > 0 && g_prevBreadcrumb.largestFreeBlock < g_prevBreadcrumb.freeHeap) {
      prevFragPct = static_cast<uint8_t>(100UL - ((g_prevBreadcrumb.largestFreeBlock * 100UL) / g_prevBreadcrumb.freeHeap));
    }
    j += ",\"prev_uptime_ms\":" + String(g_prevBreadcrumb.uptimeMs);
    j += ",\"prev_heap_free\":" + String(g_prevBreadcrumb.freeHeap);
    j += ",\"prev_heap_min\":" + String(g_prevBreadcrumb.minFreeHeap);
    j += ",\"prev_heap_largest\":" + String(g_prevBreadcrumb.largestFreeBlock);
    j += ",\"prev_heap_frag_pct\":" + String((unsigned)prevFragPct);
    j += ",\"prev_loop_max_block_ms\":" + String(g_prevBreadcrumb.loopMaxBlockMs);
    j += ",\"prev_meas_active\":" + String((g_prevBreadcrumb.flags & (1u << 0)) ? "true" : "false");
    j += ",\"prev_last_net_op\":\"" + jsonEscape(String(g_prevBreadcrumb.lastNetOp)) + "\"";
    j += ",\"prev_last_stage\":\"" + jsonEscape(String(g_prevBreadcrumb.lastStage)) + "\"";
    j += ",\"prev_last_net_err\":\"" + jsonEscape(String(g_prevBreadcrumb.lastNetErr)) + "\"";
    j += ",\"prev_meas_file\":\"" + jsonEscape(String(g_prevBreadcrumb.measFile)) + "\"";
  }
  j += "}";

  // ---- cloud block ----
  j += ",\"cloud\":{";
  j +=   "\"enabled\":" + String(cfg.cloudEnabled?"true":"false") + ",";
  j +=   "\"period\":"  + String(cfg.cloudPeriodS) + ",";
  j +=   "\"upload_on_stop\":" + String(cfg.uploadOnStop?"true":"false") + ",";
  j +=   "\"lastCode\":"+ String(g_lastHttpCode) + ",";
  j +=   "\"lastAt\":\""+ g_lastPushIso + "\",";
  j +=   "\"err\":\""  + jsonEscape(g_lastCloudErr) + "\",";
  j +=   "\"nextSec\":"+ String(g_nextPushInS);
  j += "}";

  j += ",\"command\":{";
  j +=   "\"remoteEnabled\":" + String(cfg.remoteEnabled ? "true" : "false") + ",";
  j +=   "\"lastId\":\"" + jsonEscape(g_lastRemoteCmdId) + "\",";
  j +=   "\"lastResult\":\"" + jsonEscape(g_lastRemoteCmdResult) + "\",";
  j +=   "\"pollAgeMs\":" + String(g_lastRemotePollOkMs ? (millis()-g_lastRemotePollOkMs) : 0);
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
  if (storageIsFaulted()) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"storage_fault\"}");
    return;
  }
  String path = safePath(urlDecodePath(server.arg("path")));
  if (!storageCanUseSd()) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"sd_unavailable\"}");
    return;
  }
  ScopedSpiBusLock lock;
  if (!lock) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"spi_lock_fail\"}");
    return;
  }
  prepareSdSpi();
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
  if (storageIsFaulted()) {
    server.send(503,"text/plain","storage_fault");
    return;
  }
  String path = safePath(urlDecodePath(server.arg("path")));
  if (!storageCanUseSd()) {
    server.send(503,"text/plain","SD unavailable");
    return;
  }
  ScopedSpiBusLock lock;
  if (!lock) {
    server.send(503,"text/plain","SPI lock failed");
    return;
  }
  prepareSdSpi();
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

static bool sendChunkedBytes(const uint8_t* data, size_t len) {
  size_t off = 0;
  while (off < len) {
    size_t n = len - off;
    if (n > 1024) n = 1024;  // keep chunks small for AP responsiveness
    server.sendContent(reinterpret_cast<const char*>(data + off), n);
    WiFiClient c = server.client();
    if (!c.connected()) return false;
    off += n;
    yield();
  }
  return true;
}

static void tarWriteOctal(char* field, size_t fieldLen, uint64_t value) {
  if (fieldLen < 2) return;
  memset(field, 0, fieldLen);
  char digits[32];
  snprintf(digits, sizeof(digits), "%llo", (unsigned long long)value);
  size_t dlen = strlen(digits);
  size_t width = fieldLen - 1; // keep trailing NUL
  if (dlen > width) dlen = width;
  size_t pad = width - dlen;
  memset(field, '0', pad);
  memcpy(field + pad, digits + (strlen(digits) - dlen), dlen);
  field[fieldLen - 1] = '\0';
}

static bool tarWriteHeader(const String& entryName, uint32_t fileSize, bool isDir, String* outReason = nullptr) {
  if (entryName.length() == 0) {
    if (outReason) *outReason = "empty_name";
    return false;
  }
  if (entryName.length() > 255) {
    if (outReason) *outReason = "name_too_long";
    return false;
  }

  uint8_t h[512];
  memset(h, 0, sizeof(h));
  String namePart = entryName;
  String prefixPart = "";
  if (entryName.length() > 100) {
    int cut = entryName.lastIndexOf('/');
    if (cut <= 0) {
      if (outReason) *outReason = "name_split_fail";
      return false;
    }
    prefixPart = entryName.substring(0, cut);
    namePart = entryName.substring(cut + 1);
    if (namePart.length() == 0 || namePart.length() > 100 || prefixPart.length() > 155) {
      if (outReason) *outReason = "name_prefix_limit";
      return false;
    }
  }
  memcpy(h, namePart.c_str(), namePart.length());
  if (prefixPart.length()) memcpy(&h[345], prefixPart.c_str(), prefixPart.length());
  tarWriteOctal((char*)&h[100], 8, isDir ? 0755 : 0644);
  tarWriteOctal((char*)&h[108], 8, 0);
  tarWriteOctal((char*)&h[116], 8, 0);
  tarWriteOctal((char*)&h[124], 12, isDir ? 0 : fileSize);
  time_t nowEpoch = time(nullptr);
  tarWriteOctal((char*)&h[136], 12, isEpochSane(nowEpoch) ? (uint32_t)nowEpoch : 0u);
  memset(&h[148], ' ', 8);
  h[156] = isDir ? '5' : '0';
  memcpy(&h[257], "ustar", 5);
  h[262] = '\0';
  h[263] = '0';
  h[264] = '0';

  uint32_t csum = 0;
  for (size_t i = 0; i < sizeof(h); ++i) csum += h[i];
  char chk[8];
  memset(chk, 0, sizeof(chk));
  snprintf(chk, sizeof(chk), "%06o", (unsigned)csum);
  memcpy(&h[148], chk, 6);
  h[154] = '\0';
  h[155] = ' ';

  if (!sendChunkedBytes(h, sizeof(h))) {
    if (outReason) *outReason = "header_write_fail";
    return false;
  }
  return true;
}

static bool tarStreamPathRecursive(const String& absPath, const String& relPath, uint16_t& fileCount, String& outErr) {
  digitalWrite(WIZ_CS, HIGH);
  File f = SD.open(absPath, FILE_READ);
  if (!f) {
    outErr = "open_fail:" + absPath;
    return false;
  }

  if (f.isDirectory()) {
    String dirEntry = relPath;
    if (!dirEntry.endsWith("/")) dirEntry += "/";
    String hdrErr;
    if (!tarWriteHeader(dirEntry, 0, true, &hdrErr)) {
      outErr = "tar_header_fail_dir:" + relPath + ":" + hdrErr;
      f.close();
      return false;
    }

    File child = f.openNextFile();
    while (child) {
      String childName = baseName(String(child.name()));
      child.close();
      if (childName.length() == 0 || childName == "." || childName == "..") {
        child = f.openNextFile();
        continue;
      }

      String childAbs = absPath;
      if (childAbs != "/") childAbs += "/";
      childAbs += childName;

      String childRel = relPath;
      if (childRel.length()) childRel += "/";
      childRel += childName;

      if (!tarStreamPathRecursive(childAbs, childRel, fileCount, outErr)) {
        f.close();
        return false;
      }
      child = f.openNextFile();
    }
    f.close();
    return true;
  }

  uint32_t sz = (uint32_t)f.size();
  String hdrErr;
  if (!tarWriteHeader(relPath, sz, false, &hdrErr)) {
    outErr = "tar_header_fail_file:" + relPath + ":" + hdrErr;
    f.close();
    return false;
  }

  while (f.available()) {
    int n = f.read(g_csvFrameBuf, sizeof(g_csvFrameBuf));
    if (n < 0) {
      outErr = "read_fail:" + relPath;
      f.close();
      return false;
    }
    if (n == 0) break;
    if (!sendChunkedBytes(g_csvFrameBuf, (size_t)n)) {
      outErr = "socket_write_fail:" + relPath;
      f.close();
      return false;
    }
  }
  f.close();

  uint32_t pad = (512u - (sz % 512u)) % 512u;
  if (pad > 0) {
    static uint8_t zeros[512] = {0};
    if (!sendChunkedBytes(zeros, pad)) {
      outErr = "socket_pad_fail:" + relPath;
      return false;
    }
  }

  ++fileCount;
  return true;
}

void handleDownloadBundle(){
  if (storageIsFaulted()) {
    server.send(503,"text/plain","storage_fault");
    return;
  }
  String path = safePath(urlDecodePath(server.arg("path")));
  if (!storageCanUseSd()) {
    server.send(503,"text/plain","SD unavailable");
    return;
  }
  ScopedSpiBusLock lock;
  if (!lock) {
    server.send(503,"text/plain","SPI lock failed");
    return;
  }
  prepareSdSpi();
  File dir = SD.open(path, FILE_READ);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    server.send(404,"text/plain","Not a folder");
    return;
  }
  dir.close();

  String root = baseName(path);
  if (root.length() == 0) root = "sd";
  if (root.length() > 80) root = root.substring(0, 80);

  server.sendHeader("Cache-Control","no-store");
  server.sendHeader("Content-Disposition","attachment; filename=\"" + root + ".tar\"");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"application/x-tar","");

  uint16_t fileCount = 0;
  String err;
  bool ok = tarStreamPathRecursive(path, root, fileCount, err);
  if (ok) {
    static uint8_t zeros[512] = {0};
    ok = sendChunkedBytes(zeros, sizeof(zeros)) && sendChunkedBytes(zeros, sizeof(zeros));
  }
  server.sendContent(""); // finish chunked response
  logLine(String("[DLBUNDLE] ") + (ok ? "OK " : "FAIL ")
          + "path=" + path
          + " files=" + String((unsigned)fileCount)
          + (err.length() ? (" err=" + err) : ""));
}

bool deleteRecursive(const String& path) {
  if (!storageCanUseSd()) return false;
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
  if (storageIsFaulted()) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"storage_fault\"}");
    return;
  }
  String path = safePath(urlDecodePath(server.arg("path")));
  ScopedSpiBusLock lock;
  if (!lock) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"spi_lock_fail\"}");
    return;
  }
  prepareSdSpi();
  bool ok = deleteRecursive(path);
  if (!ok && path.indexOf(' ')>=0) { // try '+' variant
    String alt = path; alt.replace(" ","+");
    ok = deleteRecursive(alt);
  }
  server.send(200,"application/json", ok?"{\"ok\":true}":"{\"ok\":false}");
}
void handleMkdir(){
  if (storageIsFaulted()) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"storage_fault\"}");
    return;
  }
  String path = safePath(urlDecodePath(server.arg("path")));
  if (!storageCanUseSd()) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"sd_unavailable\"}");
    return;
  }
  ScopedSpiBusLock lock;
  if (!lock) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"spi_lock_fail\"}");
    return;
  }
  prepareSdSpi();
  bool ok = SD.mkdir(path);
  if (!ok) storageRecordFailure(path, "mkdir_fail");
  else storageRecordSuccess();
  server.send(200,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}");
}
File _uploadFile;
static bool g_uploadAuthorized = false;
void handleUploadPost() {
  HTTPUpload& up = server.upload();
  if (portalMeasurementLockActive()) {
    if (up.status == UPLOAD_FILE_START) {
      sendPortalMeasurementLockedJson();
    }
    return;
  }
  if (storageIsFaulted()) {
    if (up.status == UPLOAD_FILE_END || up.status == UPLOAD_FILE_ABORTED) {
      server.send(503,"application/json","{\"ok\":false,\"err\":\"storage_fault\"}");
    }
    return;
  }
  if (up.status == UPLOAD_FILE_START) {
    g_uploadAuthorized = isAuthorizedRequest();
    if (!g_uploadAuthorized) return;
    String dir = safePath(urlDecode(server.arg("dir"))); if (dir.length()==0) dir="/";
    String base = up.filename; base.replace("\\","/"); int pos=base.lastIndexOf('/'); if (pos>=0) base=base.substring(pos+1);
    ScopedSpiBusLock lock;
    if (!lock) {
      g_uploadAuthorized = false;
      server.send(503,"application/json","{\"ok\":false,\"err\":\"spi_lock_fail\"}");
      return;
    }
    prepareSdSpi();
    if (dir != "/") SD.mkdir(dir);
    String path = dir + (dir=="/"?"":"/") + (base.length()?base:"upload.bin");
    _uploadFile = SD.open(path, FILE_WRITE);
    if (!_uploadFile) {
      storageRecordFailure(path, "open_fail");
      g_uploadAuthorized = false;
      server.send(503,"application/json","{\"ok\":false,\"err\":\"open_fail\"}");
      return;
    }
  } else if (!g_uploadAuthorized) {
    if (up.status == UPLOAD_FILE_END || up.status == UPLOAD_FILE_ABORTED) {
      server.send(401,"application/json","{\"ok\":false,\"err\":\"unauthorized\"}");
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (_uploadFile) {
      ScopedSpiBusLock lock;
      if (!lock) return;
      prepareSdSpi();
      size_t wrote = _uploadFile.write(up.buf, up.currentSize);
      if (wrote != up.currentSize) {
        storageRecordFailure(String(_uploadFile.name()), "write_short");
      } else {
        storageRecordSuccess();
      }
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (_uploadFile) {
      ScopedSpiBusLock lock;
      if (lock) {
        prepareSdSpi();
        _uploadFile.close();
        storageRecordSuccess();
      }
    }
    server.send(200,"application/json","{\"ok\":true}");
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (_uploadFile) {
      ScopedSpiBusLock lock;
      if (lock) {
        prepareSdSpi();
        String n=_uploadFile.name();
        _uploadFile.close();
        SD.remove(n);
      }
    }
    server.send(200,"application/json","{\"ok\":false}");
  }
}

// ---- ADS ----
// GET /ads — return current readings and config
void handleAdsGet(){
  server.sendHeader("Cache-Control","no-store");
  const bool ready = adsReady;

  // Read all channels if ADS is online
  uint8_t chans[NUM_SENSORS] = {0,1,2,3};
  size_t chanCount = ready ? NUM_SENSORS : 0;

  String j = "{";
  j += "\"ok\":true,\"ready\":"; j += (ready ? "true," : "false,");
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
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += "\"" + gainToStr(g_gainCh[ch]) + "\""; }
  j +=   "],\"rate\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_rateCh[ch]); }
  j +=   "],\"shunt\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_shuntCh[ch],3); }
  j +=   "],\"active\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += (g_adsActive[ch]?"true":"false"); }
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

  j += "}";

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

// POST /adsconf — set gain/rate/shunt/active mask
void handleAdsConf(){
  server.sendHeader("Cache-Control","no-store");

  if (g_measActive) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"measuring\"}");
    return;
  }

  bool touchedElec = false;
  bool touchedUnits = false;
  bool touchedActive = false;

  auto parseBool = [](const String& v)->int{
    String t=v; t.trim(); t.toLowerCase();
    if (t=="1"||t=="true"||t=="yes"||t=="on") return 1;
    if (t=="0"||t=="false"||t=="no"||t=="off") return 0;
    return -1;
  };

  if (server.hasArg("active")) {
    String list = server.arg("active");
    uint8_t idx = 0; int start = 0;
    while (idx < NUM_SENSORS && start < list.length()) {
      int comma = list.indexOf(',', start);
      String token = (comma >= 0) ? list.substring(start, comma) : list.substring(start);
      int val = parseBool(token);
      if (val >= 0) { g_adsActive[idx] = (val != 0); touchedActive = true; }
      idx++;
      if (comma < 0) break; else { start = comma + 1; }
    }
  }
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    String k = String("active") + ch;
    if (server.hasArg(k)) {
      int v = parseBool(server.arg(k));
      if (v >= 0) { g_adsActive[ch] = (v != 0); touchedActive = true; }
    }
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

  if (touchedActive) {
    uint32_t nowMs = millis();
    for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
      if (!g_adsActive[ch]) {
        g_alarmCh[ch] = ALARM_NORMAL;
        g_alarmLastChangeMs[ch] = nowMs;
        g_alarmPendingState[ch] = ALARM_NORMAL;
        g_alarmPendingSinceMs[ch] = 0;
        g_alarmFilterInit[ch] = false;
        g_alarmFiltered[ch] = 0.0f;
      }
    }
    updateAlarmGPIO();
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
  if (touchedActive){
    logMsg += havePart ? " | " : ": ";
    logMsg += "active=0b" + String([&](){ uint8_t m=0; for(uint8_t i=0;i<NUM_SENSORS;++i) if(g_adsActive[i]) m|=(1u<<i); return m; }(), BIN);
    havePart = true;
  }
  if (!havePart) logMsg += ": no changes";
  logLine(logMsg);

  String j="{";
  j += "\"ok\":true,";
  j += "\"cfg\":{\"gain\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += "\""+gainToStr(g_gainCh[ch])+"\""; }
  j += "],\"rate\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_rateCh[ch]); }
  j += "],\"shunt\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += String(g_shuntCh[ch],3); }
  j += "],\"active\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j+=","; j += (g_adsActive[ch]?"true":"false"); }
  j += "]}";
  j += "}";
  server.send(200,"application/json",j);
}


// Parse https://host[:port]/path  → scheme, host, port, path
static bool parseUrl(const String& url, String& scheme, String& host, uint16_t& port, String& path){
  scheme = host = path = ""; port = 0;
  String u = url;
  u.trim();
  int p = u.indexOf("://"); if (p < 0) return false;
  scheme = u.substring(0,p); String rest = u.substring(p+3);
  scheme.toLowerCase();
  rest.trim();
  int slash = rest.indexOf('/'); String hostport = (slash<0)?rest:rest.substring(0,slash);
  path = (slash<0)?"/":rest.substring(slash);
  hostport.trim();
  int colon = hostport.indexOf(':');
  if (colon>=0){ host = hostport.substring(0,colon); port = (uint16_t)hostport.substring(colon+1).toInt(); }
  else { host = hostport; port = 0; }
  host.trim();
  if (port==0) port = (scheme=="https")?443:80;
  return host.length() > 0;
}

static String urlEncode(const String& in) {
  static const char kHexDigits[] = "0123456789ABCDEF";
  String out;
  out.reserve(in.length() * 3);
  for (size_t i = 0; i < in.length(); ++i) {
    uint8_t c = static_cast<uint8_t>(in[i]);
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
      out += static_cast<char>(c);
    } else {
      out += '%';
      out += kHexDigits[(c >> 4) & 0x0F];
      out += kHexDigits[c & 0x0F];
    }
  }
  return out;
}

static String cloudApiPrefixFromServerUrl() {
  String scheme, host, path;
  uint16_t port = 0;
  if (!parseUrl(cfg.serverUrl, scheme, host, port, path)) return "";
  if (scheme != "https" || host.length() == 0) return "";
  bool defaultPort = (port == 443);
  String origin = scheme + "://" + host + (defaultPort ? "" : (String(":") + String(port)));

  int q = path.indexOf('?');
  if (q >= 0) path = path.substring(0, q);
  int hash = path.indexOf('#');
  if (hash >= 0) path = path.substring(0, hash);
  if (!path.startsWith("/")) path = "/" + path;
  while (path.length() > 1 && path.endsWith("/")) path.remove(path.length() - 1);

  // Accept serverUrl as:
  //   https://host
  //   https://host/remote
  //   https://host/remote/
  //   https://host/remote/index.php
  //   https://host/remote/ingest
  // and always normalize to the API base prefix.
  if (path.endsWith("/index.php")) path = path.substring(0, path.length() - 10);
  if (path.endsWith("/ingest")) path = path.substring(0, path.length() - 7);
  while (path.length() > 1 && path.endsWith("/")) path.remove(path.length() - 1);

  if (path == "/") path = "";
  return origin + path;
}

static String cloudIngestUrlFromServerUrl() {
  String apiPrefix = cloudApiPrefixFromServerUrl();
  if (apiPrefix.length() == 0) return "";
  String url = apiPrefix + "/ingest";
  if (cfg.deviceId.length()) {
    url += "?device_id=" + urlEncode(cfg.deviceId);
  }
  return url;
}

static IPAddress effectiveDnsServer() {
  IPAddress dnsIP = ethDnsIpSafe();
  if (dnsIP == IPAddress(0,0,0,0)) dnsIP = cfg.dns;
  if (dnsIP == IPAddress(0,0,0,0)) dnsIP = IPAddress(1,1,1,1);
  return dnsIP;
}

static void ensureDnsServerForTls() {
  if (ethDnsIpSafe() != IPAddress(0,0,0,0)) return;
  IPAddress fallback = effectiveDnsServer();
  ScopedSpiBusLock lock;
  if (!lock) return;
  prepareEthernetSpi();
  Ethernet.setDnsServerIP(fallback);
  logLine(String("[DNS] fallback DNS for TLS: ") + fallback.toString());
}

static inline bool localPortalClientConnected() {
  if (!cfg.commissioningMode) return false;
  return WiFi.softAPgetStationNum() > 0;
}

static inline bool shouldPrioritizeLocalPortal() {
  return localPortalClientConnected();
}

static void serviceLocalPortal() {
  if (!cfg.commissioningMode) return;
  noteRuntimeStage("loop_web");
  if (!measurementNetworkDeferralActive() && !portalMeasurementLockActive()) {
    dns.processNextRequest();
  }
  server.handleClient();
  WiFiClient client = server.client();
  if (client && client.connected()) {
    g_lastPortalHttpActivityMs = millis();
  }
}

static bool localPortalInteractiveActive() {
  if (!localPortalClientConnected()) return false;
  WiFiClient client = server.client();
  if (client && client.connected()) return true;
  if (g_lastPortalHttpActivityMs == 0) return false;
  return (millis() - g_lastPortalHttpActivityMs) < LOCAL_PORTAL_ACTIVITY_GRACE_MS;
}

static void resetTlsBaseClient() {
  if (_tcp.connected()) _tcp.stop();
  _tcp = EthernetClient();
  _tcp.setTimeout(2500);
}

static bool tlsConnectHost(const String& host, uint16_t port, String& outErr) {
  ScopedSpiBusLock lock;
  if (!lock) {
    outErr = "spi_lock_fail";
    return false;
  }
  prepareEthernetSpi();
  outErr = "";
  if (!host.length()) {
    outErr = "empty_host";
    return false;
  }

  ensureDnsServerForTls();
  resetTlsBaseClient();
  _tls.setTimeout(localPortalClientConnected() ? 3000 : 8000);

  if (_tls.connected()) _tls.stop();
  tlsPrepare(host);
  if (_tls.connect(host.c_str(), port)) return true;

  int sslErr = _tls.getWriteError();

  IPAddress hostIP;
  bool dnsOk = resolveHost(host.c_str(), hostIP);
  bool tcpOk = false;
  if (dnsOk) {
    EthernetClient c;
    c.setTimeout(1500);
    tcpOk = c.connect(hostIP, port);
    c.stop();
  }

  // If DNS and raw TCP both work, retry TLS once.
  if (dnsOk && tcpOk) {
    delay(40);
    if (_tls.connected()) _tls.stop();
    resetTlsBaseClient();
    tlsPrepare(host);
    if (_tls.connect(host.c_str(), port)) return true;
    sslErr = _tls.getWriteError();
  }

  outErr = "connect_fail host=" + host +
           " port=" + String(port) +
           " dns=" + Ethernet.dnsServerIP().toString() +
           " dns_ok=" + String(dnsOk ? "1" : "0") +
           " ssl_err=" + String(sslErr);
  if (dnsOk) {
    outErr += " ip=" + hostIP.toString();
    outErr += " tcp=" + String(tcpOk ? "1" : "0");
  }
  return false;
}

static String remoteCommandsUrl() {
  String apiPrefix = cloudApiPrefixFromServerUrl();
  if (apiPrefix.length() == 0) return "";
  String url = apiPrefix + "/api/v1/devices/" + urlEncode(cfg.deviceId) + "/commands?limit=1";
  if (cfg.lastCommandId.length()) {
    url += "&after_id=" + urlEncode(cfg.lastCommandId);
  }
  return url;
}

static String remoteAckUrl(const String& cmdId) {
  String apiPrefix = cloudApiPrefixFromServerUrl();
  if (apiPrefix.length() == 0) return "";
  return apiPrefix + "/api/v1/devices/" + urlEncode(cfg.deviceId) + "/commands/" + urlEncode(cmdId) + "/ack";
}

static bool jsonGetStringField(const String& json, const char* key, String& out) {
  out = "";
  String pat = "\"" + String(key) + "\"";
  int p = json.indexOf(pat);
  if (p < 0) return false;
  p = json.indexOf(':', p + pat.length());
  if (p < 0) return false;
  ++p;
  while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\t' || json[p] == '\r' || json[p] == '\n')) ++p;
  if (p >= (int)json.length()) return false;
  if (json[p] != '"') return false;
  ++p;
  while (p < (int)json.length()) {
    char c = json[p++];
    if (c == '\\' && p < (int)json.length()) {
      char esc = json[p++];
      switch (esc) {
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        case '/': out += '/'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'n': out += '\n'; break;
        case 'r': out += '\r'; break;
        case 't': out += '\t'; break;
        default: out += esc; break;
      }
      continue;
    }
    if (c == '"') return true;
    out += c;
  }
  return false;
}

static bool parseRemoteCommand(const String& json, RemoteCommand& cmd) {
  cmd = RemoteCommand{};
  bool ok = true;
  ok &= jsonGetStringField(json, "id", cmd.id);
  ok &= jsonGetStringField(json, "action", cmd.action);
  ok &= jsonGetStringField(json, "issued_at", cmd.issuedAt);
  ok &= jsonGetStringField(json, "nonce", cmd.nonce);
  ok &= jsonGetStringField(json, "sig", cmd.sig);
  String tmp;
  if (jsonGetStringField(json, "params", tmp)) cmd.params = tmp;
  if (!ok) return false;
  cmd.sig.trim();
  cmd.sig.toLowerCase();
  return cmd.id.length() && cmd.action.length() && cmd.sig.length();
}

static String toHexLower(const uint8_t* data, size_t len) {
  static const char kHexDigits[] = "0123456789abcdef";
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out += kHexDigits[(data[i] >> 4) & 0x0F];
    out += kHexDigits[data[i] & 0x0F];
  }
  return out;
}

static String canonicalCommandPayload(const RemoteCommand& cmd) {
  String s = "{";
  s += "\"id\":\"" + jsonEscape(cmd.id) + "\",";
  s += "\"action\":\"" + jsonEscape(cmd.action) + "\",";
  s += "\"params\":\"" + jsonEscape(cmd.params) + "\",";
  s += "\"issued_at\":\"" + jsonEscape(cmd.issuedAt) + "\",";
  s += "\"nonce\":\"" + jsonEscape(cmd.nonce) + "\"";
  s += "}";
  return s;
}

static bool hmacSha256Hex(const String& key, const String& msg, String& outHex) {
  outHex = "";
  const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!md) return false;
  uint8_t digest[32];
  int rc = mbedtls_md_hmac(md,
                           reinterpret_cast<const unsigned char*>(key.c_str()), key.length(),
                           reinterpret_cast<const unsigned char*>(msg.c_str()), msg.length(),
                           digest);
  if (rc != 0) return false;
  outHex = toHexLower(digest, sizeof(digest));
  return true;
}

static bool verifyCommandSignature(const RemoteCommand& cmd) {
  if (cfg.cmdSecret.length() == 0) return false;
  String canonical = canonicalCommandPayload(cmd);
  String expected;
  if (!hmacSha256Hex(cfg.cmdSecret, canonical, expected)) return false;
  return expected == cmd.sig;
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
  String j;
  j.reserve(withReadings ? 3800 : 2900);

  uint8_t activeMask = 0;
  uint8_t activeCount = 0;
  bool anyAlarm = false;
  for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
    if (g_adsActive[ch]) {
      activeMask |= (uint8_t(1u) << ch);
      ++activeCount;
    }
    if (g_alarmCh[ch] != ALARM_NORMAL) anyAlarm = true;
  }

  bool includeReadings = withReadings && adsReady;
  bool useCachedReadings = includeReadings && g_measActive;
  String mode = adsReady ? ((activeCount == NUM_SENSORS) ? "all" : ((activeCount > 1) ? "multi" : (activeCount == 1 ? "single" : "none"))) : "none";
  uint32_t freeHeap = runtimeFreeHeap();
  uint32_t minFreeHeap = runtimeMinFreeHeap();
  uint32_t largestFreeBlock = runtimeLargestFreeBlock();
  uint8_t heapFragPct = runtimeHeapFragPct();
  uint32_t loopAgeMs = g_loopLastMs ? (millis() - g_loopLastMs) : 0;

  j += "{";
  j += "\"dev\":\""+jsonEscape(cfg.devName)+"\",";
  j += "\"device_id\":\""+jsonEscape(cfg.deviceId)+"\",";
  j += "\"ip\":\""+ethLocalIpSafe().toString()+"\",";
  j += "\"time\":\""+isoNow()+"\",";
  j += "\"uptime\":\""+uptimeStr()+"\",";
  j += "\"inet\":" + String(g_internetOk?"true":"false") + ",";
  j += "\"net\":{";
  j +=   "\"link_ok\":" + String(g_linkOk ? "true" : "false") + ",";
  j +=   "\"dns_ok\":" + String(g_dnsOk ? "true" : "false") + ",";
  j +=   "\"internet_ok\":" + String(g_internetOk ? "true" : "false") + ",";
  j +=   "\"cloud_ok\":" + String(g_cloudOk ? "true" : "false");
  j += "},";
  j += "\"clock\":{";
  j +=   "\"synced\":" + String(g_timeSynced ? "true" : "false") + ",";
  j +=   "\"sync_age_ms\":" + String(g_lastTimeSyncMs ? (millis() - g_lastTimeSyncMs) : 0);
  j += "},";
  j += "\"storage\":{";
  j +=   "\"state\":\"" + jsonEscape(String(storageStateName())) + "\",";
  j +=   "\"fault\":" + String(storageIsFaulted() ? "true" : "false") + ",";
  j +=   "\"degraded\":" + String(storageIsDegraded() ? "true" : "false") + ",";
  j +=   "\"boot_mounted\":" + String(g_sdBootMounted ? "true" : "false") + ",";
  j +=   "\"last_err\":\"" + jsonEscape(String(g_storageStatus.lastError)) + "\",";
  j +=   "\"last_path\":\"" + jsonEscape(String(g_storageStatus.lastPath)) + "\",";
  j +=   "\"fault_count\":" + String(g_storageStatus.faultCount) + ",";
  j +=   "\"recovery_reboot_attempted\":" + String(g_storageStatus.recoveryRebootAttempted ? "true" : "false") + ",";
  j +=   "\"log_source\":\"" + storageLogSource() + "\",";
  j +=   "\"upload_blocked\":" + String(g_storageStatus.uploadBlocked ? "true" : "false");
  j += "},";
  j += "\"runtime\":{";
  j +=   "\"heap_free\":" + String(freeHeap) + ",";
  j +=   "\"heap_min\":" + String(minFreeHeap) + ",";
  j +=   "\"heap_largest\":" + String(largestFreeBlock) + ",";
  j +=   "\"heap_frag_pct\":" + String((unsigned)heapFragPct) + ",";
  j +=   "\"task_count\":" + String((unsigned)g_runtimeCpu.taskCount) + ",";
  j +=   "\"cpu_busy_pct\":";
  j +=   g_runtimeCpu.valid ? String((unsigned)g_runtimeCpu.cpuBusyPct) : String("null");
  j +=   ",";
  j +=   "\"loop_cpu_pct\":";
  j +=   g_runtimeCpu.valid ? String((unsigned)g_runtimeCpu.loopCpuPct) : String("null");
  j +=   ",";
  j +=   "\"meas_cpu_pct\":";
  j +=   g_runtimeCpu.valid ? String((unsigned)g_runtimeCpu.measCpuPct) : String("null");
  j +=   ",";
  j +=   "\"loop_stack_free\":" + String(stackWatermarkBytes(g_loopTaskHandle)) + ",";
  j +=   "\"meas_stack_free\":" + String(stackWatermarkBytes(g_measTask)) + ",";
  j +=   "\"loop_age_ms\":" + String(loopAgeMs) + ",";
  j +=   "\"loop_max_block_ms\":" + String(g_loopMaxBlockMs) + ",";
  j +=   "\"last_stage\":\"" + jsonEscape(String(g_lastRuntimeStage)) + "\",";
  j +=   "\"last_stage_age_ms\":" + String(g_lastRuntimeStageMs ? (millis() - g_lastRuntimeStageMs) : 0) + ",";
  j +=   "\"last_net_op\":\"" + jsonEscape(g_lastNetOp) + "\",";
  j +=   "\"last_net_err\":\"" + jsonEscape(g_lastNetErr) + "\",";
  j +=   "\"last_net_err_age_ms\":" + String(g_lastNetErrMs ? (millis() - g_lastNetErrMs) : 0);
  j += "},";
  j += "\"bootdiag\":{";
  j +=   "\"prev_valid\":" + String(g_havePrevBreadcrumb ? "true" : "false");
  if (g_havePrevBreadcrumb) {
    uint8_t prevFragPct = 0;
    if (g_prevBreadcrumb.freeHeap > 0 && g_prevBreadcrumb.largestFreeBlock < g_prevBreadcrumb.freeHeap) {
      prevFragPct = static_cast<uint8_t>(100UL - ((g_prevBreadcrumb.largestFreeBlock * 100UL) / g_prevBreadcrumb.freeHeap));
    }
    j += ",\"prev_uptime_ms\":" + String(g_prevBreadcrumb.uptimeMs);
    j += ",\"prev_heap_free\":" + String(g_prevBreadcrumb.freeHeap);
    j += ",\"prev_heap_min\":" + String(g_prevBreadcrumb.minFreeHeap);
    j += ",\"prev_heap_largest\":" + String(g_prevBreadcrumb.largestFreeBlock);
    j += ",\"prev_heap_frag_pct\":" + String((unsigned)prevFragPct);
    j += ",\"prev_loop_max_block_ms\":" + String(g_prevBreadcrumb.loopMaxBlockMs);
    j += ",\"prev_meas_active\":" + String((g_prevBreadcrumb.flags & (1u << 0)) ? "true" : "false");
    j += ",\"prev_last_net_op\":\"" + jsonEscape(String(g_prevBreadcrumb.lastNetOp)) + "\"";
    j += ",\"prev_last_stage\":\"" + jsonEscape(String(g_prevBreadcrumb.lastStage)) + "\"";
    j += ",\"prev_last_net_err\":\"" + jsonEscape(String(g_prevBreadcrumb.lastNetErr)) + "\"";
    j += ",\"prev_meas_file\":\"" + jsonEscape(String(g_prevBreadcrumb.measFile)) + "\"";
  }
  j += "},";

  j += "\"ads\":{";
  j += "\"ready\":" + String(adsReady ? "true" : "false") + ",";
  j += "\"channels\":" + String((unsigned)NUM_SENSORS) + ",";
  j += "\"mode\":\"" + mode + "\",";
  j += "\"active_mask\":" + String((unsigned)activeMask) + ",";
  j += "\"active_channels\":" + String((unsigned)activeCount) + ",";
  j += "\"cfg\":{";
  j += "\"gain\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += "\"" + gainToStr(g_gainCh[ch]) + "\""; }
  j += "],\"rate\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += String(g_rateCh[ch]); }
  j += "],\"shunt\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += String(g_shuntCh[ch],3); }
  j += "],\"active\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += (g_adsActive[ch] ? "true" : "false"); }
  j += "],\"fsmm\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += String(g_engFSmm[ch],1); }
  j += "],\"offmm\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += String(g_engOffmm[ch],3); }
  j += "]},"; // cfg
  j += "\"alarms\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    if (ch) j += ",";
    j += "\"" + jsonEscape(String(alarmStr(g_alarmCh[ch]))) + "\"";
  }
  j += "],\"alarm_any\":" + String(anyAlarm ? "true" : "false") + ",";
  j += "\"readings_source\":\"";
  if (!includeReadings) j += "none";
  else if (useCachedReadings) j += "cached";
  else j += "live";
  j += "\",\"readings\":[";
  if (includeReadings) {
    for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
      int16_t raw = 0;
      float mv = 0.0f, ma = 0.0f, pct = 0.0f;
      bool rawValid = false;
      if (g_adsActive[ch]) {
        if (useCachedReadings) {
          mv = g_lastMv[ch];
          ma = g_lastmA[ch];
          pct = g_lastPct[ch];
        } else {
          adsReadCh(ch, raw, mv, ma, pct);
          rawValid = true;
        }
      }
      float mm = mapToMM(ch, pct);
      if (ch) j += ",";
      j += "{\"ch\":" + String(ch) + ",\"active\":" + String(g_adsActive[ch] ? "true" : "false") + ",";
      if (rawValid) j += "\"raw\":" + String(raw) + ",";
      else j += "\"raw\":null,";
      j += "\"mv\":" + String(mv,3) + ",";
      j += "\"ma\":" + String(ma,3) + ",";
      j += "\"pct\":" + String(pct,1) + ",";
      j += "\"mm\":" + String(mm,2) + "}";
    }
  }
  j += "]";
  j += "},"; // ads

  j += "\"meas\":{";
  j += "\"active\":" + String(g_measActive ? "true" : "false") + ",";
  j += "\"id\":\"" + jsonEscape(g_measId) + "\",";
  j += "\"file\":\"" + jsonEscape(g_measFile) + "\",";
  j += "\"frames\":" + String((unsigned)g_frameCount) + ",";
  j += "\"bytes\":" + String((unsigned long long)g_measBytes) + ",";
  j += "\"active_mask\":" + String((unsigned)activeMask) + ",";
  j += "\"active_channels\":" + String((unsigned)activeCount) + ",";
  j += "\"sps\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += String(g_measSps[ch]); }
  j += "],\"dt_ms\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += String(g_measDtMs[ch],3); }
  j += "]}";
  j += "}";
  return j;
}

void handleAdsDump(){
  server.sendHeader("Cache-Control","no-store");
  String j = "{";
  j += "\"gain\":\"" + gainToStr(g_adsGain) + "\",";
  j += "\"rate\":" + String(g_adsRateSps) + ",";
  j += "\"shunt\":" + String(g_shuntOhms,3) + ",";
  j += "\"active\":[";
    for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { 
      if (ch) j += ","; j += (g_adsActive[ch]?"true":"false"); 
    }
  j += "],";
  j += "\"fsmm\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += String(g_engFSmm[ch],1); }
  j += "],";
  j += "\"offmm\":[";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) j += ","; j += String(g_engOffmm[ch],3); }
  j += "]";
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
  noteRuntimeStage("tls_status");
  while (tls.connected() && millis()-start < firstByteTimeoutMs) {
    if (tls.available()) { line = tls.readStringUntil('\n'); break; }
    delay(1);
  }
  if (!line.startsWith("HTTP/1.1 ")) return false;
  code = line.substring(9, 12).toInt();

  // headers
  noteRuntimeStage("tls_headers");
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

static bool readBody(SSLClient& tls, String& body, uint32_t idleTimeoutMs=1500, size_t maxBytes=6144) {
  body = "";
  uint32_t lastRx = millis();
  noteRuntimeStage("tls_body");
  while (tls.connected()) {
    while (tls.connected() && tls.available()) {
      int b = tls.read();
      if (b < 0) break;
      if (body.length() < (int)maxBytes) body += char(b);
      lastRx = millis();
    }
    if (millis() - lastRx > idleTimeoutMs) break;
    delay(1);
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
  ScopedSpiBusLock lock;
  if (!lock) {
    outCode = -1;
    outErr = "spi_lock_fail";
    return false;
  }
  prepareEthernetSpi();
  outCode=-1; outResp=""; outErr="";
  String nextUrl = urlIn;

  for (int hops=0; hops<3; ++hops) {                 // follow up to 3 redirects
    String scheme, host, path; uint16_t port;
    if (!parseUrl(nextUrl, scheme, host, port, path) || scheme!="https") { outErr="bad url"; return false; }

    String connErr;
    if (!tlsConnectHost(host, port, connErr)) { outErr = connErr; return false; }

    String req;
    req.reserve(256 + body.length());
    req += "POST " + path + " HTTP/1.1\r\n";
    req += "Host: " + host + "\r\n";
    req += "User-Agent: ESP32-W5500\r\n";
    req += "Content-Type: application/json\r\n";
    req += "X-DEV: " + cfg.devName + "\r\n";
    if (cfg.deviceId.length()) req += "X-DEVICE-ID: " + cfg.deviceId + "\r\n";
    if (bearer.length()) req += "X-API-KEY: " + bearer + "\r\n";
    req += "Content-Length: " + String(body.length()) + "\r\n";
    req += "Connection: close\r\n\r\n";

    _tls.print(req);
    _tls.print(body);

    String loc;
    if (!readStatusAndHeaders(_tls, outCode, loc)) { outErr="no status"; _tls.stop(); return false; }

    readBody(_tls, outResp);
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

static bool httpsGetText(const String& urlIn, const String& bearer,
                         int& outCode, String& outResp, String& outErr)
{
  ScopedSpiBusLock lock;
  if (!lock) {
    outCode = -1;
    outErr = "spi_lock_fail";
    return false;
  }
  prepareEthernetSpi();
  outCode=-1; outResp=""; outErr="";
  String nextUrl = urlIn;

  for (int hops=0; hops<3; ++hops) {
    String scheme, host, path; uint16_t port;
    if (!parseUrl(nextUrl, scheme, host, port, path) || scheme!="https") { outErr="bad url"; return false; }

    String connErr;
    if (!tlsConnectHost(host, port, connErr)) { outErr = connErr; return false; }

    String req;
    req.reserve(256);
    req += "GET " + path + " HTTP/1.1\r\n";
    req += "Host: " + host + "\r\n";
    req += "User-Agent: ESP32-W5500\r\n";
    req += "X-DEV: " + cfg.devName + "\r\n";
    if (bearer.length()) req += "X-API-KEY: " + bearer + "\r\n";
    req += "Connection: close\r\n\r\n";
    _tls.print(req);

    String loc;
    if (!readStatusAndHeaders(_tls, outCode, loc)) { outErr="no status"; _tls.stop(); return false; }
    readBody(_tls, outResp);
    _tls.stop();

    if (isRedirect(outCode)) {
      if (loc.length()==0) { outErr="redirect w/o Location"; return false; }
      nextUrl = absolutizeLocation(nextUrl, loc);
      continue;
    }
    return (outCode>=200 && outCode<300) || outCode==204;
  }

  outErr="too many redirects";
  return false;
}

static String cloudUploadNameForPath(const String& filePath) {
  String leaf = baseName(filePath);
  if (leaf.length() == 0) leaf = "upload.bin";

  if (g_measDir.length()) {
    String dirPrefix = g_measDir;
    if (!dirPrefix.endsWith("/")) dirPrefix += "/";
    if (filePath.startsWith(dirPrefix)) {
      String sess = baseName(g_measDir);
      if (sess.length()) {
        return sess + "/" + leaf;
      }
    }
  }

  return leaf;
}

// Upload a file as raw octet-stream to <url>?upload=1&name=<relative path>
static bool httpsUploadFile(const String& urlIn, const String& bearer, const String& filePath,
                            int& outCode, String& outResp, String& outErr)
{
  ScopedSpiBusLock lock;
  if (!lock) {
    outCode = -1;
    outErr = "spi_lock_fail";
    return false;
  }
  prepareSdSpi();
  outCode=-1; outResp=""; outErr="";
  if (storageIsFaulted()) {
    outErr = "storage_fault";
    return false;
  }
  if (!SD.exists(filePath)) { outErr="no file"; return false; }

  String nextUrl = urlIn;
  for (int hops=0; hops<3; ++hops) {
    File f = SD.open(filePath, FILE_READ); if (!f) { outErr="open fail"; return false; }

    String scheme, host, path; uint16_t port;
    if (!parseUrl(nextUrl, scheme, host, port, path) || scheme!="https") { f.close(); outErr="bad url"; return false; }

    // append upload query
    String uploadName = cloudUploadNameForPath(filePath);
    String fullPath = path + (path.indexOf('?')>=0?"&":"?") + "upload=1&name=" + urlEncode(uploadName);

    String connErr;
    if (!tlsConnectHost(host, port, connErr)) { f.close(); outErr = connErr; return false; }

    uint32_t len = f.size();
    String hdr;
    hdr.reserve(256);
    hdr += "POST " + fullPath + " HTTP/1.1\r\n";
    hdr += "Host: " + host + "\r\n";
    hdr += "User-Agent: ESP32-W5500\r\n";
    hdr += "Content-Type: application/octet-stream\r\n";
    hdr += "X-DEV: " + cfg.devName + "\r\n";
    if (cfg.deviceId.length()) hdr += "X-DEVICE-ID: " + cfg.deviceId + "\r\n";
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
  noteNetOp("cloud_post");
  if (!cfg.cloudEnabled) { g_lastCloudErr="disabled"; return false; }
  if (!g_linkOk || !g_internetOk) {
    g_lastHttpCode = -1;
    g_lastCloudErr = "offline";
    g_lastPushIso  = isoNow();
    g_cloudOk = false;
    noteNetResult(false, g_lastCloudErr);
    return false;
  }
  String body = jsonSnapshot(includeReadings);
  String ingestUrl = cloudIngestUrlFromServerUrl();
  if (ingestUrl.length() == 0) {
    g_lastHttpCode = -1;
    g_lastCloudErr = "bad_server_url";
    g_lastPushIso  = isoNow();
    g_cloudOk = false;
    noteNetResult(false, g_lastCloudErr);
    return false;
  }
  int code; String resp, err;
  bool ok = httpsPostJson(ingestUrl, cfg.apiKey, body, code, resp, err);
  noteNetResult(ok, err);
  g_lastHttpCode = code; g_lastCloudErr = err; g_lastPushIso = isoNow();
  g_cloudOk = ok;
  if (ok) g_lastCloudOkMs = millis();
  logLine(String("[CLOUD] POST ")+ (ok?"OK ":"FAIL ") + "code="+String(code) + (err.length()?(" err="+err):""));
  return ok;
}

static bool uploadCloudFilePath(const String& ingestUrl, const String& filePath) {
  noteNetOp("cloud_upload");
  int code; String resp, err;
  bool ok = httpsUploadFile(ingestUrl, cfg.apiKey, filePath, code, resp, err);
  noteNetResult(ok, err);
  g_lastHttpCode = code; g_lastCloudErr = err; g_lastPushIso = isoNow();
  g_cloudOk = ok;
  if (ok) g_lastCloudOkMs = millis();
  logLine(String("[CLOUD] UPLOAD ")+ baseName(filePath) + " → " + (ok?"OK ":"FAIL ")
          + "code="+String(code) + (err.length()?(" err="+err):""));
  return ok;
}

static bool uploadLastSessionLegacyFullScan(const String& ingestUrl,
                                            uint16_t& attempted,
                                            uint16_t& uploaded,
                                            uint16_t& failed,
                                            bool& scannedAny) {
  attempted = 0;
  uploaded = 0;
  failed = 0;
  scannedAny = false;

  if (storageIsFaulted()) return false;

  if (g_measDir.length() && g_measFileIndex != 0xFFFFFFFFu && g_measFileIndex <= 8192u) {
    scannedAny = true;
    for (uint32_t idx = 0; idx <= g_measFileIndex; ++idx) {
      String filePath = sessionFilePath(idx);
      bool exists = false;
      {
        ScopedSpiBusLock lock;
        if (lock) {
          prepareSdSpi();
          exists = SD.exists(filePath);
        }
      }
      if (!exists) continue;
      ++attempted;
      if (uploadCloudFilePath(ingestUrl, filePath)) ++uploaded;
      else ++failed;
      yield();
    }
  } else if (g_measDir.length()) {
    scannedAny = true;
    ScopedSpiBusLock lock;
    if (!lock) return false;
    prepareSdSpi();
    File dir = SD.open(g_measDir);
    if (dir && dir.isDirectory()) {
      File f = dir.openNextFile();
      while (f) {
        bool isDir = f.isDirectory();
        String nm = baseName(String(f.name()));
        f.close();
        String lower = nm; lower.toLowerCase();
        if (!isDir && lower.endsWith(".am1")) {
          String filePath = g_measDir + "/" + nm;
          ++attempted;
          if (uploadCloudFilePath(ingestUrl, filePath)) ++uploaded;
          else ++failed;
          yield();
        }
        f = dir.openNextFile();
      }
      dir.close();
    }
  }

  if (attempted == 0 && g_measFile.length()) {
    scannedAny = true;
    ++attempted;
    if (uploadCloudFilePath(ingestUrl, g_measFile)) ++uploaded;
    else ++failed;
  }

  return (uploaded > 0 && failed == 0);
}

// Upload all AM1 parts from the current measurement session.
static bool uploadLastSession(){
  if (g_measDir.length()==0 && g_measFile.length()==0) return false;
  if (storageIsFaulted()) {
    g_lastHttpCode = -1;
    g_lastCloudErr = "storage_fault";
    g_lastPushIso = isoNow();
    g_cloudOk = false;
    storage_fault::setUploadBlocked(g_storageStatus, true);
    return false;
  }
  if (!g_linkOk || !g_internetOk) {
    g_lastHttpCode = -1;
    g_lastCloudErr = "offline";
    g_lastPushIso  = isoNow();
    g_cloudOk = false;
    return false;
  }

  String ingestUrl = cloudIngestUrlFromServerUrl();
  if (ingestUrl.length() == 0) {
    g_lastHttpCode = -1;
    g_lastCloudErr = "bad_server_url";
    g_lastPushIso  = isoNow();
    g_cloudOk = false;
    return false;
  }

  const bool canTrackPerFile =
    g_measDir.length() &&
    g_measFileIndex != 0xFFFFFFFFu &&
    g_measFileIndex < upload_retry::kMaxTrackedParts;

  if (!canTrackPerFile) {
    uint16_t attempted = 0;
    uint16_t uploaded = 0;
    uint16_t failed = 0;
    bool scannedAny = false;
    bool ok = uploadLastSessionLegacyFullScan(ingestUrl, attempted, uploaded, failed, scannedAny);
    logLine(String("[CLOUD] session upload summary dir=") + g_measDir
            + " mode=legacy attempted=" + String((unsigned)attempted)
            + " uploaded=" + String((unsigned)uploaded)
            + " failed=" + String((unsigned)failed)
            + " scanned=" + String(scannedAny ? "true" : "false"));
    if (ok) {
      clearPendingAutoCycleStateFile();
      removeUploadManifest(g_measDir);
    }
    return ok;
  }

  upload_retry::Manifest manifest{};
  if (!loadOrInitUploadManifest(g_measDir, g_measFileIndex, manifest)) {
    if (storageIsFaulted()) {
      g_lastCloudErr = "storage_fault";
      storage_fault::setUploadBlocked(g_storageStatus, true);
      return false;
    }
    logLine(String("[CLOUD] upload state unavailable for dir=") + g_measDir + " fallback=legacy");
    g_lastCloudErr = "upload_state_unavailable";
    uint16_t attempted = 0;
    uint16_t uploaded = 0;
    uint16_t failed = 0;
    bool scannedAny = false;
    bool ok = uploadLastSessionLegacyFullScan(ingestUrl, attempted, uploaded, failed, scannedAny);
    logLine(String("[CLOUD] session upload summary dir=") + g_measDir
            + " mode=legacy attempted=" + String((unsigned)attempted)
            + " uploaded=" + String((unsigned)uploaded)
            + " failed=" + String((unsigned)failed)
            + " scanned=" + String(scannedAny ? "true" : "false"));
    if (ok) {
      clearPendingAutoCycleStateFile();
      removeUploadManifest(g_measDir);
    }
    return ok;
  }

  upload_retry::beginPass(manifest);
  if (!saveUploadManifest(g_measDir, manifest)) {
    g_lastCloudErr = "upload_state_persist_fail";
    logLine(String("[CLOUD] upload state persist FAIL dir=") + g_measDir + " stage=begin_pass");
    return false;
  }

  uint16_t attempted = 0;
  uint16_t uploaded = 0;
  uint16_t failed = 0;
  const uint32_t confirmedBefore = upload_retry::uploadedCount(manifest);
  String lastFailedFile = "";
  String lastFailedErr = "";
  int lastFailedCode = -1;

  for (uint32_t idx = 0; idx <= g_measFileIndex; ++idx) {
    if (upload_retry::isUploaded(manifest, idx)) continue;

    ++attempted;
    String filePath = sessionFilePathForDir(g_measDir, idx);
    bool ok = false;
    int fileCode = -1;
    String fileErr;

    bool exists = false;
    {
      ScopedSpiBusLock lock;
      if (lock) {
        prepareSdSpi();
        exists = SD.exists(filePath);
      }
    }
    if (!exists) {
      fileCode = -1;
      fileErr = "missing_file";
      g_lastHttpCode = fileCode;
      g_lastCloudErr = fileErr;
    } else {
      ok = uploadCloudFilePath(ingestUrl, filePath);
      fileCode = g_lastHttpCode;
      fileErr = g_lastCloudErr;
    }

    if (ok) {
      ++uploaded;
      upload_retry::markUploaded(manifest, idx);
      if (!saveUploadManifest(g_measDir, manifest)) {
        g_lastCloudErr = "upload_state_persist_fail";
        logLine(String("[CLOUD] upload state persist FAIL dir=") + g_measDir
                + " stage=mark_uploaded idx=" + String((unsigned)idx));
        return false;
      }
      logLine(String("[CLOUD] UPLOAD OK file=") + baseName(filePath)
              + " code=" + String(fileCode)
              + " pass=" + String((unsigned)manifest.attemptCount)
              + " remaining_after=" + String((unsigned)upload_retry::remainingCount(manifest)));
      continue;
    }

    ++failed;
    upload_retry::markFailure(manifest, idx, fileCode, fileErr.c_str());
    if (!saveUploadManifest(g_measDir, manifest)) {
      g_lastCloudErr = "upload_state_persist_fail";
      logLine(String("[CLOUD] upload state persist FAIL dir=") + g_measDir
              + " stage=mark_failure idx=" + String((unsigned)idx));
      return false;
    }
    lastFailedFile = baseName(filePath);
    lastFailedErr = fileErr;
    lastFailedCode = fileCode;
    logLine(String("[CLOUD] UPLOAD FAIL file=") + lastFailedFile
            + " code=" + String(lastFailedCode)
            + " err=" + lastFailedErr
            + " pass=" + String((unsigned)manifest.attemptCount)
            + " remaining_after=" + String((unsigned)upload_retry::remainingCount(manifest)));
  }

  const uint32_t confirmedTotal = upload_retry::uploadedCount(manifest);
  const uint32_t remainingTotal = upload_retry::remainingCount(manifest);
  if (lastFailedFile.length() == 0 && manifest.lastFailedIndex >= 0) {
    lastFailedFile = baseName(sessionFilePathForDir(g_measDir, static_cast<uint32_t>(manifest.lastFailedIndex)));
    lastFailedErr = String(manifest.lastError);
    lastFailedCode = manifest.lastHttpCode;
  }

  logLine(String("[CLOUD] session upload summary dir=") + g_measDir
          + " mode=manifest"
          + " pass=" + String((unsigned)manifest.attemptCount)
          + " attempted=" + String((unsigned)attempted)
          + " uploaded=" + String((unsigned)uploaded)
          + " failed=" + String((unsigned)failed)
          + " confirmed_before=" + String((unsigned)confirmedBefore)
          + " confirmed_total=" + String((unsigned)confirmedTotal)
          + " remaining=" + String((unsigned)remainingTotal)
          + " last_failed_file=" + lastFailedFile
          + " last_failed_code=" + String(lastFailedCode)
          + " last_failed_err=" + lastFailedErr);

  if (!upload_retry::isComplete(manifest)) return false;

  removeUploadManifest(g_measDir);
  clearPendingAutoCycleStateFile();
  return true;
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

static int parseRateFromParams(const String& params) {
  int p = params.indexOf("rate=");
  if (p < 0) return -1;
  int start = p + 5;
  int end = params.indexOf('&', start);
  String raw = (end < 0) ? params.substring(start) : params.substring(start, end);
  raw.trim();
  int r = raw.toInt();
  return validSps(r) ? r : -1;
}

static bool remoteActionAllowed(const String& action, String& reason) {
  reason = "";
  uint32_t now = millis();
  if (action == "reboot") {
    if (now - g_lastRemoteRebootMs < REMOTE_REBOOT_COOLDOWN_MS) {
      reason = "cooldown";
      return false;
    }
    return true;
  }
  if (now - g_lastRemoteStartStopMs < REMOTE_STARTSTOP_COOLDOWN_MS) {
    reason = "cooldown";
    return false;
  }
  return true;
}

static bool ackRemoteCommand(const RemoteCommand& cmd, bool ok, const String& result) {
  String ackUrl = remoteAckUrl(cmd.id);
  if (ackUrl.length() == 0) return false;
  noteNetOp("remote_ack");
  String payload = "{";
  payload += "\"ok\":" + String(ok ? "true" : "false") + ",";
  payload += "\"result\":\"" + jsonEscape(result) + "\",";
  payload += "\"time\":\"" + isoNow() + "\",";
  payload += "\"status\":" + jsonSnapshot(false);
  payload += "}";
  int code = -1;
  String resp, err;
  bool postOk = httpsPostJson(ackUrl, cfg.apiKey, payload, code, resp, err);
  noteNetResult(postOk, err);
  g_cloudOk = postOk;
  if (postOk) g_lastCloudOkMs = millis();
  g_lastHttpCode = code;
  g_lastCloudErr = err;
  return postOk;
}

static bool executeRemoteCommand(const RemoteCommand& cmd, String& result) {
  String reason;
  if (!remoteActionAllowed(cmd.action, reason)) {
    result = "rejected_" + reason;
    return false;
  }

  if (cmd.action == "measure_start") {
    int rateOverride = parseRateFromParams(cmd.params);
    bool ok = false;
    String err;
    ok = startMeasurementCore(rateOverride, err);
    g_lastRemoteStartStopMs = millis();
    if (ok && err == "already") { result = "already_running"; return true; }
    result = ok ? "started" : ("start_failed_" + err);
    return ok;
  }

  if (cmd.action == "measure_stop") {
    bool uploaded = false;
    String file;
    String err;
    bool ok = stopMeasurementCore(true, false, uploaded, file, err);
    g_lastRemoteStartStopMs = millis();
    if (ok && err == "already") { result = "already_stopped"; return true; }
    if (ok) {
      if (uploaded) {
        scheduleDeferredReboot("[MEAS] remote stop: upload complete, reboot scheduled");
        result = "stopped_uploaded";
      }
      else if (hasPendingSessionUpload()) result = "stopped_upload_pending";
      else result = "stopped";
      return true;
    }
    result = "stop_failed_" + err;
    return false;
  }

  if (cmd.action == "reboot") {
    g_lastRemoteRebootMs = millis();
    scheduleDeferredReboot("[REMOTE] reboot scheduled", 1500);
    result = "reboot_scheduled";
    return true;
  }

  result = "unsupported_action";
  return false;
}

static void rememberLastCommandId(const String& cmdId) {
  g_lastRemoteCmdId = cmdId;
  if (cfg.lastCommandId == cmdId) return;
  cfg.lastCommandId = cmdId;
  saveCfg();
}

static void remotePollTick() {
  if (!cfg.remoteEnabled) return;
  uint32_t now = millis();
  uint32_t effectiveInterval = measurementDeferredIntervalMs(g_remotePollIntervalMs);
  if (localPortalClientConnected() && effectiveInterval < REMOTE_POLL_PORTAL_INTERVAL_MS) {
    effectiveInterval = REMOTE_POLL_PORTAL_INTERVAL_MS;
  }
  if (now - g_lastRemotePollMs < effectiveInterval) return;
  g_lastRemotePollMs = now;

  if (!g_linkOk || !g_internetOk) return;

  String cmdUrl = remoteCommandsUrl();
  if (cmdUrl.length() == 0) {
    g_lastRemoteCmdResult = "bad_remote_url";
    return;
  }

  int code = -1;
  String resp, err;
  noteNetOp("remote_poll");
  bool ok = httpsGetText(cmdUrl, cfg.apiKey, code, resp, err);
  noteNetResult(ok || code == 204, err);
  if (!ok && code != 204) {
    g_cloudOk = false;
    g_lastRemoteCmdResult = "poll_failed";
    if (g_remotePollFailStreak < 10) g_remotePollFailStreak++;
    uint32_t backoff = REMOTE_POLL_INTERVAL_MS;
    for (uint8_t i = 0; i < g_remotePollFailStreak; ++i) {
      if (backoff >= REMOTE_POLL_MAX_BACKOFF_MS / 2) { backoff = REMOTE_POLL_MAX_BACKOFF_MS; break; }
      backoff *= 2;
    }
    if (backoff > REMOTE_POLL_MAX_BACKOFF_MS) backoff = REMOTE_POLL_MAX_BACKOFF_MS;
    g_remotePollIntervalMs = backoff;
    logLine(String("[REMOTE] poll FAIL code=") + String(code) + " err=" + err);
    return;
  }
  g_remotePollFailStreak = 0;
  g_remotePollIntervalMs = REMOTE_POLL_INTERVAL_MS;
  g_lastRemotePollOkMs = millis();
  g_cloudOk = true;
  g_lastCloudOkMs = millis();

  if (code == 204 || resp.length() == 0) return;

  RemoteCommand cmd;
  if (!parseRemoteCommand(resp, cmd)) {
    g_lastRemoteCmdResult = "bad_payload";
    logLine("[REMOTE] bad command payload");
    return;
  }

  if (cmd.id == cfg.lastCommandId) {
    g_lastRemoteCmdResult = "duplicate_ignored";
    ackRemoteCommand(cmd, true, "duplicate_ignored");
    return;
  }

  bool sigOk = verifyCommandSignature(cmd);
  if (!sigOk) {
    g_lastRemoteCmdResult = "bad_signature";
    ackRemoteCommand(cmd, false, "bad_signature");
    rememberLastCommandId(cmd.id);
    logLine(String("[REMOTE] signature reject id=") + cmd.id);
    return;
  }

  String result;
  bool execOk = executeRemoteCommand(cmd, result);
  bool ackOk = ackRemoteCommand(cmd, execOk, result);
  rememberLastCommandId(cmd.id);
  g_lastRemoteCmdResult = result + (ackOk ? "" : "_ack_failed");
  logLine(String("[REMOTE] id=") + cmd.id + " action=" + cmd.action + " exec=" + (execOk ? "ok" : "fail") + " ack=" + (ackOk ? "ok" : "fail"));
}

// --- Measurement control ---
static void clearMeasurementAutoRestartState() {
  g_measAutoRestartPending = false;
  g_measAutoRestartWaitingUpload = false;
  g_measAutoRestartLastAttemptMs = 0;
  g_measAutoRestartLastLogMs = 0;
  clearPendingAutoCycleStateFile();
}

static void scheduleDeferredReboot(const String& reason, uint32_t delayMs) {
  g_pendingRemoteReboot = true;
  g_pendingRemoteRebootAtMs = millis() + delayMs;
  g_pendingRebootReason = reason;
  if (reason.length()) logLine(reason);
}

static bool scheduleAutoCycleRebootAfterUpload() {
  if (storageIsFaulted()) return false;
  if (g_measDir.length() == 0 || g_measFileIndex == 0xFFFFFFFFu) {
    return false;
  }
  if (!savePendingAutoCycleState(g_measDir, g_measFileIndex, false, true)) {
    logLine(String("[MEAS] auto cycle: failed to persist restart-after-upload state dir=") + g_measDir);
    return false;
  }

  g_measAutoRestartPending = false;
  g_measAutoRestartWaitingUpload = false;
  g_measAutoRestartLastAttemptMs = 0;
  g_measAutoRestartLastLogMs = 0;
  scheduleDeferredReboot("[MEAS] auto cycle: upload complete, rebooting before restart");
  return true;
}

static bool shouldWaitForMeasurementAutoRestartUpload() {
  return cfg.cloudEnabled && cfg.uploadOnStop;
}

static void measurementAutoRestartLog(const String& msg, bool force=false) {
  uint32_t now = millis();
  if (!force && g_measAutoRestartLastLogMs != 0 && (now - g_measAutoRestartLastLogMs) < 30000UL) {
    return;
  }
  g_measAutoRestartLastLogMs = now;
  logLine(msg);
}

static bool startMeasurementCore(int rateOverride, String& outErr) {
  outErr = "";
  if (g_measActive) {
    outErr = "already";
    return true;
  }

  if (validSps(rateOverride)) {
    for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) g_rateCh[ch] = rateOverride;
    g_adsRateSps = rateOverride;
    adsConfigSave();
    adsConfigLoad();
    if (adsReady) adsApplyHW();
  }

  int usedChannels = 0;
  int effectiveRateSps = 0;
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    if (!g_adsActive[ch]) continue;
    const int sps = validSps(g_rateCh[ch]) ? g_rateCh[ch] : ADS_DEFAULT_SPS;
    usedChannels++;
    if (effectiveRateSps == 0 || sps < effectiveRateSps) {
      effectiveRateSps = sps;
    }
  }
  const float totalSps = (usedChannels > 0) ? float(effectiveRateSps) : 0.0f;
  const float perChSps = (totalSps > 0.0f && usedChannels > 0)
                           ? (totalSps / float(usedChannels))
                           : 0.0f;

  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) {
    if (g_adsActive[ch]) {
      g_measSps[ch]  = perChSps;
      g_measDtMs[ch] = (g_measSps[ch]>0) ? (1000.0f / g_measSps[ch]) : 0.0f;
    } else {
      g_measSps[ch]  = 0.0f;
      g_measDtMs[ch] = 0.0f;
    }
  }

  g_measId     = isoNowFileSafe();
  g_measDir    = "/meas/sess_" + g_measId;
  g_measFile   = "";
  g_measFileIndex = 0xFFFFFFFFu;
  g_measFileIdxOffset = 0;
  g_measHaveRawIdx = false;
  g_measLastRawIdx = 0;
  g_startUs = 0;
  g_nextDueUs = 0;
  g_frameCount = 0;
  g_measBytes  = 0;
  if (!measurementResetBuffers()) {
    outErr = "buffer alloc fail";
    return false;
  }
  if (!switchToMeasFile(0)) {
    g_measId        = "";
    g_measDir       = "";
    g_measFile      = "";
    g_measFileIndex = 0xFFFFFFFFu;
    g_measBytes     = 0;
    logLine("[MEAS] start failed: cannot open first file");
    outErr = "sd write fail";
    return false;
  }

  g_measActive = true;
  g_pairHz     = 0.0f;

  BaseType_t ok = xTaskCreatePinnedToCore(meas_task_bin, "meas_bin", 6144, nullptr, 2, &g_measTask, 0);
  if (ok != pdPASS) {
    g_measActive = false;
    outErr = "task create fail";
    return false;
  }

  clearMeasurementAutoRestartState();
  String spsLog = "[MEAS] start BIN: " + g_measFile + " | SPS=";
  for (uint8_t ch=0; ch<NUM_SENSORS; ++ch) { if (ch) spsLog += "/"; spsLog += String(g_measSps[ch]); }
  logLine(spsLog);
  return true;
}

static bool stopMeasurementCore(bool doUpload, bool persistAutoCyclePending,
                                bool& outUploaded, String& outFile, String& outErr) {
  outErr = "";
  outUploaded = false;
  outFile = g_measFile;

  if (!g_measActive) {
    if (doUpload && cfg.cloudEnabled && cfg.uploadOnStop && hasPendingSessionUpload()) {
      if (storageIsFaulted()) {
        outErr = "storage_fault";
        return false;
      }
      logLine(String("[MEAS] stop BIN: completing pending upload for ") + g_measFile);
      g_measAutoRestartPending = false;
      g_measAutoRestartWaitingUpload = false;
      g_measAutoRestartLastAttemptMs = 0;
      g_measAutoRestartLastLogMs = 0;
      outUploaded = uploadLastSession();
      if (!outUploaded) {
        outErr = "upload_failed";
        return false;
      }
      outFile = g_measFile;
      return true;
    }
    outErr = "already";
    return true;
  }

  g_measActive = false;
  uint32_t t0 = millis();
  while (g_measTask && millis() - t0 < 800) { delay(10); }
  bool flushedAll = measurementFlushAllBuffersBlocking();

  logLine("[MEAS] stop BIN: " + g_measFile);

  if (!flushedAll) {
    outErr = "sd_write_fail";
    return false;
  }

  if (doUpload && storageIsFaulted()) {
    outErr = "storage_fault";
    return false;
  }

  if (doUpload && cfg.cloudEnabled && cfg.uploadOnStop &&
      g_measDir.length() && g_measFileIndex != 0xFFFFFFFFu) {
    if (!savePendingAutoCycleState(g_measDir, g_measFileIndex, true, persistAutoCyclePending)) {
      logLine(String("[MEAS] pending upload state persist FAIL dir=") + g_measDir);
    }
  }

  if (doUpload && cfg.cloudEnabled && cfg.uploadOnStop) {
    outUploaded = uploadLastSession();
  } else if (doUpload && cfg.cloudEnabled && !cfg.uploadOnStop) {
    logLine("[CLOUD] upload on stop disabled");
  }
  outFile = g_measFile;
  return true;
}

static void measurementAutoCycleTick() {
  uint32_t now = millis();

  if (g_measActive && !g_measAutoRestartPending) {
    if (g_startUs == 0) return;

    uint64_t elapsedUs = monotonicMicros() - g_startUs;
    uint64_t elapsedTicks = elapsedUs / 10ULL;
    if (elapsedTicks < (uint64_t)MEAS_AUTOCYCLE_LIMIT_TICKS) return;

    logLine("[MEAS] auto cycle: session limit reached, stopping for scheduled restart");
    bool waitForUpload = shouldWaitForMeasurementAutoRestartUpload();
    bool uploaded = false;
    String file;
    String err;
    bool stopOk = stopMeasurementCore(waitForUpload, true, uploaded, file, err);
    if (!stopOk) {
      if (err == "storage_fault") {
        clearMeasurementAutoRestartState();
        measurementAutoRestartLog("[MEAS] auto cycle: storage fault blocks upload; staying in standby", true);
        return;
      }
      measurementAutoRestartLog(String("[MEAS] auto cycle: stop failed err=") + err, true);
      return;
    }

    g_measAutoRestartPending = true;
    g_measAutoRestartWaitingUpload = waitForUpload && !uploaded;
    g_measAutoRestartLastAttemptMs = now;
    g_measAutoRestartLastLogMs = 0;

    if (g_measAutoRestartWaitingUpload) {
      measurementAutoRestartLog("[MEAS] auto cycle: waiting for successful upload before restart", true);
      return;
    }

    if (uploaded && waitForUpload) {
      if (scheduleAutoCycleRebootAfterUpload()) {
        return;
      }
      measurementAutoRestartLog("[MEAS] auto cycle: reboot scheduling failed, restarting without reboot", true);
    }

    String startErr;
    bool startOk = startMeasurementCore(-1, startErr);
    if (!startOk) {
      g_measAutoRestartPending = true;
      g_measAutoRestartWaitingUpload = false;
      g_measAutoRestartLastAttemptMs = now;
      measurementAutoRestartLog(String("[MEAS] auto cycle: immediate restart failed err=") + startErr, true);
      return;
    }

    logLine("[MEAS] auto cycle: new session started");
    return;
  }

  if ((!g_measAutoRestartPending && !g_measAutoRestartWaitingUpload) || g_measActive) return;

  if (g_measAutoRestartWaitingUpload) {
    if (storageIsFaulted()) {
      clearMeasurementAutoRestartState();
      measurementAutoRestartLog("[MEAS] auto cycle: upload blocked by storage fault; staying in standby", true);
      return;
    }
    if (!shouldWaitForMeasurementAutoRestartUpload()) {
      g_measAutoRestartWaitingUpload = false;
      g_measAutoRestartLastAttemptMs = now;
      if (g_measAutoRestartPending) {
        logLine("[MEAS] auto cycle: upload wait skipped, restarting offline");
      } else {
        clearMeasurementAutoRestartState();
        logLine("[MEAS] pending upload cancelled: upload on stop disabled");
        return;
      }
    } else {
      if ((now - g_measAutoRestartLastAttemptMs) < MEAS_AUTOCYCLE_UPLOAD_RETRY_MS) return;
      if (!g_linkOk || !g_internetOk) {
        measurementAutoRestartLog("[MEAS] pending upload waiting for connectivity");
        return;
      }
      g_measAutoRestartLastAttemptMs = now;
      bool uploaded = uploadLastSession();
      if (!uploaded) {
        measurementAutoRestartLog(String("[MEAS] auto cycle: upload retry failed err=") + g_lastCloudErr);
        return;
      }
      g_measAutoRestartWaitingUpload = false;
      g_measAutoRestartLastAttemptMs = now;
      if (!g_measAutoRestartPending) {
        clearMeasurementAutoRestartState();
        logLine("[MEAS] pending upload complete");
        return;
      }
      if (scheduleAutoCycleRebootAfterUpload()) {
        return;
      }
      measurementAutoRestartLog("[MEAS] auto cycle: reboot scheduling failed, restarting without reboot", true);
      logLine("[MEAS] auto cycle: upload complete, restarting measurement");
    }
  }

  if ((now - g_measAutoRestartLastAttemptMs) < MEAS_AUTOCYCLE_RESTART_RETRY_MS) return;
  g_measAutoRestartLastAttemptMs = now;

  String startErr;
  bool startOk = startMeasurementCore(-1, startErr);
  if (!startOk) {
    measurementAutoRestartLog(String("[MEAS] auto cycle: restart retry failed err=") + startErr);
    return;
  }

  logLine("[MEAS] auto cycle: new session started");
}

void handleMeasStart(){
  int rateOverride = -1;
  if (server.hasArg("rate")) rateOverride = server.arg("rate").toInt();
  String err;
  bool ok = startMeasurementCore(rateOverride, err);
  if (ok && err == "already") {
    server.send(200,"application/json","{\"ok\":true,\"already\":true}");
    return;
  }
  if (!ok) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"" + jsonEscape(err) + "\"}");
    return;
  }
  g_portalMeasurementLockHold = true;
  server.send(200,"application/json","{\"ok\":true}");
}

void handleMeasStop(){
  bool uploaded = false;
  String file;
  String err;
  bool ok = stopMeasurementCore(true, false, uploaded, file, err);
  if (ok && err == "already") {
    server.send(200,"application/json","{\"ok\":true,\"already\":true}");
    return;
  }
  if (!ok) {
    server.send(200,"application/json","{\"ok\":false,\"err\":\"" + jsonEscape(err) + "\"}");
    return;
  }
  g_portalMeasurementLockHold = true;
  String j = String("{\"ok\":true,\"uploaded\":") + (uploaded?"true":"false") + ",\"file\":\""+jsonEscape(file)+"\"}";
  server.send(200,"application/json", j);
  if (uploaded) {
    scheduleDeferredReboot("[MEAS] local stop: upload complete, reboot scheduled");
  }
}

void handleMeasDebug(){
  String j;
  j.reserve(220);
  j += "{\"active\":";
  j += g_measActive ? "true" : "false";
  j += ",\"task\":\"";
  j += g_measTask ? "yes" : "no";
  j += "\",\"batch\":";
  j += String((unsigned)measurementBufferedFrameCount());
  j += ",\"frames\":";
  j += String((unsigned)g_frameCount);
  j += ",\"frame_hz\":";
  j += String(g_pairHz,1);
  j += ",\"loop_stack_free\":";
  j += String(stackWatermarkBytes(g_loopTaskHandle));
  j += ",\"meas_stack_free\":";
  j += String(stackWatermarkBytes(g_measTask));
  j += ",\"heap_free\":";
  j += String(runtimeFreeHeap());
  j += ",\"heap_min\":";
  j += String(runtimeMinFreeHeap());
  j += ",\"cpu_busy_pct\":";
  j += g_runtimeCpu.valid ? String((unsigned)g_runtimeCpu.cpuBusyPct) : String("null");
  j += ",\"last_stage\":\"";
  j += jsonEscape(String(g_lastRuntimeStage));
  j += "\"";
  j += ",\"loop_max_block_ms\":";
  j += String(g_loopMaxBlockMs);
  j += "}";
  server.send(200,"application/json", j);
}

void handleMeasStatus(){
  String j = "{";
  j += "\"active\":" + String(g_measActive?"true":"false") + ",";
  j += "\"id\":\"" + g_measId + "\",\"file\":\"" + g_measFile + "\",";
  j += "\"frames\":" + String((unsigned)g_frameCount) + ",";
  j += "\"bytes\":" + String((unsigned long long)g_measBytes) + ",";
  j += "\"heap_free\":" + String(runtimeFreeHeap()) + ",";
  j += "\"heap_min\":" + String(runtimeMinFreeHeap()) + ",";
  j += "\"heap_frag_pct\":" + String((unsigned)runtimeHeapFragPct()) + ",";
  j += "\"cpu_busy_pct\":";
  j += g_runtimeCpu.valid ? String((unsigned)g_runtimeCpu.cpuBusyPct) : String("null");
  j += ",";
  j += "\"loop_stack_free\":" + String(stackWatermarkBytes(g_loopTaskHandle)) + ",";
  j += "\"meas_stack_free\":" + String(stackWatermarkBytes(g_measTask)) + ",";
  j += "\"loop_age_ms\":" + String(g_loopLastMs ? (millis() - g_loopLastMs) : 0) + ",";
  j += "\"loop_max_block_ms\":" + String(g_loopMaxBlockMs) + ",";
  j += "\"last_stage\":\"" + jsonEscape(String(g_lastRuntimeStage)) + "\",";
  j += "\"storage\":{";
  j += "\"state\":\"" + jsonEscape(String(storageStateName())) + "\",";
  j += "\"fault\":" + String(storageIsFaulted() ? "true" : "false") + ",";
  j += "\"degraded\":" + String(storageIsDegraded() ? "true" : "false") + ",";
  j += "\"log_source\":\"" + storageLogSource() + "\",";
  j += "\"last_err\":\"" + jsonEscape(String(g_storageStatus.lastError)) + "\",";
  j += "\"upload_blocked\":" + String(g_storageStatus.uploadBlocked ? "true" : "false");
  j += "},";
  // Expose which channels are logically active (no guessing in JS)
  uint8_t activeMask = 0;
  uint8_t activeCount = 0;
  for (uint8_t ch = 0; ch < NUM_SENSORS; ++ch) {
    if (g_adsActive[ch]) {
      activeMask |= (uint8_t(1u) << ch);
      ++activeCount;
    }
  }
  j += "\"active_mask\":" + String((unsigned)activeMask) + ",";
  j += "\"active_channels\":" + String((unsigned)activeCount) + ",";

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
  if (storageIsFaulted()) { server.send(503,"text/plain","storage_fault"); return; }
  String path = safePath(urlDecodePath(server.arg("path")));
  if (!storageCanUseSd()) { server.send(503,"text/plain","SD unavailable"); return; }
  ScopedSpiBusLock lock;
  if (!lock) { server.send(503,"text/plain","SPI lock failed"); return; }
  prepareSdSpi();
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

  if (storageIsFaulted() || nm == "ram-live.log") {
    String tail = ramLogText(lines);
    if (tail.length() == 0) tail = "(no log)";
    server.sendContent(sseEncode(tail));
    return;
  }

  // Read last N lines and push as a single SSE event, then return immediately
  if (!storageCanUseSd()) {
    server.sendContent("data: (sd unavailable)\n\n");
    return;
  }
  ScopedSpiBusLock lock;
  if (!lock) {
    server.sendContent("data: (spi lock failed)\n\n");
    return;
  }
  prepareSdSpi();

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
  if (storageIsFaulted()) {
    String ram = ramLogText();
    String out = "{\"source\":\"ram\",\"items\":[{\"name\":\"ram-live.log\",\"size\":" + String(ram.length()) + "}]}";
    server.send(200,"application/json", out);
    return;
  }
  if (!storageCanUseSd()) { server.send(200,"application/json","{\"source\":\"ram\",\"items\":[]}"); return; }
  ScopedSpiBusLock lock;
  if (!lock) { server.send(200,"application/json","{\"source\":\"ram\",\"items\":[]}"); return; }
  prepareSdSpi();
  if (!SD.exists("/logs")) SD.mkdir("/logs");

  File dir = SD.open("/logs");                // some cores need "/logs/" — we try plain "/logs"
  if (!dir) { server.send(200,"application/json","{\"source\":\"sd\",\"items\":[]}"); return; }

  String out = "{\"source\":\"sd\",\"items\":[";
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
  String nm = baseName(urlDecode(server.arg("file")));
  if (storageIsFaulted() || nm == "ram-live.log") {
    String text = ramLogText();
    server.sendHeader("Content-Disposition","attachment; filename=\"ram-live.log\"");
    server.send(200, "text/plain", text);
    return;
  }
  if (!storageCanUseSd()) { server.send(404,"text/plain","Not found"); return; }
  ScopedSpiBusLock lock;
  if (!lock) { server.send(503,"text/plain","SPI lock failed"); return; }
  prepareSdSpi();

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
  clearRamLogBuffer();

  if (!storageCanUseSd()) { server.send(200,"text/plain","ok"); return; }
  ScopedSpiBusLock lock;
  if (!lock) { server.send(200,"text/plain","ok"); return; }
  prepareSdSpi();
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
  if (storageIsFaulted()) {
    server.send(200, "text/plain", ramLogText(server.hasArg("lines") ? server.arg("lines").toInt() : 200));
    return;
  }

  if (!storageCanUseSd()) { server.send(503, "text/plain", "SD not mounted"); return; }

  String nm = baseName(urlDecode(server.arg("file")));
  if (nm.length() == 0) nm = "log0.log";
  int lines = server.hasArg("lines") ? server.arg("lines").toInt() : 200;
  if (lines < 10) lines = 10;
  if (lines > 1000) lines = 1000;

  if (nm == "ram-live.log") {
    server.send(200, "text/plain", ramLogText(lines));
    return;
  }

  ScopedSpiBusLock lock;
  if (!lock) { server.send(503, "text/plain", "SPI lock failed"); return; }
  prepareSdSpi();

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
  HTTPMethod method = server.method();
  String uri = server.uri();
  Serial.printf("[HTTP] 404 %s %s\n",
    method==HTTP_GET?"GET":
    method==HTTP_POST?"POST":
    method==HTTP_OPTIONS?"OPTIONS":
    method==HTTP_HEAD?"HEAD":"OTHER",
    uri.c_str());

  // For captive probes and other non-browser requests, return quickly so the
  // single-client AP web server is not monopolized by background probe traffic.
  if (method == HTTP_POST || method == HTTP_OPTIONS || method == HTTP_HEAD) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(204);
    return;
  }

  // Unknown GETs from AP clients are best redirected to the local portal root.
  if (cfg.commissioningMode) {
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/");
    server.send(302, "text/plain", "");
    return;
  }

  handleRoot();
}

// ---- ROUTES: OTA ----
static bool g_otaAuthorized = false;
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

  if (portalMeasurementLockActive()) {
    if (up.status == UPLOAD_FILE_START) {
      sendPortalMeasurementLockedJson();
    }
    return;
  }

  if (up.status == UPLOAD_FILE_START) {
    g_otaAuthorized = isAuthorizedRequest();
    if (!g_otaAuthorized) return;
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

  } else if (!g_otaAuthorized) {
    if (up.status == UPLOAD_FILE_END || up.status == UPLOAD_FILE_ABORTED) {
      server.send(401,"application/json","{\"ok\":false,\"err\":\"unauthorized\"}");
    }
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
  String host = makeHostname(cfg.devName);

  WiFi.mode(WIFI_AP);
  WiFi.softAPsetHostname(host.c_str());
  WiFi.softAP(apSsid.c_str(), cfg.apPass.c_str());
  delay(100);
  Serial.print("[AP] SSID: "); Serial.println(apSsid);
  Serial.print("[AP] PASS: "); Serial.println(cfg.apPass);
  Serial.print("[AP] IP: ");   Serial.println(WiFi.softAPIP());
  Serial.print("[AP] HOST: "); Serial.println(host + ".local");

  dns.start(53, "*", WiFi.softAPIP());

  // --- mDNS on the AP interface (after AP exists) ---
  if (g_mdnsRunning) { MDNS.end(); g_mdnsRunning = false; }
  if (MDNS.begin(host.c_str())) {
    MDNS.addService("http", "tcp", 80);
    g_mdnsRunning = true;
    Serial.printf("[mDNS] %s.local on AP\n", host.c_str());
  } else {
    Serial.println("[mDNS] start failed (AP)");
  }
  // --------------------------------------------------
  const char* hdrs[] = {"Authorization"};
  server.collectHeaders(hdrs, 1);

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
  server.on("/service/update2/json", HTTP_ANY, [](){ // Firefox / browser background probe
    server.sendHeader("Cache-Control", "no-store");
    server.send(204);
  });
  server.on("/ncsi.txt", HTTP_GET, [](){            // Windows
    server.send(200, "text/plain", "Microsoft NCSI");
  });

  // Config + status
  server.on("/",       HTTP_GET, handleRoot);
  server.on("/save",   HTTP_POST, [](){ if (rejectPortalDuringMeasurementJson()) return; if (!requireAuth()) return; handleSave(); });
  server.on("/reboot", HTTP_GET,  [](){ if (rejectPortalDuringMeasurementText()) return; if (!requireAuth()) return; handleReboot(); });
  server.on("/status", HTTP_GET,  handleStatus);

  // SD
  server.on("/fs",     HTTP_GET,  [](){ if (rejectPortalDuringMeasurementJson()) return; handleFsList(); });
  server.on("/dl",     HTTP_GET,  [](){ if (rejectPortalDuringMeasurementText()) return; handleDownload(); });
  server.on("/dlbundle", HTTP_GET, [](){ if (rejectPortalDuringMeasurementText()) return; handleDownloadBundle(); });
  server.on("/rm",     HTTP_POST, [](){ if (rejectPortalDuringMeasurementJson()) return; if (!requireAuth()) return; handleDelete(); });
  server.on("/mkdir",  HTTP_POST, [](){ if (rejectPortalDuringMeasurementJson()) return; if (!requireAuth()) return; handleMkdir(); });
  server.on("/upload", HTTP_POST, [](){}, handleUploadPost);

  // Logs
  server.on("/logs",     HTTP_GET,  [](){ if (rejectPortalDuringMeasurementJson()) return; handleLogsList(); });
  server.on("/logdl",    HTTP_GET,  [](){ if (rejectPortalDuringMeasurementText()) return; handleLogDownload(); });
  server.on("/logclear", HTTP_POST, [](){ if (rejectPortalDuringMeasurementText()) return; if (!requireAuth()) return; handleLogClear(); });

  // OTA
  server.on("/ota", HTTP_POST, [](){}, handleOTAUpload);

  server.on("/logtail", HTTP_GET, [](){ if (rejectPortalDuringMeasurementText()) return; handleLogTail(); });

  server.on("/logstream", HTTP_GET, [](){ if (rejectPortalDuringMeasurementText()) return; handleLogStream(); });

  server.on("/ads",     HTTP_GET,  [](){ if (rejectPortalDuringMeasurementJson()) return; handleAdsGet(); });
  server.on("/adsconf", HTTP_POST, [](){ if (rejectPortalDuringMeasurementJson()) return; if (!requireAuth()) return; handleAdsConf(); });
  server.on("/adsregs", HTTP_GET, [](){ if (rejectPortalDuringMeasurementText()) return; handleAdsRegs(); });

  // register:
  server.on("/adsdump", HTTP_GET, [](){ if (rejectPortalDuringMeasurementJson()) return; handleAdsDump(); });

  server.on("/cloudtest", HTTP_ANY, [](){ if (rejectPortalDuringMeasurementJson()) return; if (!requireAuth()) return; handleCloudTest(); });
  server.on("/clouddiag", HTTP_GET, [](){ if (rejectPortalDuringMeasurementJson()) return; handleCloudDiag(); });
  server.on("/measure/start", HTTP_POST, [](){ if (rejectPortalDuringMeasurementJson()) return; if (!requireAuth()) return; handleMeasStart(); });
  server.on("/measure/stop",  HTTP_POST, [](){ if (!requireAuth()) return; handleMeasStop(); });
  server.on("/measure/status",HTTP_GET,  handleMeasStatus);
  server.on("/measure/debug", HTTP_GET, [](){ if (rejectPortalDuringMeasurementJson()) return; handleMeasDebug(); });

  server.on("/export_csv", HTTP_GET, [](){ if (rejectPortalDuringMeasurementText()) return; handleExportCsv(); });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("[WEB] Portal ready");
}

// ---- SETUP/LOOP ----
void setup() {
  Serial.begin(115200);
  g_loopTaskHandle = xTaskGetCurrentTaskHandle();
  prepareBootBreadcrumb();
  g_adsMutex = xSemaphoreCreateMutex();
  g_spiMutex = xSemaphoreCreateRecursiveMutex();
  delay(250);
  Serial.println("\n=== Boot ===");

  seedRNG_noADC();

  statusLedsInit();
  ledSelfTest();

  deselectAll();
  deriveEthMacFromEfuse();
  loadCfg();
  ensureSecurityDefaults();
  g_lastRemoteCmdId = cfg.lastCommandId;

  // Ethernet + SD
  bool ethOk = ethernetInit();
  lastLink = ethLinkStatusSafe();
  g_linkOk = (lastLink == LinkON);
  bool hadStorageRecoveryMarker = loadStorageRecoveryMarker();
  if (hadStorageRecoveryMarker) {
    storage_fault::setRecovering(g_storageStatus, true);
  }
  sdMounted = sdInit();
  g_sdBootMounted = sdMounted;
  if (sdMounted) {
    String sdSelfTestErr;
    String sdSelfTestPath;
    if (!runSdSelfTest(sdSelfTestErr, sdSelfTestPath)) {
      handlePersistentStorageFault(sdSelfTestPath.length() ? sdSelfTestPath : String("/sd"),
                                   sdSelfTestErr.length() ? sdSelfTestErr : String("selftest_fail"));
    } else {
      if (hadStorageRecoveryMarker) {
        saveStorageRecoveryMarker(false);
        storageClearHealthy();
        emitSerialAndRamOnly("[SD] recovery self-test OK");
      } else {
        storageClearHealthy();
      }
      ensureLogsDir();
    }
  } else {
    storage_fault::setRecovering(g_storageStatus, hadStorageRecoveryMarker);
    storage_fault::setUploadBlocked(g_storageStatus, true);
    handlePersistentStorageFault("/sd", "mount_fail");
  }
  logLine(String("[SUM] ETH=") + (ethOk?"OK":"FAIL") + " LINK=" + (lastLink==LinkON?"UP":"DOWN") + " SD=" + (sdMounted?"OK":"FAIL"));
  if (g_havePrevBreadcrumb) {
    logLine(String("[BOOTDIAG] prev ") + previousBreadcrumbSummary());
  }

  // NTP time
  // set TZ even before sync, so localtime uses Budapest after sync
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();

  refreshInternetState(true, 1000);
  g_forceNtpSync = true;
  ntpSyncTick();
  setupTime();
  restorePendingAutoCycleState();

  adsPrefsReady = adsPrefs.begin(ADS_PREF_NS, false);  // 1) open NVS
  adsConfigLoad();                     // 2) pull config from NVS into globals
  adsInit();                           // 3) init ADS (ads.begin(...) sets adsReady)
  adsApplyHW();                        // 4) program gain/rate to chip from loaded globals

  // AP + web (optional commissioning mode)
  if (cfg.commissioningMode) {
    startApAndPortal();
  } else {
    WiFi.mode(WIFI_OFF);
    Serial.println("[WEB] commissioning mode disabled; AP portal not started");
  }

  // LEDs
  updateStatusLeds();
}

void loop() {
  uint32_t now = millis();
  if (g_loopPrevMs != 0) {
    uint32_t delta = now - g_loopPrevMs;
    if (delta > g_loopMaxBlockMs) g_loopMaxBlockMs = delta;
    if (delta >= 2000UL && (now - g_lastLoopStallLogMs) >= 10000UL) {
      g_lastLoopStallLogMs = now;
      logLine(String("[HEALTH] loop stall ms=") + String(delta)
              + " stage=" + String(g_lastRuntimeStage)
              + " net_op=" + g_lastNetOp
              + " heap=" + String(runtimeFreeHeap())
              + " loop_stack=" + String(stackWatermarkBytes(g_loopTaskHandle))
              + " meas_stack=" + String(stackWatermarkBytes(g_measTask)));
    }
  }
  g_loopPrevMs = now;
  g_loopLastMs = now;

  const bool portalPriority = shouldPrioritizeLocalPortal();

  if (cfg.commissioningMode) {
    serviceLocalPortal();
    if (portalPriority) {
      serviceLocalPortal();
    }
  }

  noteRuntimeStage("loop_policy");
  measurementNetworkPolicyTick();
  noteRuntimeStage("loop_measio");
  measurementFlushPendingBufferIfAny();
  const bool portalInteractive = localPortalInteractiveActive();
  if (portalInteractive && cfg.commissioningMode) {
    serviceLocalPortal();
  }

  // Link watchdog every ~1s
  static uint32_t t=0;
  if (millis()-t>1000) {
    noteRuntimeStage("loop_link");
    t = millis();
    linkWatchdog();
  }
  if (!portalInteractive) {
    noteRuntimeStage("loop_net");
    refreshInternetState();
    dhcpMaintainTick();
  }
  noteRuntimeStage("loop_adswd");
  adsWatchdog();
  noteRuntimeStage("loop_alarm");
  alarmsTask();
  if (!portalInteractive) {
    noteRuntimeStage("loop_ntp");
    ntpSyncTick();
  }
  noteRuntimeStage("loop_auto");
  measurementAutoCycleTick();
  noteRuntimeStage("loop_health");
  runtimeHealthTick();

  // ---- Periodic cloud push ----
  if (cfg.cloudEnabled && !portalInteractive) {
    uint32_t now = millis();
    uint32_t periodMs = measurementDeferredIntervalMs(cfg.cloudPeriodS * 1000UL);
    if (localPortalClientConnected() && periodMs < CLOUD_PUSH_PORTAL_INTERVAL_MS) {
      periodMs = CLOUD_PUSH_PORTAL_INTERVAL_MS;
    }
    uint32_t due = g_lastPushMs + periodMs;
    if (now - g_lastPushMs >= periodMs) {
      bool netReady = (g_linkOk && g_internetOk);
      if (!netReady) {
        noteRuntimeStage("loop_net_force");
        refreshInternetState(true);
        netReady = (g_linkOk && g_internetOk);
      }
      if (netReady) {
        noteRuntimeStage("loop_cloud");
        pushCloudNow(true);     // includes readings
      } else if (now - g_lastCloudSkipLogMs > 30000UL) {
        logLine("[CLOUD] skip push: offline");
        g_lastCloudSkipLogMs = now;
      }
      g_lastPushMs = now;
    }
    // next-in seconds for /status
    if (now <= due) g_nextPushInS = (due - now) / 1000UL;
    else g_nextPushInS = 0;
  } else {
    g_nextPushInS = 0;
  }

  if (!portalInteractive) {
    noteRuntimeStage("loop_remote");
    remotePollTick();
  }

  if (g_pendingRemoteReboot && millis() >= g_pendingRemoteRebootAtMs) {
    g_pendingRemoteReboot = false;
    String reason = g_pendingRebootReason;
    g_pendingRebootReason = "";
    noteRuntimeStage("loop_reboot");
    if (reason.length()) logLine(String("[SYS] rebooting now: ") + reason);
    else logLine("[SYS] rebooting now");
    delay(100);
    ESP.restart();
  }

  // Measurement sampling is now handled entirely by meas_task_bin().

  noteRuntimeStage("loop_led");
  updateStatusLeds();
  if (portalPriority && cfg.commissioningMode) {
    serviceLocalPortal();
  }
  noteRuntimeStage("loop_idle");
}
