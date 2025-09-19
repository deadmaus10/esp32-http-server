#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
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

#include "index_html.h"   // UI page

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

#define ADS_PREF_NS "ads"   // make sure you use this same namespace everywhere

Adafruit_ADS1115 ads;
Preferences adsPrefs;            // NVS namespace handle for ADS config
static bool adsReady = false;

static const int I2C_SDA = 21, I2C_SCL = 22;

// Selection: A0 / A1 / BOTH
enum AdsSel : uint8_t { ADS_SEL_A0=0, ADS_SEL_A1=1, ADS_SEL_BOTH=2 };
static AdsSel  g_adsSel       = ADS_SEL_A0;

static adsGain_t g_adsGain    = GAIN_ONE;   // ±4.096 V (20mA * 160Ω = 3.2V)
static int       g_adsRateSps = 250;        // SPS as plain int
static float     g_shuntOhms  = 160.0f;     // your shunt (Ω)

// LSB (mV per code) for ADS1115 by gain
static float adsLSB_mV(adsGain_t g){
  switch(g){
    case GAIN_TWOTHIRDS: return 0.1875f;
    case GAIN_ONE:       return 0.1250f;  // we use this
    case GAIN_TWO:       return 0.0625f;
    case GAIN_FOUR:      return 0.03125f;
    case GAIN_EIGHT:     return 0.015625f;
    case GAIN_SIXTEEN:   return 0.0078125f;
  }
  return 0.1250f;
}

// Per-channel engineering units config for mm
static float g_engFSmm[2]  = {40.0f, 40.0f};  // full-scale in mm (A0, A1) — selectable 40/80
static float g_engOffmm[2] = {0.0f,  0.0f };  // offset added after scaling (mm)

// Map 4–20 mA % to mm for channel ch (0=A0, 1=A1)
static float mapToMM(uint8_t ch, float pct){
  if (ch > 1) ch = 1;
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

// Apply current RAM settings to the ADS chip (no defaults here!)
static void adsApplyHW(){
  if (!adsReady) return;
  ads.setGain(g_adsGain);
  adsSetRateSps(ads, g_adsRateSps);
}

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

String apSsid;
const char* apPass = "changeme123";

byte ethMac[6] = { 0x02,0x11,0x22,0x33,0x55,0x77 };
IPAddress testHost(1,1,1,1);
const uint16_t testPort = 53;

struct AppCfg {
  String devName   = "esp32-device";
  String serverUrl = "http://example.com/ingest";
  String apiKey    = "";
  bool useStatic   = false;
  IPAddress ip     = IPAddress(0,0,0,0);
  IPAddress gw     = IPAddress(0,0,0,0);
  IPAddress mask   = IPAddress(255,255,255,0);
  IPAddress dns    = IPAddress(1,1,1,1);
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
bool otaInProgress = false;

static bool g_mdnsRunning = false;

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
// Save ALL fields, every time
static void adsConfigSave(){
  adsPrefs.putUChar("sel",   (uint8_t)g_adsSel);
  adsPrefs.putUChar("gain",  gainToCode(g_adsGain));
  adsPrefs.putInt(  "rate",  g_adsRateSps);
  adsPrefs.putFloat("shunt", g_shuntOhms);

  // engineering units
  adsPrefs.putFloat("fs0",   g_engFSmm[0]);
  adsPrefs.putFloat("fs1",   g_engFSmm[1]);
  adsPrefs.putFloat("off0",  g_engOffmm[0]);
  adsPrefs.putFloat("off1",  g_engOffmm[1]);

  Serial.println("[ADS] config saved to NVS");
}

// Load ALL fields (use sane defaults only here)
static inline float clampf(float v, float lo, float hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }

static void adsConfigLoad(){
  uint8_t selCode  = adsPrefs.getUChar("sel",  (uint8_t)ADS_SEL_A0);
  uint8_t gainCode = adsPrefs.getUChar("gain", (uint8_t)1); // GAIN_ONE
  int     rateSps  = adsPrefs.getInt( "rate",  250);
  float   shunt    = adsPrefs.getFloat("shunt", 160.0f);

  float fs0  = adsPrefs.getFloat("fs0",  40.0f);
  float fs1  = adsPrefs.getFloat("fs1",  40.0f);
  float off0 = adsPrefs.getFloat("off0", 0.0f);
  float off1 = adsPrefs.getFloat("off1", 0.0f);

  g_adsSel       = (AdsSel)selCode;
  g_adsGain      = codeToGain(gainCode);
  g_adsRateSps   = rateSps;
  g_shuntOhms    = shunt;

  g_engFSmm[0]   = clampf(fs0,  1.0f, 10000.0f);
  g_engFSmm[1]   = clampf(fs1,  1.0f, 10000.0f);
  g_engOffmm[0]  = clampf(off0, -100000.0f, 100000.0f);
  g_engOffmm[1]  = clampf(off1, -100000.0f, 100000.0f);

  Serial.printf("[ADS] loaded sel=%u gain=%u rate=%d shunt=%.3fΩ | FS=[%.1f,%.1f] off=[%.3f,%.3f]\n",
    selCode, gainCode, g_adsRateSps, g_shuntOhms, g_engFSmm[0], g_engFSmm[1], g_engOffmm[0], g_engOffmm[1]);
}

void adsInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // Fast-mode I2C

  if (!ads.begin(0x48, &Wire)) {
    adsReady = false;
    Serial.println("[ADS] not found @0x48");
    return;
  }
  ads.setGain(g_adsGain);
  adsSetRateSps(ads, g_adsRateSps); // robust across lib versions
  adsReady = true;
  Serial.printf("[ADS] ready (A0/A1), shunt=%.1fΩ, rate=%d SPS\n", g_shuntOhms, g_adsRateSps);
}

// Convert one channel to raw / mV / mA / % of 4–20 mA span
static void adsReadCh(uint8_t ch, int16_t &raw, float &mv, float &ma, float &pct){
  if (ch>3) ch=0;
  raw = ads.readADC_SingleEnded(ch);
  mv  = raw * adsLSB_mV(g_adsGain);               // mV
  ma  = (g_shuntOhms > 0.1f) ? (mv / g_shuntOhms) : 0.0f; // mA since mA = mV / Ω
  pct = ((ma - 4.0f) / 16.0f) * 100.0f;           // 4–20 mA → 0–100%
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
}

static String gainStr(){
  switch(g_adsGain){
    case GAIN_TWOTHIRDS: return "6.144";
    case GAIN_ONE:       return "4.096";
    case GAIN_TWO:       return "2.048";
    case GAIN_FOUR:      return "1.024";
    case GAIN_EIGHT:     return "0.512";
    default:             return "0.256";
  }
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
  cfg.devName   = prefs.getString("devName", cfg.devName);
  cfg.serverUrl = prefs.getString("serverUrl", cfg.serverUrl);
  cfg.apiKey    = prefs.getString("apiKey", cfg.apiKey);
  cfg.useStatic = prefs.getBool  ("useStatic", cfg.useStatic);

  IPAddress t;
  if (parseIP(prefs.getString("ip",   "0.0.0.0"), t)) cfg.ip=t;
  if (parseIP(prefs.getString("gw",   "0.0.0.0"), t)) cfg.gw=t;
  if (parseIP(prefs.getString("mask", "255.255.255.0"), t)) cfg.mask=t;
  if (parseIP(prefs.getString("dns",  "1.1.1.1"), t)) cfg.dns=t;
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
  // try DHCP up to ~10 seconds
  unsigned long start = millis(); bool ok=false;
  while (millis()-start < 10000) {
    if (Ethernet.begin(ethMac)) { ok=true; break; }
    delay(500);
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

// ---- ROUTES: Config & Status ----
void handleRoot(){
  String page(FPSTR(INDEX_HTML));
  page.replace("%DEVNAME%", cfg.devName);
  page.replace("%SERVERURL%", cfg.serverUrl);
  page.replace("%APIKEY%", cfg.apiKey);
  page.replace("%DHCPSEL%", cfg.useStatic ? "" : "selected");
  page.replace("%STATICSEL%", cfg.useStatic ? "selected" : "");
  page.replace("%IPBOXDISP%", cfg.useStatic ? "block" : "none");
  page.replace("%IP%",   cfg.ip.toString());
  page.replace("%GW%",   cfg.gw.toString());
  page.replace("%MASK%", cfg.mask.toString());
  page.replace("%DNS%",  cfg.dns.toString());
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
  saveCfg();
  server.send(200,"application/json","{\"ok\":true}");
}
void handleReboot(){ server.send(200,"text/plain","Rebooting…"); delay(200); ESP.restart(); }
void handleStatus() {
  bool link = (Ethernet.linkStatus() == LinkON);
  bool inet = internetOK();
  // was: bool mdnsOk = MDNS.isRunning();
  String mdnsName = g_mdnsRunning ? (cfg.devName + ".local") : "off";

  String json = "{";
  json += "\"ethUp\":" + String((lastLink != Unknown) ? "true" : "false") + ",";
  json += "\"link\":"  + String(link ? "true" : "false") + ",";
  json += "\"inet\":"  + String(inet ? "true" : "false") + ",";
  json += "\"ip\":\""  + Ethernet.localIP().toString() + "\",";
  json += "\"sd\":"    + String(sdMounted ? "true" : "false") + ",";
  json += "\"time\":\""+ isoNow() + "\",";
  json += "\"timesynced\":" + String(g_timeSynced ? "true" : "false") + ",";
  json += "\"uptime\":\"" + uptimeStr() + "\",";
  json += "\"reboot\":\"" + resetReasonStr() + "\",";
  json += "\"mdns\":\"" + mdnsName + "\"";
  json += "}";
  server.sendHeader("Cache-Control","no-store");
  server.send(200, "application/json", json);
}

// ---- ROUTES: SD Browser ----
void handleFsList(){
  String path = safePath(urlDecode(server.arg("path")));
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
  String path = safePath(urlDecode(server.arg("path")));
  digitalWrite(WIZ_CS, HIGH);
  File f = SD.open(path, FILE_READ);
  if (!f || f.isDirectory()) { server.send(404,"text/plain","Not found"); return; }
  server.sendHeader("Content-Disposition","attachment; filename=\""+String(f.name())+"\"");
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
void handleDelete(){ String path = safePath(urlDecode(server.arg("path"))); bool ok = deleteRecursive(path); server.send(200,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}"); }
void handleMkdir(){ String path = safePath(urlDecode(server.arg("path"))); bool ok = SD.mkdir(path); server.send(200,"application/json",ok?"{\"ok\":true}":"{\"ok\":false}"); }
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
// GET /ads — single (A0/A1) or both
void handleAdsGet(){
  server.sendHeader("Cache-Control","no-store");
  if (!adsReady){ server.send(200,"application/json","{\"ok\":false,\"ready\":false}"); return; }

  AdsSel sel = g_adsSel;
  if (server.hasArg("sel")){
    String s = server.arg("sel");
    if      (s=="both") sel = ADS_SEL_BOTH;
    else if (s=="1")    sel = ADS_SEL_A1;
    else                sel = ADS_SEL_A0;
  } else if (server.hasArg("ch")){
    sel = (server.arg("ch").toInt()==1) ? ADS_SEL_A1 : ADS_SEL_A0;
  }

  String j = "{";
  j += "\"ok\":true,\"ready\":true,";
  j += "\"sel\":" + String((int)g_adsSel) + ",";                 // <--- add this line
  j += "\"gain\":\""+gainStr()+"\",";
  j += "\"rate\":"+String(g_adsRateSps)+",";
  j += "\"shunt\":"+String(g_shuntOhms,3)+",";
  j += "\"units\":{\"fsmm\":["+String(g_engFSmm[0],1)+","+String(g_engFSmm[1],1)+"],"
                    "\"offmm\":["+String(g_engOffmm[0],3)+","+String(g_engOffmm[1],3)+"]},";

  if (sel == ADS_SEL_BOTH){
    int16_t r0=0,r1=0; float mv0=0,mv1=0, ma0=0,ma1=0, p0=0,p1=0;
    adsReadCh(0,r0,mv0,ma0,p0);
    adsReadCh(1,r1,mv1,ma1,p1);
    float mm0 = mapToMM(0, p0);
    float mm1 = mapToMM(1, p1);
    j += "\"mode\":\"both\",\"readings\":[";
    j += "{\"ch\":0,\"raw\":"+String(r0)+",\"mv\":"+String(mv0,3)+",\"ma\":"+String(ma0,3)+",\"pct\":"+String(p0,1)+",\"mm\":"+String(mm0,2)+"},";
    j += "{\"ch\":1,\"raw\":"+String(r1)+",\"mv\":"+String(mv1,3)+",\"ma\":"+String(ma1,3)+",\"pct\":"+String(p1,1)+",\"mm\":"+String(mm1,2)+"}";
    j += "]}";
  } else {
    uint8_t ch = (sel==ADS_SEL_A1)?1:0;
    int16_t r=0; float mv=0,ma=0,p=0;
    adsReadCh(ch,r,mv,ma,p);
    float mm = mapToMM(ch, p);
    j += "\"mode\":\"single\",\"ch\":"+String(ch)+",";
    j += "\"raw\":"+String(r)+",\"mv\":"+String(mv,3)+",\"ma\":"+String(ma,3)+",\"pct\":"+String(p,1)+",\"mm\":"+String(mm,2)+"}";
  }
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
  if (!adsReady) { server.send(200,"application/json","{\"ok\":false,\"ready\":false}"); return; }

  // ----- selection -----
  if (server.hasArg("sel")) {
    String s = server.arg("sel"); s.trim(); s.toLowerCase();
    if      (s=="both") g_adsSel = ADS_SEL_BOTH;
    else if (s=="1" || s=="a1" || s=="ch1") g_adsSel = ADS_SEL_A1;
    else g_adsSel = ADS_SEL_A0;
  }

  // ----- gain -----
  if (server.hasArg("gain")) {
    adsGain_t g;
    String gs = server.arg("gain");
    if (gs=="6.144") g=GAIN_TWOTHIRDS;
    else if (gs=="4.096") g=GAIN_ONE;
    else if (gs=="2.048") g=GAIN_TWO;
    else if (gs=="1.024") g=GAIN_FOUR;
    else if (gs=="0.512") g=GAIN_EIGHT;
    else g=GAIN_SIXTEEN;
    g_adsGain = g;
  }

  // ----- rate (SPS) -----
  if (server.hasArg("rate")) {
    int sps = server.arg("rate").toInt();
    if (sps==8||sps==16||sps==32||sps==64||sps==128||sps==250||sps==475||sps==860)
      g_adsRateSps = sps;
  }

  // ----- shunt (Ohms) -----
  if (server.hasArg("shunt")) {
    float ohm = server.arg("shunt").toFloat();
    if (ohm > 1.0f && ohm < 10000.0f) g_shuntOhms = ohm;
  }

  // ----- units -----
  if (server.hasArg("type0")) g_engFSmm[0] = (server.arg("type0").indexOf("80")>=0) ? 80.0f : 40.0f;
  if (server.hasArg("type1")) g_engFSmm[1] = (server.arg("type1").indexOf("80")>=0) ? 80.0f : 40.0f;
  if (server.hasArg("fs0"))   g_engFSmm[0] = clampf(server.arg("fs0").toFloat(), 1.0f, 10000.0f);
  if (server.hasArg("fs1"))   g_engFSmm[1] = clampf(server.arg("fs1").toFloat(), 1.0f, 10000.0f);
  if (server.hasArg("off0"))  g_engOffmm[0]= clampf(server.arg("off0").toFloat(), -100000.0f, 100000.0f);
  if (server.hasArg("off1"))  g_engOffmm[1]= clampf(server.arg("off1").toFloat(), -100000.0f, 100000.0f);

  // ----- apply to HW + persist -----
  adsApplyHW();     // programs chip with new gain/rate immediately
  adsConfigSave();  // ALWAYS save so we never miss persistence

  // echo
  String j = "{";
  j += "\"ok\":true,";
  j += "\"sel\":" + String((int)g_adsSel) + ",";
  j += "\"gain\":\"" + gainToStr(g_adsGain) + "\",";
  j += "\"rate\":" + String(g_adsRateSps) + ",";
  j += "\"shunt\":" + String(g_shuntOhms,3) + ",";
  j += "\"fsmm\":[" + String(g_engFSmm[0],1) + "," + String(g_engFSmm[1],1) + "],";
  j += "\"offmm\":[" + String(g_engOffmm[0],3) + "," + String(g_engOffmm[1],3) + "]";
  j += "}";
  server.send(200,"application/json", j);
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
  String expected = urlDecode(server.arg("md5"));
  HTTPUpload &up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    otaInProgress = true;
    logLine("[OTA] Start: " + up.filename);
    if (expected.length()==32) {
      // If known ahead, we set it; Update will verify at end.
      Update.setMD5(expected.c_str());
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // choose next OTA slot
      logLine("[OTA] Begin failed: " + String(Update.errorString()));
      server.send(200,"application/json","{\"ok\":false,\"err\":\"begin\"}");
      otaInProgress = false; return;
    }
    mbedtls_md5_init(&md5ctx); mbedtls_md5_starts_ret(&md5ctx);

  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      logLine("[OTA] Write failed: " + String(Update.errorString()));
    }
    mbedtls_md5_update_ret(&md5ctx, up.buf, up.currentSize);

  } else if (up.status == UPLOAD_FILE_END) {
    mbedtls_md5_finish_ret(&md5ctx, nullptr); // we'll compute string separately
    // If expected MD5 was set via Update.setMD5, Update.end(true) checks it.
    bool ok = Update.end(expected.length()==32 /* even without, end will finalize */);
    char md5hex[33]; // also compute self-MD5 for reporting
    {
      // recompute with a fresh context using the already collected state is tricky,
      // so instead we track MD5 as we streamed: above we finished into a buffer
      // For simplicity, we use Update.md5String() which Arduino core exposes.
      String m = Update.md5String(); strncpy(md5hex, m.c_str(), 32); md5hex[32] = 0;
    }
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

  // register:
  server.on("/adsdump", HTTP_GET, handleAdsDump);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("[WEB] Portal ready");
}

// ---- SETUP/LOOP ----
void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println("\n=== Boot ===");

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

  if (Ethernet.linkStatus() == LinkON && internetOK()) {
    if (ntpSyncW5500("pool.ntp.org") || ntpSyncW5500("time.google.com")) {
      logLine("[TIME] NTP sync OK: " + isoNow());
    } else {
      logLine("[TIME] NTP sync FAILED; DNS=" + Ethernet.dnsServerIP().toString());
    }
  }
  setupTime();

  adsPrefs.begin(ADS_PREF_NS, false);  // 1) open NVS
  adsConfigLoad();                     // 2) pull config from NVS into globals
  adsInit();                           // 3) init ADS (ads.begin(...) sets adsReady)
  adsApplyHW();                        // 4) program gain/rate to chip from loaded globals

  // AP + web
  startApAndPortal();
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
  static uint32_t lastNtp=0;
  if (millis()-lastNtp > 3600000UL) {
    lastNtp = millis();
    if (Ethernet.linkStatus()==LinkON && internetOK()) ntpSyncW5500("pool.ntp.org");
  }
}
