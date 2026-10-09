// Exercise the production OTA flow with simulated network, flash and crypto APIs.
// Crypto internals are mocked: these tests verify that activation is gated on the
// library's digest result, complete download, bounds, and successful flash writes.
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include "../src/remote_maintenance_logic.h"
class String : public std::string {
public:
  String() = default;
  String(const char* s) : std::string(s) {}
  String(const std::string& s) : std::string(s) {}
  String(unsigned n) : std::string(std::to_string(n)) {}
  String(int n) : std::string(std::to_string(n)) {}
  bool startsWith(const String& p) const { return compare(0, p.size(), p) == 0; }
  int indexOf(char c) const { auto p = find(c); return p == npos ? -1 : static_cast<int>(p); }
  String substring(size_t start) const { return substr(start); }
  String substring(size_t start, size_t end) const { return substr(start, end - start); }
  void trim() { auto a = find_first_not_of(" \t\r\n"); auto b = find_last_not_of(" \t\r\n"); *this = a == npos ? "" : substr(a,b-a+1); }
  void toLowerCase() { std::transform(begin(),end(),begin(),[](unsigned char c){return std::tolower(c);}); }
};
uint32_t clockMs = 0;
uint32_t millis() { return ++clockMs; }
void delay(unsigned n) { clockMs += n; }
struct ScopedSpiBusLock { explicit operator bool() const { return true; } };
struct File {
  explicit operator bool() const { return false; }
  size_t size() const { return 0; }
  int read(uint8_t*, size_t) { return 0; }
  size_t write(const uint8_t*, size_t n) { return n; }
  void close() {}
};
struct FakeSd {
  bool exists(const String&) { return false; }
  bool mkdir(const String&) { return true; }
  bool remove(const String&) { return true; }
  File open(const String&, int) { return {}; }
} SD;
constexpr int FILE_READ = 0, FILE_WRITE = 1, U_FLASH = 0;
constexpr int MAX_LOG_FILES = 5;
constexpr size_t MAX_LOG_SIZE = 512*1024;
struct { bool cloudEnabled=true; String apiKey="key", serverUrl="https://dashboard.albasqueeze.com"; } cfg;
bool g_measActive=false, g_measAutoRestartPending=false, g_measAutoRestartWaitingUpload=false;
bool otaInProgress=false, g_pendingRemoteReboot=false;
void* g_measTask=nullptr;
unsigned g_bootCount=1;
String g_restartHistory="[]";
String cloudIngestUrlFromServerUrl() { return cfg.serverUrl; }
String isoNowFileSafe() { return "capture"; }
String isoNow() { return "time"; }
String jsonEscape(const String& s) { return s; }
String ramLogText() { return "log"; }
String urlEncode(const String& s) { return s; }
bool httpsPostJson(const String&, const String&, const String&, int&, String&, String&) { return true; }
bool storageCanUseSd() { return false; }
void prepareSdSpi() {}
void prepareEthernetSpi() {}
bool uploadCloudFilePath(const String&, const String&) { return true; }
void noteNetOp(const String&) {}
void noteRuntimeStage(const String&) {}
void noteNetResult(bool, const String& = "") {}
String suppliedPath, suppliedHash;
bool jsonGetStringField(const String&, const char* key, String& out) {
  out = strcmp(key,"path") == 0 ? suppliedPath : suppliedHash;
  return true;
}
struct esp_partition_t { uint32_t size=1310720; } partition;
const esp_partition_t* esp_ota_get_next_update_partition(void*) { return &partition; }
struct FakeTls {
  std::vector<uint8_t> incoming;
  size_t offset=0;
  bool stopped=false, hang=false;
  int available() { return static_cast<int>(incoming.size()-offset); }
  int read() { return available() ? incoming[offset++] : -1; }
  int read(uint8_t* out, size_t n) {
    n=std::min(n,incoming.size()-offset); std::memcpy(out,incoming.data()+offset,n);offset+=n;return static_cast<int>(n);
  }
  bool connected() { return !stopped && (available() || hang); }
  size_t print(const String& s) { return s.size(); }
  void stop() { stopped=true; }
} _tls;
bool tlsConnectHost(const String&, uint16_t, String&) { return true; }
bool parseUrl(const String&, String& scheme, String& host, uint16_t& port, String& path) {
  scheme="https";host="dashboard.albasqueeze.com";port=443;path="/";return true;
}
struct FakeUpdate {
  bool begun=false, aborted=false, activated=false, writeFail=false, finalizeFail=false;
  size_t expected=0,written=0;
  bool begin(size_t n, int) { begun=true;expected=n;return true; }
  size_t write(const uint8_t*, size_t n) { if(writeFail)return 0;written+=n;return n; }
  void abort() { aborted=true;begun=false; }
  bool end(bool acceptIncomplete) {
    assert(!acceptIncomplete); assert(written==expected);
    activated=!finalizeFail;return activated;
  }
  const char* errorString() { return "simulated"; }
} Update;
struct mbedtls_sha256_context {};
void mbedtls_sha256_init(mbedtls_sha256_context*) {}
void mbedtls_sha256_free(mbedtls_sha256_context*) {}
int mbedtls_sha256_starts_ret(mbedtls_sha256_context*, int) { return 0; }
int mbedtls_sha256_update_ret(mbedtls_sha256_context*, const uint8_t*, size_t) { return 0; }
int mbedtls_sha256_finish_ret(mbedtls_sha256_context*, uint8_t* digest) { std::memset(digest,0,32);return 0; }
String toHexLower(const uint8_t*, size_t) { return std::string(64,'0'); }
void scheduleDeferredReboot(const String&, uint32_t) { g_pendingRemoteReboot=true; }
#include "../src/remote_maintenance_impl.h"
void setupCase(const String& headers="Content-Length: 32\r\n", size_t bodySize=32) {
  clockMs=0; Update={};_tls={};otaInProgress=false;g_pendingRemoteReboot=false;
  suppliedPath="/firmware/test.bin";suppliedHash=std::string(64,'0');
  const String response="HTTP/1.1 200 OK\r\n"+headers+"\r\n";
  _tls.incoming.assign(response.begin(),response.end());
  if(bodySize) { _tls.incoming.push_back(0xE9);_tls.incoming.insert(_tls.incoming.end(),bodySize-1,0); }
}
void expectFailure(const char* expected) {
  String result;assert(!installRemoteFirmware("params",result));
  assert(result==expected);assert(!Update.activated);assert(!g_pendingRemoteReboot);assert(!otaInProgress);assert(_tls.stopped);
}
int main() {
  setupCase();String result;
  assert(remoteMaintenanceAllowed());g_measActive=true;assert(!remoteMaintenanceAllowed());g_measActive=false;
  assert(uploadRemoteLogs(result));assert(result=="logs_uploaded_ram_only");
  assert(installRemoteFirmware("params",result));
  assert(Update.activated && g_pendingRemoteReboot && !otaInProgress);
  setupCase();suppliedHash=std::string(64,'1');expectFailure("ota_sha256_mismatch");assert(Update.aborted);
  setupCase("Content-Length: 64\r\n");expectFailure("ota_download_truncated");assert(Update.aborted);
  setupCase("Content-Length: 1310721\r\n");expectFailure("ota_length_invalid");assert(!Update.begun);
  setupCase("Content-Length: 32\r\nTransfer-Encoding: chunked\r\n");expectFailure("ota_encoding_unsupported");
  setupCase("Content-Length: 32\r\nContent-Encoding: gzip\r\n");expectFailure("ota_encoding_unsupported");
  setupCase("Content-Length: 32\r\nContent-Length: 32\r\n");expectFailure("ota_length_invalid");
  setupCase();Update.writeFail=true;expectFailure("ota_flash_write_failed_simulated");assert(Update.aborted);
  setupCase();Update.finalizeFail=true;expectFailure("ota_finalize_failed_simulated");assert(Update.aborted);
  setupCase("Content-Length: 32\r\n",0);_tls.hang=true;expectFailure("ota_download_timeout");assert(Update.aborted);
  setupCase();_tls.incoming[0]='X';expectFailure("ota_http_status_invalid");
  setupCase();suppliedPath="/firmware/../bad.bin";
  assert(!installRemoteFirmware("params", result));assert(result=="ota_invalid_path_or_sha256");
  assert(!Update.begun && !Update.activated && !g_pendingRemoteReboot && !otaInProgress);
  // Before a connection is opened, invalid params return without touching the TLS object.
  std::cout << "12 production OTA flow scenarios passed\n";
}
