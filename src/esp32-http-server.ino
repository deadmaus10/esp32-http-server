#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>

// -------------------- PINS: ADAPT IF NEEDED --------------------
static const int WIZ_CS   = 5;    // WIZ850io CS  (change if different)
static const int WIZ_RST  = 27;   // WIZ850io RST (ACTIVE LOW)  << you said 27
static const int SD_CS    = 4;    // SD card CS   << you said 4

// VSPI (ESP32 default)
static const int VSPI_SCK  = 18;
static const int VSPI_MISO = 19;
static const int VSPI_MOSI = 23;
// ---------------------------------------------------------------

// Unique MAC for DHCP
byte mac[6] = { 0x02, 0x11, 0x22, 0x33, 0x55, 0x77 };

// Simple Internet reachability target (TCP open = OK)
IPAddress testHost(1, 1, 1, 1); // Cloudflare DNS
const uint16_t testPort = 53;

static inline void deselectAll() {
  pinMode(WIZ_CS, OUTPUT);
  pinMode(SD_CS,  OUTPUT);
  digitalWrite(WIZ_CS, HIGH);
  digitalWrite(SD_CS,  HIGH);
}

bool ethernetInit() {
  // Hardware reset the WIZ850io (active LOW), then wait ~50ms
  pinMode(WIZ_RST, OUTPUT);
  digitalWrite(WIZ_RST, LOW);
  delay(5);
  digitalWrite(WIZ_RST, HIGH);
  delay(60); // WIZ850io auto-config time (~50ms required)

  // Init shared SPI and tell Ethernet which CS to use
  SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, WIZ_CS);
  Ethernet.init(WIZ_CS);

  Serial.println("[ETH] Starting DHCP…");
  if (!Ethernet.begin(mac)) {
    Serial.println("[ETH] DHCP FAILED (router not giving IP?)");
    return false;
  }

  Serial.print("[ETH] IP: ");      Serial.println(Ethernet.localIP());
  Serial.print("[ETH] Gateway: "); Serial.println(Ethernet.gatewayIP());
  Serial.print("[ETH] DNS: ");     Serial.println(Ethernet.dnsServerIP());

  // Link status print
  auto hw = Ethernet.hardwareStatus();
  auto lk = Ethernet.linkStatus();
  Serial.print("[ETH] HW: ");
  Serial.println(hw == EthernetW5500 ? "W5500 OK" :
                 (hw == EthernetNoHardware ? "No HW" : "Other/Unknown"));

  Serial.print("[ETH] LINK: ");
  Serial.println(lk == LinkON ? "UP" : (lk == LinkOFF ? "DOWN" : "UNKNOWN"));
  return true;
}

bool internetOK(uint16_t timeoutMs = 2000) {
  EthernetClient c;
  c.setTimeout(timeoutMs);
  if (!c.connect(testHost, testPort)) return false;
  c.stop();
  return true;
}

void listDir(const char* dirname, uint8_t levels) {
  File root = SD.open(dirname);
  if (!root) { Serial.println("[SD] Cannot open root"); return; }
  if (!root.isDirectory()) { Serial.println("[SD] Not a directory"); root.close(); return; }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  <DIR> "); Serial.println(file.name());
      if (levels) {
        char sub[128];
        snprintf(sub, sizeof(sub), "%s/%s", dirname, file.name());
        listDir(sub, levels - 1);
      }
    } else {
      Serial.print("  "); Serial.print(file.name());
      Serial.print("  "); Serial.print(file.size()); Serial.println(" bytes");
    }
    file = root.openNextFile();
  }
  root.close();
}

bool sdInitAndTest() {
  Serial.println("[SD] Mounting…");

  // Make sure Ethernet is deselected during SD ops
  digitalWrite(WIZ_CS, HIGH);

  // SPI is already begun above; 1-arg begin is safest across cores
  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] Mount FAIL. Check wiring and 3.3V logic.");
    return false;
  }
  Serial.println("[SD] Mount OK");

  // Card info (ESP32 SD library)
  uint8_t type = SD.cardType();
  Serial.print("[SD] Type: ");
  if      (type == CARD_NONE) { Serial.println("No card"); return false; }
  else if (type == CARD_MMC)  Serial.println("MMC");
  else if (type == CARD_SD)   Serial.println("SDSC");
  else if (type == CARD_SDHC) Serial.println("SDHC/SDXC");
  else                        Serial.println("Unknown");

  uint64_t sizeMB = SD.cardSize() / (1024ULL * 1024ULL);
  Serial.printf("[SD] Size: %llu MB\n", (unsigned long long)sizeMB);

  // Write -> Read -> Verify
  const char* path = "/sd_test.txt";
  String msg = String("Hello SD! millis=") + String(millis());

  // WRITE
  digitalWrite(WIZ_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  File wf = SD.open(path, FILE_WRITE);
  if (!wf) { Serial.println("[SD] Open for write FAILED"); digitalWrite(SD_CS, HIGH); return false; }
  bool w = (wf.println(msg) > 0);
  wf.close();
  digitalWrite(SD_CS, HIGH);
  if (!w) { Serial.println("[SD] Write FAILED"); return false; }
  Serial.println("[SD] Write OK");

  // READ & VERIFY
  digitalWrite(WIZ_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  File rf = SD.open(path, FILE_READ);
  if (!rf) { Serial.println("[SD] Open for read FAILED"); digitalWrite(SD_CS, HIGH); return false; }
  String back = rf.readString();
  rf.close();
  digitalWrite(SD_CS, HIGH);

  back.trim(); msg.trim();
  Serial.print("[SD] Readback: "); Serial.println(back);
  bool ok = (back.endsWith(msg)); // last line match (println adds \r\n)
  Serial.println(ok ? "[SD] Verify OK" : "[SD] Verify FAIL");

  Serial.println("[SD] Root listing:");
  listDir("/", 1);
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32 + WIZ850io + SD full test ===");

  // Keep CS lines idle HIGH from the start
  deselectAll();

  // 1) Ethernet bring-up
  bool ethUp = ethernetInit();

  // If link + IP, try Internet
  if (ethUp && Ethernet.linkStatus() == LinkON) {
    Serial.print("[ETH] Internet: ");
    Serial.println(internetOK() ? "OK" : "NO");
  }

  // 2) SD test
  bool sdOk = sdInitAndTest();

  Serial.println("\n=== SUMMARY ===");
  Serial.print("ETH: ");
  Serial.println(ethUp ? "INIT OK" : "INIT FAIL");
  Serial.print("LINK: ");
  auto lk = Ethernet.linkStatus();
  Serial.println(lk == LinkON ? "UP" : (lk == LinkOFF ? "DOWN" : "UNKNOWN"));
  Serial.print("INET: ");
  Serial.println((ethUp && lk == LinkON && internetOK()) ? "OK" : "NO");
  Serial.print("SD: ");
  Serial.println(sdOk ? "R/W OK" : "FAIL");
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 3000) {
    t = millis();
    auto lk = Ethernet.linkStatus();
    Serial.print("[ETH] Link=");
    Serial.print(lk == LinkON ? "UP" : (lk == LinkOFF ? "DOWN" : "UNKNOWN"));
    Serial.print("  IP="); Serial.print(Ethernet.localIP());
    Serial.print("  Internet=");
    Serial.println(internetOK() ? "OK" : "NO");
  }
}
