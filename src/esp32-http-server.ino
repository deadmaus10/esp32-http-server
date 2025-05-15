// 📦 Final integrated ESP32 DevKitC sketch with fallback AP mode
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <time.h>
#include <FS.h>
#include <SPIFFS.h>
#include "miniz.h"

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1
#define SDA_PIN 21
#define SCL_PIN 22
#define POT_PIN 34
#define R_SENSE 165.0
#define BTN_NEXT 32
#define BTN_SELECT 33
#define MINIZ_HEADER_FILE_ONLY
#define MINIZ_NO_ARCHIVE_APIS 0
#define MINIZ_NO_ARCHIVE_WRITING_APIS 0
#define MINIZ_NO_STDIO 1  // optional, disables fopen etc.
#define Spiffs SPIFFS

struct SensorData {
  String timestamp;
  float voltage;
  float current;
  float distance;
};

const int GRAPH_WIDTH = 128;  // OLED width in pixels
float currentGraph[GRAPH_WIDTH] = {0};  // circular buffer

std::vector<SensorData> logBuffer;  // dynamic storage
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 150;  // in ms

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);
Preferences prefs;

const char* menuItems[] = {"Wi-Fi Setup", "Sensor Monitor", "Sensor Graph", "Start Session", "File Browser", "Restart", "Factory Reset"};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
int menuIndex = 0;
bool wifiReady = false;
String selectedSSID, wifiPassword;
String startTimestamp = "";
bool apMode = false;

bool sessionActive = false;
unsigned long sessionStartTime = 0;
unsigned long sessionDurationMs = 0;  // 0 = manual stop
bool manualStop = false;

void renderMenu();
void executeMenuAction(int index);
String askForPassword();
void wifiSetupUI();
void sensorMonitorUI();
void saveWiFiCredentials(String ssid, String pass);
void loadWiFiCredentials();
void factoryReset();
void startFallbackAP();
void initTime();

void setup() {
  Serial.begin(115200);

  // PRODUCTION = FALSE, TESTING = TRUE
  if (!Spiffs.begin(true)) {
    Serial.println("❌ Spiffs mount failed");
  } else {
    Serial.println("✅ Spiffs mounted");
  }

  analogReadResolution(12);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  loadWiFiCredentials();

  // REAL WIFI SETUP - NEED TO TEST ON REAL DEVICE
  /* if (wifiReady) {
    Serial.println("[WiFi] Connecting to: " + selectedSSID);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true); // clear old config
    delay(1000);
    WiFi.begin(selectedSSID.c_str(), wifiPassword.c_str());
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting to:");
    display.println(selectedSSID);
    display.display();

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();
    Serial.print("[WiFi] Status: ");
    Serial.println(WiFi.status());

    display.setCursor(0, 40);
    display.println(WiFi.status() == WL_CONNECTED ? "Wi-Fi OK" : "Failed");
    display.display();
    delay(1500);
  }

  if (WiFi.status() != WL_CONNECTED) {
    startFallbackAP();
  } */

  // DUMMY WIFI CONNECTION TO SIMULATE
  WiFi.disconnect(true);  // erase old config
  delay(500);
  WiFi.mode(WIFI_STA);    // important!
  WiFi.begin("Wokwi-GUEST");  // open network

  Serial.println("Connecting to Wokwi-GUEST...");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ Connected to Wokwi-GUEST");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    initTime();
  } else {
    Serial.println("❌ Failed to connect to Wokwi-GUEST");
    startFallbackAP();  // fallback to AP if needed
  }

  renderMenu();

  // JSON backend
  server.on("/adc", []() {
    int adc = analogRead(POT_PIN);
    float voltage = adc * 3.3 / 4095.0;
    float current = voltage / R_SENSE * 1000.0;
    float distance = (current - 4.0) * (40.0 / 16.0);
    distance = constrain(distance, 0, 40);

    // Get current time
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    char nowBuf[30];
    strftime(nowBuf, sizeof(nowBuf), "%Y-%m-%dT%H:%M:%SZ", timeinfo);

    // Create JSON response
    String json = "{";
    json += "\"timestamp\":\"" + String(nowBuf) + "\",";
    json += "\"started\":\"" + startTimestamp + "\",";
    json += "\"adc\":" + String(adc) + ",";
    json += "\"voltage\":" + String(voltage, 2) + ",";
    json += "\"current\":" + String(current, 2) + ",";
    json += "\"distance\":" + String(distance, 2);
    json += "}";

    server.send(200, "application/json", json);
  });

  // HTML dashboard
  server.on("/", []() {
    server.send(200, "text/html", R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head><title>ESP32 Sensor Dashboard</title></head>
      <body>
        <h1>4-20 mA Sensor Monitor</h1>
        <p>Start Time: <span id="starttime">-</span></p>
        <p>Last modified: <span id="timestamp">-</span></p>
        <p>ADC Value: <span id="adc">-</span></p>
        <p>Voltage: <span id="voltage">-</span> V</p>
        <p>Current: <span id="current">-</span> mA</p>
        <p>Distance: <span id="distance">-</span> mm</p>
        <hr>
        <button onclick="fetch('/clear').then(()=>alert('Log cleared'));">Clear Log</button>
        <button onclick="fetch('/save').then(()=>alert('Saved to /log.csv'));">Save Log</button>
        <a href="/export" download="log.csv"><button>Download CSV</button></a>

        <script>
          setInterval(() => {
            fetch('/adc')
              .then(response => response.json())
              .then(data => {
                document.getElementById('starttime').textContent = data.started;
                document.getElementById('timestamp').textContent = data.timestamp;
                document.getElementById('adc').textContent = data.adc;
                document.getElementById('voltage').textContent = data.voltage;
                document.getElementById('current').textContent = data.current;
                document.getElementById('distance').textContent = data.distance;
              });
          }, 150);
        </script>
      </body>
      </html>
    )rawliteral");
  });

  // SESSION LOG
  server.on("/log", []() {
    String json = "[";
    for (size_t i = 0; i < logBuffer.size(); i++) {
      const SensorData& d = logBuffer[i];
      json += "{";
      json += "\"timestamp\":\"" + d.timestamp + "\",";
      json += "\"voltage\":" + String(d.voltage, 2) + ",";
      json += "\"current\":" + String(d.current, 2) + ",";
      json += "\"distance\":" + String(d.distance, 2);
      json += "}";
      if (i < logBuffer.size() - 1) json += ",";
    }
    json += "]";
    server.send(200, "application/json", json);
  });

  // LOG CLEAR
  server.on("/clear", []() {
    logBuffer.clear();
    server.send(200, "text/plain", "Log cleared");
  });

  server.on("/save", []() {
    File file = Spiffs.open("/log.csv", FILE_WRITE);
    if (!file) {
      server.send(500, "text/plain", "Failed to open file for writing");
      return;
    }

    file.println("Timestamp,Voltage (V),Current (mA),Distance (mm)");
    for (const auto& d : logBuffer) {
      file.printf("%s,%.2f,%.2f,%.2f\n", d.timestamp.c_str(), d.voltage, d.current, d.distance);
    }

    file.close();
    server.send(200, "text/plain", "Log saved to /log.csv");
  });

  server.on("/export", []() {
    File file = Spiffs.open("/log.csv", FILE_READ);
    if (!file) {
      server.send(404, "text/plain", "File not found");
      return;
    }

    server.sendHeader("Content-Type", "text/csv");
    server.sendHeader("Content-Disposition", "attachment; filename=log.csv");
    server.sendHeader("Connection", "close");
    server.streamFile(file, "text/csv");
    file.close();
  });

  server.on("/files", []() {
    File root = Spiffs.open("/");
    if (!root || !root.isDirectory()) {
      server.send(500, "text/plain", "Spiffs not available");
      return;
    }

    String html = R"(
      <h1>Saved Session Logs</h1>
      <button onclick="window.location='/zip'">📦 Download All as ZIP</button>
      <ul>
    )";

    File file = root.openNextFile();
    while (file) {
      String name = file.name();
      if (name.endsWith(".csv")) {
        html += "<li><a href='" + name + "' download>" + name + "</a> ";
        html += "<button onclick=\"if(confirm('Delete " + name + "?')){fetch('/delete?file=" + name + "').then(()=>location.reload())}\">🗑️ Delete</button>";
        html += "</li>";
      }
      file = root.openNextFile();
    }
    html += "</ul>";
    server.send(200, "text/html", html);
  });

  server.onNotFound([]() {
    String path = server.uri();
    if (path.endsWith(".csv")) {
      File file = Spiffs.open(path, FILE_READ);
      if (file) {
        server.sendHeader("Content-Type", "text/csv");
        server.sendHeader("Content-Disposition", "attachment; filename=" + path.substring(1));
        server.sendHeader("Connection", "close");
        server.streamFile(file, "text/csv");
        file.close();
        return;
      }
    }

    server.send(404, "text/plain", "Not Found");
  });

  server.on("/delete", []() {
    if (!server.hasArg("file")) {
      server.send(400, "text/plain", "Missing file parameter");
      return;
    }
    String filename = server.arg("file");
    if (!Spiffs.exists(filename)) {
      server.send(404, "text/plain", "File not found");
      return;
    }

    if (Spiffs.remove(filename)) {
      server.send(200, "text/plain", "Deleted: " + filename);
    } else {
      server.send(500, "text/plain", "Failed to delete: " + filename);
    }
  });

  server.on("/zip", []() {
    #define ZIP_BUFFER_SIZE 8192
    mz_zip_archive zip;
    memset(&zip, 0, sizeof(zip));
    if (!mz_zip_writer_init_heap(&zip, 0, 0)) {
      server.send(500, "text/plain", "Failed to init zip");
      return;
    }

    File root = Spiffs.open("/");
    File file = root.openNextFile();
    while (file) {
      String name = file.name();
      if (name.endsWith(".csv")) {
        size_t len = file.size();
        char* content = new char[len];
        file.read((uint8_t*)content, len);
        mz_zip_writer_add_mem(&zip, name.c_str() + 1, content, len, MZ_BEST_COMPRESSION);
        delete[] content;
      }
      file = root.openNextFile();
    }

    void* pBuf = nullptr;
    size_t buf_size = 0;
    if (!mz_zip_writer_finalize_heap_archive(&zip, &pBuf, &buf_size)) {
      server.send(500, "text/plain", "Failed to finalize zip");
      mz_zip_writer_end(&zip);
      return;
    }

    server.sendHeader("Content-Type", "application/zip");
    server.sendHeader("Content-Disposition", "attachment; filename=logs.zip");
    server.send_P(200, "application/zip", (const char*)pBuf, buf_size);

    mz_zip_writer_end(&zip);
    mz_free(pBuf);
  });

  server.begin();
}

void loop() {
  server.handleClient();
  static bool nextPressed = false, selectPressed = false;

  if (digitalRead(BTN_NEXT) == LOW && !nextPressed) {
    nextPressed = true;
    menuIndex = (menuIndex + 1) % menuLength;
    renderMenu();
  }
  if (digitalRead(BTN_NEXT) == HIGH) nextPressed = false;

  if (digitalRead(BTN_SELECT) == LOW && !selectPressed) {
    selectPressed = true;
    executeMenuAction(menuIndex);
  }
  if (digitalRead(BTN_SELECT) == HIGH) selectPressed = false;

  delay(10);

  if (sessionActive) {
  unsigned long elapsed = millis() - sessionStartTime;
  unsigned long remaining = (sessionDurationMs > 0) ? sessionDurationMs - elapsed : 0;

  // Stop if timed session completes
  if (sessionDurationMs > 0 && elapsed >= sessionDurationMs) {
    sessionActive = false;

    // ⛔ Check if Spiffs is mounted first (for safety)
    if (Spiffs.begin(true)) {
      String filename = getCurrentTimestampFilename();
      File file = Spiffs.open(filename, FILE_WRITE);
      if (file) {
        file.println("Timestamp,Voltage (V),Current (mA),Distance (mm)");
        for (const auto& d : logBuffer) {
          file.printf("%s,%.2f,%.2f,%.2f\n", d.timestamp.c_str(), d.voltage, d.current, d.distance);
        }
        file.close();
        Serial.println("✅ Session saved to: " + filename);
      } else {
        Serial.println("❌ Failed to save session log.");
      }
    } else {
      Serial.println("❌ Spiffs not mounted");
    }

    logBuffer.clear();  // ✅ important: clear buffer

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Session complete!");
    display.display();
    delay(2000);
    renderMenu();
    return;
  }

    if (millis() - lastSampleTime >= sampleInterval) {
      lastSampleTime = millis();

      int adc = analogRead(POT_PIN);
      float voltage = adc * 3.3 / 4095.0;
      float current = voltage / R_SENSE * 1000.0;
      float distance = (current - 4.0) * (40.0 / 16.0);
      distance = constrain(distance, 0, 40);

      time_t now = time(nullptr);
      struct tm* timeinfo = localtime(&now);
      char nowBuf[30];
      strftime(nowBuf, sizeof(nowBuf), "%Y-%m-%dT%H:%M:%SZ", timeinfo);

      SensorData entry;
      entry.timestamp = String(nowBuf);
      entry.voltage = voltage;
      entry.current = current;
      entry.distance = distance;

      logBuffer.push_back(entry);

      // Optional: limit buffer size to avoid memory overflow
      if (logBuffer.size() > 1000) {
        logBuffer.erase(logBuffer.begin());  // remove oldest entry
      }
    }

    // Display session progress
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Sensoring in progress");
    unsigned long displayTime = (manualStop ? elapsed : remaining);
    int h = displayTime / 3600000;
    int m = (displayTime % 3600000) / 60000;
    int s = (displayTime % 60000) / 1000;
    display.printf("%02d:%02d:%02d %s\n", h, m, s, manualStop ? "elapsed" : "left");
    display.println("Hold OK to stop");
    display.display();

    // Manual stop handler
    unsigned long holdStart = millis();
    while (digitalRead(BTN_SELECT) == LOW) {
      if (millis() - holdStart > 1000) {
        sessionActive = false;

        // ⛔ Check if Spiffs is mounted first (for safety)
        if (Spiffs.begin(true)) {
          String filename = getCurrentTimestampFilename();
          File file = Spiffs.open(filename, FILE_WRITE);
          if (file) {
            file.println("Timestamp,Voltage (V),Current (mA),Distance (mm)");
            for (const auto& d : logBuffer) {
              file.printf("%s,%.2f,%.2f,%.2f\n", d.timestamp.c_str(), d.voltage, d.current, d.distance);
            }
            file.close();
            Serial.println("✅ Session saved to: " + filename);
          } else {
            Serial.println("❌ Failed to save session log.");
          }
        } else {
          Serial.println("❌ Spiffs not mounted");
        }

        logBuffer.clear();  // ✅ important: clear buffer

        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Session stopped!");
        display.display();
        delay(2000);
        renderMenu();
        return;
      }
    }

  delay(200);
  return;  // skip regular UI
  }
}

void renderMenu() {
  display.clearDisplay();
  display.setCursor(0, 0);
  for (int i = 0; i < menuLength; i++) {
    display.print(i == menuIndex ? "> " : "  ");
    display.println(menuItems[i]);
  }
  display.display();
}

void executeMenuAction(int index) {

  if (index == 0) wifiSetupUI();
  else if (index == 1) sensorMonitorUI();
  else if (index == 2) sensorCurrentGraphUI();
  else if (index == 3) startSensorSessionMenu();
  else if (index == 4) fileBrowserUI();
  else if (index == 5) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Restarting...");
    display.display();
    delay(1000);
    ESP.restart();
  }
  else if (index == 6) factoryReset();
  renderMenu();
}

void drawCurrentGraph() {
  display.clearDisplay();

  // Title
  display.setCursor(0, 0);
  display.println("Current (mA)");

  // Draw Y-axis markers (optional)
  display.setCursor(0, 8);
  display.print("20");

  display.setCursor(0, OLED_HEIGHT - 8);
  display.print(" 4");

  // Draw graph (skip first pixel)
  for (int x = 1; x < GRAPH_WIDTH; x++) {
    int y1 = map(currentGraph[x - 1], 4, 20, OLED_HEIGHT - 1, 16);
    int y2 = map(currentGraph[x],     4, 20, OLED_HEIGHT - 1, 16);
    display.drawLine(x - 1, y1, x, y2, SSD1306_WHITE);
  }

  display.display();
}

void sensorCurrentGraphUI() {
  while (true) {
    int raw = analogRead(POT_PIN);
    float voltage = raw * 3.3 / 4095.0;
    float current = voltage / R_SENSE * 1000.0;
    current = constrain(current, 4.0, 20.0);  // for clean graph scaling

    // Scroll left
    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
      currentGraph[i] = currentGraph[i + 1];
    }
    currentGraph[GRAPH_WIDTH - 1] = current;

    drawCurrentGraph();

    // Exit on long press
    unsigned long hold = millis();
    while (digitalRead(BTN_SELECT) == LOW) {
      if (millis() - hold > 1000) return;
    }

    delay(150);  // ~6.7 samples/sec
  }
}

void wifiSetupUI() {
  int networkIndex = 0;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // ensures a clean scan
  delay(100);
  int n = WiFi.scanNetworks();
  if (n == 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("No networks found");
    display.display(); delay(2000); return;
  }
  while (true) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Choose network:");
    display.println(WiFi.SSID(networkIndex));
    display.display();
    if (digitalRead(BTN_NEXT) == LOW) {
      delay(200);
      networkIndex = (networkIndex + 1) % n;
    }
    if (digitalRead(BTN_SELECT) == LOW) {
      selectedSSID = WiFi.SSID(networkIndex);
      if (WiFi.encryptionType(networkIndex) == WIFI_AUTH_OPEN) {
        wifiPassword = "";
      } else {
        wifiPassword = askForPassword();
      }
      saveWiFiCredentials(selectedSSID, wifiPassword);
      return;
    }
  }
}

String askForPassword() {
  const char* chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*_-+=<>.?[]{}";
  int charIndex = 0; String password = "";
  while (true) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Pass: "); display.println(password + "_");
    display.setCursor(0, 20);
    display.print("> ");
    if (charIndex < strlen(chars)) display.println(chars[charIndex]);
    else if (charIndex == strlen(chars)) display.println("[BACK]");
    else display.println("[OK]");
    display.display();
    if (digitalRead(BTN_NEXT) == LOW) {
      delay(150);
      charIndex = (charIndex + 1) % (strlen(chars) + 2);
    }
    if (digitalRead(BTN_SELECT) == LOW) {
      delay(150);
      if (charIndex < strlen(chars)) password += chars[charIndex];
      else if (charIndex == strlen(chars)) password.remove(password.length() - 1);
      else return password;
    }
  }
}

void sensorMonitorUI() {
  while (true) {
    int raw = analogRead(POT_PIN);
    float voltage = raw * 3.3 / 4095.0;
    float current = voltage / R_SENSE * 1000.0;
    float distance = (current - 4.0) * (40.0 / 16.0);
    distance = constrain(distance, 0, 40);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Sensor Monitor");
    display.printf("I: %.2f mA\n", current);
    display.printf("U: %.2f V\n", voltage);
    display.printf("d: %.2f mm\n", distance);
    display.println("Hold OK to exit");
    display.display();
    unsigned long hold = millis();
    while (digitalRead(BTN_SELECT) == LOW) {
      if (millis() - hold > 1000) return;
    }
    delay(200);
  }
}

void saveWiFiCredentials(String ssid, String pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
  wifiReady = true;
}

void loadWiFiCredentials() {
  prefs.begin("wifi", false);
  selectedSSID = "";
  wifiPassword = "";

  if (prefs.isKey("ssid")) {
    selectedSSID = prefs.getString("ssid");
    Serial.println("[WiFi] Loaded SSID: " + selectedSSID);
  } else {
    Serial.println("[WiFi] No saved SSID");
  }

  if (prefs.isKey("pass")) {
    wifiPassword = prefs.getString("pass");
    Serial.print("[WiFi] Loaded Password: ");
    Serial.println(wifiPassword.length() ? "(hidden)" : "(none)");
  } else {
    Serial.println("[WiFi] No saved password");
  }

  prefs.end();
  wifiReady = (selectedSSID.length() > 0);
}

void factoryReset() {
  prefs.begin("wifi", false);
  prefs.clear();
  prefs.end();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Preferences cleared");
  display.display();
  delay(1500);
  ESP.restart();
}

void startFallbackAP() {
  Serial.print("SSID: "); Serial.println(WiFi.softAPSSID());
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());
  apMode = true;
  WiFi.softAP("ESP32-Setup", "12345678");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("SSID: "); Serial.println(WiFi.softAPSSID());
  Serial.print("IP: "); Serial.println(IP);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Fallback AP Mode");
  display.println(WiFi.softAPSSID());
  display.print("IP: ");
  display.println(IP);
  display.display();
}

void initTime() {
  configTime(0, 0, "pool.ntp.org");  // UTC
  Serial.print("Waiting for NTP time sync");
  while (time(nullptr) < 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", timeinfo);
  startTimestamp = String(buf);
  Serial.println("Start time: " + startTimestamp);
}

void startSensorSessionMenu() {
  const char* options[] = {"2 hours", "4 hours", "8 hours", "Manual Stop"};
  const unsigned long durations[] = {7200000, 14400000, 28800000, 0};  // in ms
  int sel = 0;
  while (true) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Select duration:");
    display.println(sel == 0 ? "> 2 hours" :
                    sel == 1 ? "> 4 hours" :
                    sel == 2 ? "> 8 hours" : "> Manual Stop");
    display.display();
    if (digitalRead(BTN_NEXT) == LOW) {
      delay(200);
      sel = (sel + 1) % 4;
    }
    if (digitalRead(BTN_SELECT) == LOW) {
      delay(200);
      sessionActive = true;
      sessionDurationMs = durations[sel];
      sessionStartTime = millis();
      manualStop = (sessionDurationMs == 0);
      return;
    }
  }
}

String getCurrentTimestampFilename() {

  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char buf[32];
  strftime(buf, sizeof(buf), "/log-%Y-%m-%dT%H-%M-%S.csv", timeinfo);
  return String(buf);
}

void fileBrowserUI() {
  File root = Spiffs.open("/");
  if (!root || !root.isDirectory()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SPIFFS error");
    display.display();
    delay(2000);
    return;
  }

  std::vector<String> files;
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory() && String(file.name()).endsWith(".csv")) {
      files.push_back(String(file.name()));
    }
    file = root.openNextFile();
  }

  if (files.empty()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("No log files found");
    display.display();
    delay(2000);
    return;
  }

  int index = 0;

  while (true) {
    String fname = files[index];
    File f = Spiffs.open(fname);
    size_t fsize = f.size();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("> %s\n", fname.c_str());

    // 🧾 Show file size
    display.printf("Size: %dB\n", (int)fsize);

    // 📈 Preview first 2 lines (max ~40 chars total)
    String preview = "";
    while (f.available() && preview.length() < 40) {
      char c = f.read();
      if (c == '\r') continue;
      if (c == '\n') preview += "\\n";
      else preview += c;
    }
    f.close();
    display.setCursor(0, 30);
    display.print(preview.substring(0, 20));
    if (preview.length() > 20) {
      display.setCursor(0, 40);
      display.print(preview.substring(20));
    }

    display.setCursor(0, 54);
    display.print("NEXT=Scroll OK=Menu");

    display.display();

    // NEXT to scroll
    if (digitalRead(BTN_NEXT) == LOW) {
      delay(200);
      index = (index + 1) % files.size();
    }

    // SELECT to enter submenu
    if (digitalRead(BTN_SELECT) == LOW) {
      delay(200);
      int subSel = 0;
      while (true) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.printf("File: %s\n\n", fname.c_str());
        display.println(subSel == 0 ? "> Cancel" : "  Cancel");
        display.println(subSel == 1 ? "> Delete" : "  Delete");
        display.display();

        if (digitalRead(BTN_NEXT) == LOW) {
          delay(200);
          subSel = (subSel + 1) % 2;
        }
        if (digitalRead(BTN_SELECT) == LOW) {
          delay(200);
          if (subSel == 1) {
            // ✅ Confirm deletion
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Hold OK to confirm");
            display.println("DELETE:");
            display.println(fname);
            display.display();

            unsigned long hold = millis();
            while (digitalRead(BTN_SELECT) == LOW) {
              if (millis() - hold > 1000) {
                if (Spiffs.remove(fname)) {
                  display.clearDisplay();
                  display.setCursor(0, 0);
                  display.println("✅ Deleted");
                  display.display();
                  delay(1000);
                  files.erase(files.begin() + index);
                  if (files.empty()) return;
                  index = index % files.size();
                } else {
                  display.clearDisplay();
                  display.setCursor(0, 0);
                  display.println("❌ Failed to delete");
                  display.display();
                  delay(1000);
                }
                break;
              }
            }
          }
          break;  // exit submenu
        }
      }
    }

    delay(10);
  }
}
