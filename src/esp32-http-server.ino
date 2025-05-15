// 📦 Final integrated ESP32 DevKitC sketch with fallback AP mode
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1
#define SDA_PIN 21
#define SCL_PIN 22
#define POT_PIN 34
#define R_SENSE 165.0
#define BTN_NEXT 32
#define BTN_SELECT 33

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);
Preferences prefs;

const char* menuItems[] = {"Wi-Fi Setup", "Sensor Monitor", "Restart", "Factory Reset"};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
int menuIndex = 0;
bool wifiReady = false;
String selectedSSID, wifiPassword;
bool apMode = false;

void renderMenu();
void executeMenuAction(int index);
String askForPassword();
void wifiSetupUI();
void sensorMonitorUI();
void saveWiFiCredentials(String ssid, String pass);
void loadWiFiCredentials();
void factoryReset();
void startFallbackAP();

void setup() {
  Serial.begin(115200);
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

  if (wifiReady) {
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
  }

  renderMenu();
  server.on("/adc", []() {
    int adc = analogRead(POT_PIN);
    float voltage = adc * 3.3 / 4095.0;
    float current = voltage / R_SENSE * 1000.0;
    float distance = (current - 4.0) * (40.0 / 16.0);
    distance = constrain(distance, 0, 40);
    String json = "{";
    json += "\"adc\":" + String(adc);
    json += ",\"voltage\":" + String(voltage, 2);
    json += ",\"current\":" + String(current, 2);
    json += ",\"distance\":" + String(distance, 2);
    json += "}";
    server.send(200, "application/json", json);
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
  else if (index == 2) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Restarting...");
    display.display();
    delay(1000);
    ESP.restart();
  }
  else if (index == 3) factoryReset();
  renderMenu();
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
        wifiPassword = "";  // open network, skip password input
      } else {
        wifiPassword = askForPassword();
      }
      saveWiFiCredentials(selectedSSID, wifiPassword);
      return;
    }
  }
}

String askForPassword() {
  /* const char* chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*_-+=<>.?[]{}"; */
  const char* chars = "0123456789";
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
