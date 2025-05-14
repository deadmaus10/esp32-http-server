#include <WiFi.h>
#include <WebServer.h>

#define POT_PIN 34     // GPI34 = ADC1_CH34 on ESP32-C3
#define R_SENSE 165.0 // 165 ohm resistor

const char* ssid = "Wokwi-GUEST";     // Replace with your own Wi-Fi SSID
const char* password = "";            // Replace with your Wi-Fi password

WebServer server(80);  // HTTP server on port 80
unsigned long lastRead = 0;
int adcValue = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // 12-bit ADC (0-4095)

  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // JSON endpoint with ADC, voltage, current and distance
  server.on("/adc", []() {
    adcValue = analogRead(POT_PIN);
    float voltage = adcValue * 3.3 / 4095.0;
    float current_mA = voltage / R_SENSE * 1000.0; // I = V/R
    float distance_mm = (current_mA - 4.0) * (40.0 / 16.0); // scale 4–20 mA to 0–40 mm

    // Clamp values for safety
    if (current_mA < 4.0) current_mA = 4.0;
    if (current_mA > 20.0) current_mA = 20.0;
    if (distance_mm < 0) distance_mm = 0;
    if (distance_mm > 40) distance_mm = 40;

    String json = "{";
    json += "\"adc\": " + String(adcValue);
    json += ", \"voltage\": " + String(voltage, 3);
    json += ", \"current_mA\": " + String(current_mA, 2);
    json += ", \"distance_mm\": " + String(distance_mm, 2);
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
        <h1>4 20 mA Sensor Monitor</h1>
        <p>ADC Value: <span id="adc">-</span></p>
        <p>Voltage: <span id="voltage">-</span> V</p>
        <p>Current: <span id="current">-</span> mA</p>
        <p>Distance: <span id="distance">-</span> mm</p>
        <script>
          setInterval(() => {
            fetch('/adc')
              .then(response => response.json())
              .then(data => {
                document.getElementById('adc').textContent = data.adc;
                document.getElementById('voltage').textContent = data.voltage;
                document.getElementById('current').textContent = data.current_mA;
                document.getElementById('distance').textContent = data.distance_mm;
              });
          }, 150);
        </script>
      </body>
      </html>
    )rawliteral");
  });

  server.begin();
}

void loop() {
  server.handleClient();
}
