/*
 * IoT Water Quality Monitor (ESP32)
 * Sensors : Turbidity (GPIO34, via divider), TDS (GPIO35), DS18B20 (GPIO4),
 *           HC-SR04 (TRIG 5 / ECHO 19 via divider), Float switch (GPIO27)
 * Outputs : OLED (I2C 21/22), LED (GPIO26), Buzzer (GPIO25)
 * Cloud   : MQTT (PubSubClient) -> public HiveMQ broker -> web dashboard (mqtt_dashboard.html)
 *
 * Libraries to install (Arduino IDE > Library Manager):
 *   OneWire, DallasTemperature, Adafruit SSD1306, Adafruit GFX, PubSubClient
 * Board: "ESP32 Dev Module"
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= USER SETTINGS =================
const char* WIFI_SSID  = "ArnaviPhone";
const char* WIFI_PASS  = "arnav1234";

// MQTT (public test broker: anyone who knows your topic can read it,
// so change the topic to something unique, and use the SAME one in the dashboard)
const char* MQTT_BROKER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;
const char* MQTT_TOPIC  = "vit-wqm-2026/data";

// Alert thresholds (tune these after testing with your own sensors)
const float TDS_LIMIT_PPM   = 500;   // above this = poor water
const float TURB_LIMIT_NTU  = 50;    // above this = too cloudy
const float TEMP_LIMIT_C    = 35;    // above this = too warm
const float TANK_HEIGHT_CM  = 20;    // distance from HC-SR04 down to tank bottom

const unsigned long READ_INTERVAL_MS = 2000;   // read sensors every 2 s
const unsigned long SEND_INTERVAL_MS = 5000;   // publish to MQTT every 5 s

// ================= PINS =================
#define PIN_TURB   34
#define PIN_TDS    35
#define PIN_DS18   4
#define PIN_FLOAT  27
#define PIN_TRIG   5
#define PIN_ECHO   19
#define PIN_BUZZER 25
#define PIN_LED    26

// ================= OBJECTS =================
OneWire oneWire(PIN_DS18);
DallasTemperature tempSensor(&oneWire);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ================= STATE =================
float turbVolt = 0, turbNTU = 0, tdsPPM = 0, tempC = 25, levelCm = -1;
bool  floatTriggered = false;
int   alertLevel = 0;               // 0 = normal, 1 = warning, 2 = critical
unsigned long lastRead = 0, lastSend = 0;

// ---------- Helpers ----------
// Average of 30 ADC samples (in millivolts), ignores the lowest/highest 5
float avgMilliVolts(int pin) {
  const int N = 30;
  int s[N];
  for (int i = 0; i < N; i++) { s[i] = analogReadMilliVolts(pin); delay(5); }
  for (int i = 0; i < N - 1; i++)
    for (int j = i + 1; j < N; j++)
      if (s[j] < s[i]) { int t = s[i]; s[i] = s[j]; s[j] = t; }
  long sum = 0;
  for (int i = 5; i < N - 5; i++) sum += s[i];
  return sum / float(N - 10);
}

float readTemperature() {
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) return NAN;   // sensor not detected
  return t;
}

float readTurbidityNTU() {
  float mv = avgMilliVolts(PIN_TURB);
  turbVolt = (mv / 1000.0) * 1.5;               // undo the 10k/20k divider (x1.5)
  // Approximate curve for common analog turbidity modules.
  // Clear water ~4.2 V -> 0 NTU. Calibrate for better accuracy.
  if (turbVolt >= 4.2) return 0;
  if (turbVolt < 2.5)  return 3000;
  float ntu = -1120.4 * turbVolt * turbVolt + 5742.3 * turbVolt - 4352.9;
  return ntu < 0 ? 0 : ntu;
}
float readTDS(float waterTempC) {
  float v = avgMilliVolts(PIN_TDS) / 1000.0;
  float coef = 1.0 + 0.02 * (waterTempC - 25.0);  // temperature compensation
  float cv = v / coef;
  float tds = (133.42 * cv * cv * cv - 255.86 * cv * cv + 857.39 * cv) * 0.5;
  return tds < 0 ? 0 : tds;
}

float readLevelCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long dur = pulseIn(PIN_ECHO, HIGH, 30000);
  if (dur == 0) return -1;                       // no echo
  float distance = dur * 0.0343 / 2.0;
  float level = TANK_HEIGHT_CM - distance;
  return constrain(level, 0, TANK_HEIGHT_CM);
}

// ---------- Main reading + alert logic ----------
void readAll() {
  float t = readTemperature();
  if (!isnan(t)) tempC = t;
  float tForComp = isnan(t) ? 25.0 : tempC;

  turbNTU = readTurbidityNTU();
  tdsPPM  = readTDS(tForComp);
  levelCm = readLevelCm();
  floatTriggered = (digitalRead(PIN_FLOAT) == LOW);  // LOW = float switch closed

  int bad = 0;
  if (tdsPPM  > TDS_LIMIT_PPM)  bad++;
  if (turbNTU > TURB_LIMIT_NTU) bad++;
  if (tempC   > TEMP_LIMIT_C)   bad++;
  if (floatTriggered)           bad++;
  alertLevel = (bad == 0) ? 0 : (bad == 1 ? 1 : 2);

  Serial.printf("Temp %.1fC | Turb %.0f NTU (%.2fV) | TDS %.0f ppm | Level %.1f cm | Float %d | Alert %d\n",
                tempC, turbNTU, turbVolt, tdsPPM, levelCm, floatTriggered, alertLevel);
}

// Non-blocking LED / buzzer patterns
void updateAlerts() {
  unsigned long now = millis();
  if (alertLevel == 0) {
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_BUZZER, LOW);
  } else if (alertLevel == 1) {                    // warning: LED solid
    digitalWrite(PIN_LED, HIGH);
    digitalWrite(PIN_BUZZER, LOW);
  } else {                                         // critical: LED + buzzer beeping
    bool on = (now / 300) % 2 == 0;
    digitalWrite(PIN_LED, on);
    digitalWrite(PIN_BUZZER, on);
  }
}

void updateOLED() {
  const char* status[] = {"NORMAL", "WARNING", "CRITICAL"};
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.printf("Water Quality: %s\n", status[alertLevel]);
  oled.printf("Temp : %.1f C\n", tempC);
  oled.printf("TDS  : %.0f ppm\n", tdsPPM);
  oled.printf("Turb : %.0f NTU\n", turbNTU);
  oled.printf("Level: %.1f cm\n", levelCm);
  oled.printf("Float: %s\n", floatTriggered ? "HIGH WATER" : "ok");
  oled.printf("MQTT : %s", mqtt.connected() ? "connected" : "offline");
  oled.display();
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("WiFi status code: %d\n", WiFi.status());
}

bool mqttReconnect() {
  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return false;
  String id = "wqm-esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  return mqtt.connect(id.c_str());
}

void publishData() {
  if (!mqtt.connected() && !mqttReconnect()) {
    Serial.println("MQTT offline, skipping publish");
    return;
  }
  char payload[200];
  snprintf(payload, sizeof(payload),
           "{\"temp\":%.1f,\"tds\":%.0f,\"turb\":%.0f,\"turbV\":%.2f,\"level\":%.1f,\"float\":%d,\"alert\":%d}",
           tempC, tdsPPM, turbNTU, turbVolt, levelCm, floatTriggered ? 1 : 0, alertLevel);
  bool ok = mqtt.publish(MQTT_TOPIC, payload, true);   // retained: dashboard shows last value instantly
  Serial.printf("MQTT publish %s: %s\n", ok ? "OK" : "FAILED", payload);
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  Serial.println("Networks visible to ESP32:");
  for (int i = 0; i < n; i++) Serial.println(WiFi.SSID(i));
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_FLOAT, INPUT_PULLUP);

  tempSensor.begin();

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED not found (check wiring / address)");
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Water Quality Monitor");
  oled.println("Connecting WiFi...");
  oled.display();

  connectWiFi();
  readAll();
  updateOLED();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setBufferSize(512);
  publishData();
  lastRead = lastSend = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastRead >= READ_INTERVAL_MS) { lastRead = now; readAll(); updateOLED(); }
  if (now - lastSend >= SEND_INTERVAL_MS) { lastSend = now; publishData(); }
  mqtt.loop();
  updateAlerts();
}
