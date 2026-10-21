#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <MAX30105.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// Device Wi-Fi and SIM800L recipient.
const char* WIFI_SSID = "iPhone";
const char* WIFI_PASSWORD = "12345678";
const char* ALERT_PHONE = "+918838846160";

const char* MQTT_HOST = "y717c21f.ala.eu-central-1.emqxsl.com";
const uint16_t MQTT_PORT = 8883;
const char* MQTT_USER = "ESP32";
const char* MQTT_PASSWORD = "ESP32";
const char* MQTT_CLIENT_ID = "ESP32";

constexpr int SIM_RX_PIN = 16;
constexpr int SIM_TX_PIN = 17;
constexpr int BUZZER_PIN = 25;
constexpr int OLED_WIDTH = 128;
constexpr int OLED_HEIGHT = 64;
constexpr uint8_t BUZZER_ON = LOW;
constexpr uint8_t BUZZER_OFF = HIGH;

constexpr char TOPIC_STATUS[] = "ESP32/status";
constexpr char TOPIC_TELEMETRY[] = "ESP32/telemetry";
constexpr char TOPIC_ALERT[] = "ESP32/alert";
constexpr char TOPIC_SETTINGS[] = "ESP32/settings";
constexpr char TOPIC_SET[] = "ESP32/set";
constexpr char TOPIC_COMMAND[] = "ESP32/command";

WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);
HardwareSerial sim800(2);
MPU6050 mpu(Wire);
MAX30105 pulseSensor;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Preferences preferences;

bool mpuReady = false;
bool pulseSensorReady = false;
bool oledReady = false;
bool hasLocation = false;
double savedLatitude = 0.0;
double savedLongitude = 0.0;
float rollLimit = 15.0;

bool accidentActive = false;
bool contactDetected = false;
bool callInProgress = false;
bool waitingForSmsBody = false;
String smsSender;
String simLine;
uint32_t callEndAt = 0;
uint32_t rolloverStartAt = 0;
uint32_t lastTelemetryAt = 0;
uint32_t lastStatusAt = 0;
uint32_t lastPulseSampleAt = 0;
uint32_t lastOledUpdateAt = 0;
uint32_t contactDetectedAt = 0;
uint32_t lastMqttAttemptAt = 0;
uint32_t lastWifiAttemptAt = 0;
bool wifiAttempted = false;
bool mqttAttempted = false;
bool wifiWasConnected = false;

float bpmSamples[4] = {};
uint8_t bpmSampleIndex = 0;
uint8_t bpmSampleCount = 0;
uint32_t lastBeatAt = 0;
float currentBpm = NAN;
float currentSpo2 = NAN;
bool currentSpo2Valid = false;

constexpr size_t OXIMETER_WINDOW = 100;
uint32_t redSamples[OXIMETER_WINDOW] = {};
uint32_t irSamples[OXIMETER_WINDOW] = {};
size_t oximeterSampleCount = 0;
size_t oximeterWriteIndex = 0;
float infraredBaseline = 0.0;
float previousInfraredDeviation = 0.0;

bool waitForSIM(const char* expected, uint32_t timeoutMs) {
  String response;
  const uint32_t startedAt = millis();
  while (millis() - startedAt < timeoutMs) {
    while (sim800.available()) {
      const char value = static_cast<char>(sim800.read());
      response += value;
      if (response.indexOf(expected) >= 0) {
        return true;
      }
      if (response.indexOf("ERROR") >= 0) {
        Serial.print("SIM800L error response: ");
        Serial.println(response);
        return false;
      }
    }
    delay(5);
  }
  Serial.print("SIM800L timed out waiting for: ");
  Serial.println(expected);
  return false;
}

bool sendSms(const String& text) {
  if (String(ALERT_PHONE).indexOf('X') >= 0) {
    Serial.println("Set ALERT_PHONE before sending SMS.");
    return false;
  }

  sim800.println("AT+CMGF=1");
  if (!waitForSIM("OK", 3000)) return false;
  sim800.print("AT+CMGS=\"");
  sim800.print(ALERT_PHONE);
  sim800.println("\"");
  if (!waitForSIM(">", 5000)) return false;
  sim800.print(text);
  sim800.write(26);
  return waitForSIM("+CMGS:", 20000);
}

String mapUrl() {
  if (!hasLocation) return "Location is not configured.";
  return "https://www.openstreetmap.org/?mlat=" +
         String(savedLatitude, 6) + "&mlon=" +
         String(savedLongitude, 6) + "#map=16/" +
         String(savedLatitude, 6) + "/" + String(savedLongitude, 6);
}

String locationMessage() {
  if (!hasLocation) {
    return "Alert coordinates are not configured in website settings.";
  }
  return "Configured alert coordinates\nLatitude: " + String(savedLatitude, 6) +
         "\nLongitude: " + String(savedLongitude, 6) +
         "\nMap: " + mapUrl();
}

bool normalizePhone(String& value) {
  value.trim();
  value.replace(" ", "");
  value.replace("-", "");
  return value.length() > 0;
}

void handleSimLine(String line) {
  line.trim();
  if (line.startsWith("+CMT:")) {
    const int firstQuote = line.indexOf('"');
    const int secondQuote = line.indexOf('"', firstQuote + 1);
    smsSender = firstQuote >= 0 && secondQuote > firstQuote
                    ? line.substring(firstQuote + 1, secondQuote)
                    : "";
    waitingForSmsBody = true;
    return;
  }

  if (!waitingForSmsBody) return;
  waitingForSmsBody = false;

  String expectedPhone = ALERT_PHONE;
  String receivedPhone = smsSender;
  if (!normalizePhone(expectedPhone) || !normalizePhone(receivedPhone) ||
      receivedPhone != expectedPhone) {
    Serial.println("Ignored SMS command from an unconfigured phone.");
    return;
  }

  line.toUpperCase();
  if (line == "LOCATION") {
    sendSms(locationMessage());
  }
}

void processSimInput() {
  while (sim800.available()) {
    const char value = static_cast<char>(sim800.read());
    if (value == '\r') continue;
    if (value == '\n') {
      if (simLine.length() > 0) handleSimLine(simLine);
      simLine = "";
    } else if (simLine.length() < 240) {
      simLine += value;
    } else {
      simLine = "";
      waitingForSmsBody = false;
    }
  }
}

void beginCall() {
  if (String(ALERT_PHONE).indexOf('X') >= 0) {
    Serial.println("Set ALERT_PHONE before placing a call.");
    return;
  }
  sim800.print("ATD");
  sim800.print(ALERT_PHONE);
  sim800.println(";");
  callInProgress = true;
  callEndAt = millis() + 20000;
}

void endCallIfDue() {
  if (callInProgress && static_cast<int32_t>(millis() - callEndAt) >= 0) {
    sim800.println("ATH");
    callInProgress = false;
  }
}

void publishJson(const char* topic, JsonDocument& document, bool retained = false) {
  char output[768];
  const size_t length = serializeJson(document, output, sizeof(output));
  if (length == 0 || length >= sizeof(output)) {
    Serial.println("Could not serialize MQTT message.");
    return;
  }
  if (!mqtt.publish(topic, output, retained)) {
    Serial.print("MQTT publish failed for ");
    Serial.println(topic);
  }
}

void publishSettings() {
  JsonDocument document;
  document["rollLimit"] = rollLimit;
  if (hasLocation) {
    document["lat"] = savedLatitude;
    document["lon"] = savedLongitude;
  } else {
    document["lat"] = nullptr;
    document["lon"] = nullptr;
  }
  publishJson(TOPIC_SETTINGS, document, true);
}

bool validLocation(double latitude, double longitude) {
  return isfinite(latitude) && isfinite(longitude) &&
         latitude >= -90.0 && latitude <= 90.0 &&
         longitude >= -180.0 && longitude <= 180.0;
}

void saveSettings(JsonDocument& document) {
  const float newRollLimit = document["rollLimit"] | rollLimit;
  if (newRollLimit < 15.0 || newRollLimit > 90.0) {
    Serial.println("Rejected invalid rollover threshold.");
    return;
  }

  JsonVariantConst lat = document["lat"];
  JsonVariantConst lon = document["lon"];
  const bool clearsLocation = lat.isNull() && lon.isNull();
  double newLatitude = savedLatitude;
  double newLongitude = savedLongitude;
  if (!clearsLocation) {
    if (lat.isNull() || lon.isNull()) {
      Serial.println("Rejected incomplete MQTT coordinates.");
      return;
    }
    newLatitude = lat.as<double>();
    newLongitude = lon.as<double>();
    if (!validLocation(newLatitude, newLongitude)) {
      Serial.println("Rejected invalid MQTT coordinates.");
      return;
    }
  }

  rollLimit = newRollLimit;
  preferences.putFloat("roll15", rollLimit);

  if (clearsLocation) {
    hasLocation = false;
    preferences.remove("hasLocation");
    preferences.remove("lat");
    preferences.remove("lon");
  } else {
    savedLatitude = newLatitude;
    savedLongitude = newLongitude;
    hasLocation = true;
    preferences.putBool("hasLocation", true);
    preferences.putFloat("lat", static_cast<float>(savedLatitude));
    preferences.putFloat("lon", static_cast<float>(savedLongitude));
  }

  publishSettings();
  Serial.println("Settings stored in ESP32 flash.");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  message.reserve(length);
  for (unsigned int index = 0; index < length; index++) {
    message += static_cast<char>(payload[index]);
  }

  if (strcmp(topic, TOPIC_SET) == 0) {
    JsonDocument document;
    const DeserializationError error = deserializeJson(document, message);
    if (error) {
      Serial.print("Invalid settings JSON: ");
      Serial.println(error.c_str());
      return;
    }
    saveSettings(document);
  } else if (strcmp(topic, TOPIC_COMMAND) == 0 && message == "LOCATION") {
    sendSms(locationMessage());
  }
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (wifiAttempted && millis() - lastWifiAttemptAt < 10000) return;
  wifiAttempted = true;
  lastWifiAttemptAt = millis();
  if (WiFi.getMode() != WIFI_STA) WiFi.mode(WIFI_STA);
  Serial.printf("Retrying Wi-Fi (status %d)\n", static_cast<int>(WiFi.status()));
  WiFi.reconnect();
}

void connectMqtt() {
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED ||
      (mqttAttempted && millis() - lastMqttAttemptAt < 3000)) return;
  mqttAttempted = true;
  lastMqttAttemptAt = millis();
  if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD,
                   TOPIC_STATUS, 1, true, "offline")) {
    Serial.println("MQTT connected.");
    mqtt.publish(TOPIC_STATUS, "online", true);
    mqtt.subscribe(TOPIC_SET);
    mqtt.subscribe(TOPIC_COMMAND);
    publishSettings();
  } else {
    Serial.print("MQTT connection failed, state: ");
    Serial.println(mqtt.state());
  }
}

bool detectBeat(uint32_t infrared) {
  static uint32_t lastDetectedAt = 0;
  if (infraredBaseline == 0.0) infraredBaseline = infrared;
  infraredBaseline += 0.01f * (infrared - infraredBaseline);
  const float deviation = infrared - infraredBaseline;
  const float threshold = max(100.0f, infraredBaseline * 0.003f);
  const bool risingCrossing =
      previousInfraredDeviation <= threshold && deviation > threshold;
  previousInfraredDeviation = deviation;

  if (!risingCrossing || millis() - lastDetectedAt < 300) return false;
  lastDetectedAt = millis();
  return true;
}

void updatePulseReadings(uint32_t red, uint32_t infrared) {
  if (millis() - lastPulseSampleAt < 20) return;
  lastPulseSampleAt = millis();

  if (infrared >= 50000) {
    if (contactDetectedAt == 0) contactDetectedAt = millis();
    contactDetected = millis() - contactDetectedAt >= 1000;
  } else {
    contactDetectedAt = 0;
    contactDetected = false;
  }

  if (detectBeat(infrared)) {
    const uint32_t now = millis();
    if (lastBeatAt != 0) {
      const float bpm = 60000.0 / static_cast<float>(now - lastBeatAt);
      if (bpm >= 30.0 && bpm <= 220.0) {
        bpmSamples[bpmSampleIndex] = bpm;
        bpmSampleIndex = (bpmSampleIndex + 1) % 4;
        if (bpmSampleCount < 4) bpmSampleCount++;
        float total = 0;
        for (uint8_t index = 0; index < bpmSampleCount; index++) {
          total += bpmSamples[index];
        }
        currentBpm = total / bpmSampleCount;
      }
    }
    lastBeatAt = now;
  }

  redSamples[oximeterWriteIndex] = red;
  irSamples[oximeterWriteIndex] = infrared;
  oximeterWriteIndex = (oximeterWriteIndex + 1) % OXIMETER_WINDOW;
  if (oximeterSampleCount < OXIMETER_WINDOW) oximeterSampleCount++;
  if (oximeterSampleCount < OXIMETER_WINDOW || infrared < 50000) {
    currentSpo2Valid = false;
    return;
  }

  uint32_t redMin = UINT32_MAX, irMin = UINT32_MAX;
  uint32_t redMax = 0, irMax = 0;
  uint64_t redTotal = 0, irTotal = 0;
  for (size_t index = 0; index < OXIMETER_WINDOW; index++) {
    redMin = min(redMin, redSamples[index]);
    redMax = max(redMax, redSamples[index]);
    irMin = min(irMin, irSamples[index]);
    irMax = max(irMax, irSamples[index]);
    redTotal += redSamples[index];
    irTotal += irSamples[index];
  }

  const float redMean = redTotal / static_cast<float>(OXIMETER_WINDOW);
  const float irMean = irTotal / static_cast<float>(OXIMETER_WINDOW);
  if (redMean <= 0 || irMean <= 0 || irMax <= irMin) {
    currentSpo2Valid = false;
    return;
  }
  const float ratio = ((redMax - redMin) / redMean) /
                      ((irMax - irMin) / irMean);
  const float estimate = 110.0 - 25.0 * ratio;
  if (isfinite(estimate) && estimate >= 70.0 && estimate <= 100.0) {
    currentSpo2 = estimate;
    currentSpo2Valid = true;
  } else {
    currentSpo2Valid = false;
  }
}

void publishAlert(const char* type, const char* details) {
  JsonDocument document;
  document["type"] = type;
  document["details"] = details;
  document["active"] = true;
  document["timestamp"] = millis();
  if (hasLocation) {
    document["lat"] = savedLatitude;
    document["lon"] = savedLongitude;
  }
  publishJson(TOPIC_ALERT, document);
  String sms = String("HELMET ALERT: ") + type + "\n" + details +
               "\n" + locationMessage();
  sendSms(sms);
}

void updateAlert(bool condition, bool& active, const char* type,
                 const char* details, bool callOnTrigger = false) {
  if (condition && !active) {
    active = true;
    updateBuzzer();
    publishAlert(type, details);
    if (callOnTrigger) beginCall();
  } else if (!condition && active) {
    active = false;
  }
}

void updateBuzzer() {
  digitalWrite(BUZZER_PIN, accidentActive ? BUZZER_ON : BUZZER_OFF);
}

void showOledStartup(const char* message) {
  if (!oledReady) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("HELMET MONITOR");
  oled.println();
  oled.println(message);
  oled.display();
  Serial.println(message);
}

void updateOled(float roll, float pitch, float temperature, bool mpuOk) {
  if (!oledReady || millis() - lastOledUpdateAt < 500) return;
  lastOledUpdateAt = millis();

  char line[32];
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("HELMET MONITOR");
  oled.print("Contact: ");
  oled.println(contactDetected ? "DETECTED" : "NO");

  oled.print("SpO2: ");
  if (currentSpo2Valid) {
    oled.print(currentSpo2, 0);
    oled.print("%");
  } else {
    oled.print("--");
  }
  oled.print("  HR: ");
  if (isfinite(currentBpm)) oled.println(currentBpm, 0);
  else oled.println("--");

  if (mpuOk) {
    snprintf(line, sizeof(line), "Roll:%4.0f Pitch:%4.0f", roll, pitch);
    oled.println(line);
    snprintf(line, sizeof(line), "MPU temp: %.1f C", temperature);
    oled.println(line);
  } else {
    oled.println("MPU6050 unavailable");
    oled.println("MPU temp: --");
  }

  oled.print("Status: ");
  oled.println(accidentActive ? "ROLLOVER ALERT" : "MONITORING");
  oled.display();
}

void publishTelemetry(float roll, float pitch, float temperature,
                      bool mpuOk) {
  JsonDocument document;
  document["mpuOK"] = mpuOk;
  document["contactDetected"] = contactDetected;
  if (mpuOk) {
    document["roll"] = roll;
    document["pitch"] = pitch;
    document["temperature"] = temperature;
  } else {
    document["roll"] = nullptr;
    document["pitch"] = nullptr;
    document["temperature"] = nullptr;
  }
  if (isfinite(currentBpm)) document["bpm"] = currentBpm;
  else document["bpm"] = nullptr;
  if (currentSpo2Valid) document["spo2"] = currentSpo2;
  else document["spo2"] = nullptr;
  document["accident"] = accidentActive;
  if (hasLocation) {
    document["lat"] = savedLatitude;
    document["lon"] = savedLongitude;
  }
  publishJson(TOPIC_TELEMETRY, document);
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_OFF);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiAttempted = true;
  lastWifiAttemptAt = millis();
  Serial.printf("Starting Wi-Fi connection to %s\n", WIFI_SSID);

  preferences.begin("helmet", false);
  rollLimit = preferences.getFloat("roll15", rollLimit);
  hasLocation = preferences.getBool("hasLocation", false);
  if (hasLocation) {
    savedLatitude = preferences.getFloat("lat", 0.0);
    savedLongitude = preferences.getFloat("lon", 0.0);
    if (!validLocation(savedLatitude, savedLongitude)) hasLocation = false;
  }

  Wire.begin();
  Wire.setTimeOut(50);
  oledReady = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledReady) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("Helmet monitor");
    oled.println("Starting...");
    oled.display();
    Serial.println("SSD1306 OLED ready at I2C 0x3C.");
  } else {
    Serial.println("SSD1306 OLED not detected at I2C 0x3C.");
  }

  showOledStartup("Checking MPU6050...");
  mpuReady = (mpu.begin() == 0);
  if (mpuReady) {
    showOledStartup("Keep still: 3 sec");
    delay(3000);
    mpu.calcOffsets(true, true);
    showOledStartup("MPU6050 calibrated");
  } else {
    showOledStartup("MPU6050 not found");
  }

  showOledStartup("Checking MAX30102...");
  pulseSensorReady = pulseSensor.begin(Wire, I2C_SPEED_STANDARD);
  if (pulseSensorReady) {
    pulseSensor.setup(60, 4, 2, 100, 411, 4096);
    pulseSensor.setPulseAmplitudeRed(0x1F);
    pulseSensor.setPulseAmplitudeIR(0x1F);
    pulseSensor.setPulseAmplitudeGreen(0);
    showOledStartup("MAX30102 ready");
  } else {
    showOledStartup("MAX30102 not found");
  }

  if (WiFi.status() == WL_CONNECTED && !wifiWasConnected) {
    wifiWasConnected = true;
    Serial.print("Wi-Fi connected, IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Wi-Fi RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else if (WiFi.status() != WL_CONNECTED) {
    wifiWasConnected = false;
  }

  showOledStartup("Starting SIM800L...");
  sim800.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delay(5000);
  sim800.println("AT");
  delay(250);
  sim800.println("ATE0");
  delay(250);
  sim800.println("AT+CMGF=1");
  delay(250);
  sim800.println("AT+CNMI=2,2,0,0,0");
  delay(250);
  showOledStartup("SIM800L commands sent");

  tlsClient.setInsecure();  // Testing only; use CA validation for deployment.
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setSocketTimeout(2);
  mqtt.setKeepAlive(20);
  mqtt.setBufferSize(1024);
}

void loop() {
  if (pulseSensorReady) {
    updatePulseReadings(pulseSensor.getRed(), pulseSensor.getIR());
  }

  float roll = NAN;
  float pitch = NAN;
  float temperature = NAN;
  bool mpuOk = false;
  if (mpuReady) {
    mpu.update();
    roll = mpu.getAngleX();
    pitch = mpu.getAngleY();
    temperature = mpu.getTemp();
    mpuOk = isfinite(roll) && isfinite(pitch) && isfinite(temperature);
  }

  const bool rolloverAngle = mpuOk && contactDetected &&
      (fabsf(roll) >= rollLimit || fabsf(pitch) >= rollLimit);
  if (rolloverAngle) {
    if (rolloverStartAt == 0) rolloverStartAt = millis();
  } else {
    rolloverStartAt = 0;
  }
  const bool rolloverDetected = rolloverStartAt != 0 &&
                                millis() - rolloverStartAt >= 1500;

  updateAlert(rolloverDetected, accidentActive, "ACCIDENT",
              "MAX30102 finger contact detected and MPU6050 tilt exceeded the configured angle for 1.5 seconds.", true);
  updateBuzzer();
  updateOled(roll, pitch, temperature, mpuOk);

  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMqtt();
  mqtt.loop();
  processSimInput();
  endCallIfDue();

  if (mqtt.connected() && millis() - lastTelemetryAt >= 1000) {
    lastTelemetryAt = millis();
    publishTelemetry(roll, pitch, temperature, mpuOk);
  }
  if (mqtt.connected() && millis() - lastStatusAt >= 5000) {
    lastStatusAt = millis();
    mqtt.publish(TOPIC_STATUS, "online", true);
  }
  delay(5);
}
