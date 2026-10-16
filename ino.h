#include <Wire.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <MPU6050.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <uri/UriBraces.h>
#include <LittleFS.h>
#include <vector>
#include <map>
#include <cmath>

// ===================== Pin Mapping (ESP32 DevKit V1) =====================
#define I2C_SDA 21
#define I2C_SCL 22

#define PIN_LED_GREEN 15
#define PIN_LED_YELLOW 5
#define PIN_LED_RED 18
#define PIN_BUZZER 19

// GPS on UART2
#define RX2_PIN 16
#define TX2_PIN 17

// Timezone offset for Philippines (UTC +8)
#define TIMEZONE_OFFSET 8

// APA Detection Radius (in meters)
#define APA_DETECTION_RADIUS 15000
#define APA_AUTO_ADD_RADIUS 5000

// ===================== Event Storage =====================
#define EVENTS_FILE "/events.csv"
#define RECENT_CACHE_SIZE 20
#define MAP_FETCH_LIMIT 5000

// GPS Baud Rate Auto-Scan
int gpsBaudRates[] = { 9600, 4800, 115200, 38400, 19200, 57600 };
int numBaudRates = 6;
int currentBaudIndex = 0;
bool gpsDataDetected = false;
unsigned long lastBaudSwitch = 0;
const uint32_t BAUD_SWITCH_INTERVAL = 3000;

// GPS Debug
bool gpsDebugMode = false;
unsigned long lastGPSDebug = 0;
bool gpsFixAcquired = false;

HardwareSerial GPSSerial(2);

// ===================== Device Configuration =====================
const char* AP_SSID = "BRAKESAFE_AP";
const char* AP_PASSWORD = "";

Preferences preferences;

// ===================== Globals =====================
MPU6050 mpu;
TinyGPSPlus gps;
DNSServer dnsServer;
WebServer server(80);

static const float ACCEL_LSB_PER_G = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;   // ±250°/s range
float ax_offset = 0.0f;
float ax_g_filt = 0.0f;
const float AX_ALPHA = 0.2f;

// Compass heading (0=N, 90=E, 180=S, 270=W)
float currentHeadingDeg = 0.0f;
unsigned long lastGyroMs = 0;

bool mpuInitialized = false;

enum BrakeLevel { BRAKE_NORMAL,
                  BRAKE_SOFT,
                  BRAKE_MODERATE,
                  BRAKE_HARD,
                  BRAKE_EMERGENCY };

struct BrakeEvent {
  double lat;
  double lng;
  float speed;
  float decel;
  BrakeLevel level;
  String timestamp;
  bool sent;
  String brakeLevelStr;
  String barangay;
  String municipality;
  String province;
  String direction;
};

BrakeEvent recentEvents[RECENT_CACHE_SIZE];
int recentCount = 0;
int historyCount = 0;

bool emergencyTriggered = false;
unsigned long emergencyStartTime = 0;
const uint32_t EMERGENCY_BUZZER_DURATION = 5000;

unsigned long lastAccelSampleMs = 0;
const uint32_t ACCEL_SAMPLE_PERIOD_MS = 10;
unsigned long lastEventMs = 0;

const uint32_t EVENT_COOLDOWN_SOFT_MS = 3000;
const uint32_t EVENT_COOLDOWN_HARD_MS = 1500;

String lastEventTimeStr = "None";
BrakeLevel lastBrakeLevel = BRAKE_NORMAL;
float lastDecel = 0.0;

// ===================== WiFi State =====================
String wifiSSID = "";
String wifiPassword = "";
String apPasswordSaved = "";
bool wifiConnected = false;
bool apModeActive = false;
bool apHasPassword = false;

unsigned long wifiReconnectAttempt = 0;
const uint32_t WIFI_RECONNECT_INTERVAL = 30000;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
bool wifiEverConnected = false;

// APA
struct APA {
  double lat;
  double lng;
  String reason;
  String timestamp;
  String barangay;
  String municipality;
  String province;
  String direction;
  bool autoAdded;
};
std::vector<APA> accidentProneAreas;

const char* COMPASS_DIRECTIONS[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };

bool apaAlertActive = false;
unsigned long apaAlertStartTime = 0;
const uint32_t APA_ALERT_DURATION = 3000;

// ===================== Forward declarations =====================
String formatTimestamp();

// ===================== Helper Functions =====================
void setLeds(bool g, bool y, bool r) {
  digitalWrite(PIN_LED_GREEN, g ? HIGH : LOW);
  digitalWrite(PIN_LED_YELLOW, y ? HIGH : LOW);
  digitalWrite(PIN_LED_RED, r ? HIGH : LOW);
}

void buzzerOn(bool on) {
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

BrakeLevel classifyBrake(float decel_g) {
  if (decel_g < 0.20f) return BRAKE_NORMAL;
  if (decel_g < 0.35f) return BRAKE_SOFT;
  if (decel_g < 0.50f) return BRAKE_MODERATE;
  if (decel_g < 0.70f) return BRAKE_HARD;
  return BRAKE_EMERGENCY;
}

String getBrakeLevelString(BrakeLevel lvl) {
  switch (lvl) {
    case BRAKE_NORMAL: return "Normal";
    case BRAKE_SOFT: return "Soft";
    case BRAKE_MODERATE: return "Moderate";
    case BRAKE_HARD: return "Hard";
    case BRAKE_EMERGENCY: return "Emergency";
    default: return "Unknown";
  }
}

String headingToCompass(float deg) {
  if (deg < 0) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  int idx = (int)((deg + 22.5f) / 45.0f) % 8;
  return String(COMPASS_DIRECTIONS[idx]);
}

String getCurrentDirection() {
  if (gps.course.isValid() && gps.speed.isValid() && gps.speed.kmph() > 3.0f) {
    return headingToCompass(gps.course.deg());
  }
  return headingToCompass(currentHeadingDeg);
}

String getBarangay(double lat, double lng) {
  if (lat > 11.20 && lat < 11.30 && lng > 123.70 && lng < 123.80) {
    return "Bantayan";
  }
  return "Unknown";
}

String getMunicipality(double lat, double lng) {
  if (lat > 11.20 && lat < 11.30 && lng > 123.70 && lng < 123.80) {
    return "Bantayan";
  }
  return "Unknown";
}

String getProvince(double lat, double lng) {
  if (lat > 11.20 && lat < 11.30 && lng > 123.70 && lng < 123.80) {
    return "Cebu";
  }
  return "Unknown";
}

double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

bool isInAPAZone(double lat, double lng) {
  for (size_t i = 0; i < accidentProneAreas.size(); i++) {
    double dist = haversineDistance(lat, lng, accidentProneAreas[i].lat, accidentProneAreas[i].lng);
    if (dist < APA_DETECTION_RADIUS) {
      return true;
    }
  }
  return false;
}

void checkEmergencyBrakeAPA(double lat, double lng) {
  for (size_t i = 0; i < accidentProneAreas.size(); i++) {
    double dist = haversineDistance(lat, lng, accidentProneAreas[i].lat, accidentProneAreas[i].lng);
    if (dist < APA_DETECTION_RADIUS) {
      if (!apaAlertActive) {
        apaAlertActive = true;
        apaAlertStartTime = millis();
        buzzerOn(true);
        Serial.println("⚠️ APA ALERT! Within " + String(APA_DETECTION_RADIUS / 1000) + "km of accident prone area!");
        Serial.println("📍 Distance: " + String(dist / 1000, 2) + " km");
        Serial.println("📍 APA: " + accidentProneAreas[i].reason);
      }
      break;
    }
  }
}

void persistAPA() {
  preferences.begin("apa", false);
  String apaData = "";
  for (size_t i = 0; i < accidentProneAreas.size(); i++) {
    apaData += String(accidentProneAreas[i].lat, 6) + "," + String(accidentProneAreas[i].lng, 6) + ",";
    apaData += accidentProneAreas[i].reason + "," + accidentProneAreas[i].timestamp + ",";
    apaData += accidentProneAreas[i].barangay + "," + accidentProneAreas[i].municipality + ",";
    apaData += accidentProneAreas[i].province + "," + accidentProneAreas[i].direction + ",";
    apaData += String(accidentProneAreas[i].autoAdded ? "1" : "0") + ";";
  }
  preferences.putString("apa_data", apaData);
  preferences.end();
}

void autoAddAPA(double lat, double lng, String reason) {
  for (size_t i = 0; i < accidentProneAreas.size(); i++) {
    double dist = haversineDistance(lat, lng, accidentProneAreas[i].lat, accidentProneAreas[i].lng);
    if (dist < APA_AUTO_ADD_RADIUS) {
      Serial.println("⚠️ APA already exists within 5km radius");
      return;
    }
  }

  APA newAPA;
  newAPA.lat = lat;
  newAPA.lng = lng;
  newAPA.reason = reason;
  newAPA.timestamp = formatTimestamp();
  newAPA.barangay = getBarangay(lat, lng);
  newAPA.municipality = getMunicipality(lat, lng);
  newAPA.province = getProvince(lat, lng);
  newAPA.direction = getCurrentDirection();
  newAPA.autoAdded = true;
  accidentProneAreas.push_back(newAPA);
  persistAPA();

  Serial.println("✅ Auto-Added APA at: " + String(lat, 6) + ", " + String(lng, 6));
  Serial.println("📍 Reason: " + newAPA.reason);
}

void applyIndicators(BrakeLevel lvl) {
  switch (lvl) {
    case BRAKE_NORMAL:
      setLeds(true, false, false);
      break;
    case BRAKE_SOFT:
    case BRAKE_MODERATE:
      setLeds(false, true, false);
      break;
    case BRAKE_HARD:
    case BRAKE_EMERGENCY:
      setLeds(false, false, true);
      break;
  }

  if (lvl == BRAKE_EMERGENCY) {
    if (!emergencyTriggered) {
      emergencyTriggered = true;
      emergencyStartTime = millis();
      buzzerOn(true);
    } else if (millis() - emergencyStartTime > EMERGENCY_BUZZER_DURATION) {
      buzzerOn(false);
      emergencyTriggered = false;
    }
  } else {
    if (emergencyTriggered && millis() - emergencyStartTime > EMERGENCY_BUZZER_DURATION) {
      buzzerOn(false);
      emergencyTriggered = false;
    }
  }
}

String formatTimestamp() {
  if (gps.date.isValid() && gps.time.isValid()) {
    int utcHour = gps.time.hour();
    int utcMinute = gps.time.minute();
    int utcSecond = gps.time.second();
    int day = gps.date.day();
    int month = gps.date.month();
    int year = gps.date.year();

    int localHour = utcHour + TIMEZONE_OFFSET;
    if (localHour >= 24) {
      localHour -= 24;
      day++;
      if (day > 30) {
        day = 1;
        month++;
      }
      if (month > 12) {
        month = 1;
        year++;
      }
    }

    char buf[25];
    snprintf(buf, sizeof(buf), "%02d/%02d/%04d %02d:%02d:%02d",
             day, month, year, localHour, utcMinute, utcSecond);
    return String(buf);
  }
  unsigned long seconds = millis() / 1000UL;
  unsigned long hours = (seconds / 3600) % 24;
  unsigned long minutes = (seconds % 3600) / 60;
  unsigned long secs = seconds % 60;
  char buf[32];
  snprintf(buf, sizeof(buf), "01/01/1970 %02lu:%02lu:%02lu", hours, minutes, secs);
  return String(buf);
}

String formatTimestampUTC() {
  if (gps.date.isValid() && gps.time.isValid()) {
    char buf[25];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    return String(buf);
  }
  unsigned long seconds = millis() / 1000UL;
  unsigned long hours = (seconds / 3600) % 24;
  unsigned long minutes = (seconds % 3600) / 60;
  unsigned long secs = seconds % 60;
  char buf[32];
  snprintf(buf, sizeof(buf), "1970-01-01 %02lu:%02lu:%02lu", hours, minutes, secs);
  return String(buf);
}

String getCurrentTimestamp() {
  unsigned long now = millis() / 1000UL;
  char buf[32];
  snprintf(buf, sizeof(buf), "%lu", now);
  return String(buf);
}

// ===================== Event Storage (append-only LittleFS) =====================
void appendEventToFile(const BrakeEvent &event) {
  File f = LittleFS.open(EVENTS_FILE, "a");
  if (!f) {
    Serial.println("[Events] ⚠️ Failed to open /events.csv for append!");
    return;
  }
  f.printf("%.6f,%.6f,%.2f,%.2f,%d,%s,%s\n",
           event.lat,
           event.lng,
           event.speed,
           event.decel,
           (int)event.level,
           event.timestamp.c_str(),
           event.direction.c_str());
  f.close();
  historyCount++;
}

void pushRecentEvent(const BrakeEvent &event) {
  if (recentCount < RECENT_CACHE_SIZE) {
    recentEvents[recentCount++] = event;
  } else {
    for (int i = 0; i < RECENT_CACHE_SIZE - 1; i++) {
      recentEvents[i] = recentEvents[i + 1];
    }
    recentEvents[RECENT_CACHE_SIZE - 1] = event;
  }
}

void recordEvent(BrakeEvent event) {
  appendEventToFile(event);
  pushRecentEvent(event);
}

// Purge legacy (0,0) rows from the file on boot
void purgeZeroZeroEvents() {
  if (!LittleFS.exists(EVENTS_FILE)) return;

  File in = LittleFS.open(EVENTS_FILE, "r");
  if (!in) return;

  File out = LittleFS.open("/events_tmp.csv", "w");
  if (!out) { in.close(); return; }

  int kept = 0, dropped = 0;
  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) continue;

    float lat = line.substring(0, c1).toFloat();
    float lng = line.substring(c1 + 1, c2).toFloat();

    if (fabs(lat) < 0.000001 && fabs(lng) < 0.000001) {
      dropped++;
      continue;
    }
    out.println(line);
    kept++;
  }
  in.close();
  out.close();

  LittleFS.remove(EVENTS_FILE);
  LittleFS.rename("/events_tmp.csv", EVENTS_FILE);

  if (dropped > 0) {
    Serial.printf("[Events] Purged %d 0,0 rows (kept %d)\n", dropped, kept);
  }
}

void loadEventsFromFile() {
  historyCount = 0;
  recentCount = 0;

  if (!LittleFS.exists(EVENTS_FILE)) {
    Serial.println("[Events] No events file yet — starting fresh");
    return;
  }

  File f = LittleFS.open(EVENTS_FILE, "r");
  if (!f) {
    Serial.println("[Events] Failed to open events file");
    return;
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);
    int c5 = line.indexOf(',', c4 + 1);
    int c6 = line.indexOf(',', c5 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0 || c5 < 0 || c6 < 0) continue;

    BrakeEvent ev;
    ev.lat = line.substring(0, c1).toFloat();
    ev.lng = line.substring(c1 + 1, c2).toFloat();

    // ── Skip 0,0 rows that snuck in before the guard ──
    if (fabs(ev.lat) < 0.000001 && fabs(ev.lng) < 0.000001) {
      continue;
    }

    ev.speed = line.substring(c2 + 1, c3).toFloat();
    ev.decel = line.substring(c3 + 1, c4).toFloat();
    ev.level = (BrakeLevel)line.substring(c4 + 1, c5).toInt();
    ev.timestamp = line.substring(c5 + 1, c6);
    ev.direction = line.substring(c6 + 1);
    ev.brakeLevelStr = getBrakeLevelString(ev.level);
    ev.barangay = getBarangay(ev.lat, ev.lng);
    ev.municipality = getMunicipality(ev.lat, ev.lng);
    ev.province = getProvince(ev.lat, ev.lng);
    ev.sent = true;

    historyCount++;
    pushRecentEvent(ev);
  }
  f.close();

  Serial.printf("[Events] Loaded %d events from LittleFS (last %d cached in RAM)\n",
                historyCount, recentCount);

  if (recentCount > 0) {
    lastEventTimeStr = recentEvents[recentCount - 1].timestamp;
    lastBrakeLevel   = recentEvents[recentCount - 1].level;
    lastDecel        = recentEvents[recentCount - 1].decel;
  }
}

// ===================== GPS FUNCTIONS =====================
void initGPS() {
  currentBaudIndex = 0;
  gpsDataDetected = false;
  gpsFixAcquired = false;
  lastBaudSwitch = millis();

  GPSSerial.begin(gpsBaudRates[currentBaudIndex], SERIAL_8N1, RX2_PIN, TX2_PIN);

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║     GPS AUTO-DETECTION STARTED         ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println();
  Serial.println("📡 PIN CONFIGURATION:");
  Serial.println("   GPS TX → GPIO16 (RX2)");
  Serial.println("   GPS RX → GPIO17 (TX2)");
  Serial.println();
  Serial.println("🔍 SCANNING BAUD RATES: 9600, 4800, 115200, 38400, 19200, 57600");
  Serial.println("============================================");
  Serial.println();
}

void switchToNextBaudRate() {
  currentBaudIndex++;
  if (currentBaudIndex >= numBaudRates) {
    currentBaudIndex = 0;
  }

  GPSSerial.end();
  delay(50);
  GPSSerial.begin(gpsBaudRates[currentBaudIndex], SERIAL_8N1, RX2_PIN, TX2_PIN);

  Serial.print("🔄 Switching to ");
  Serial.print(gpsBaudRates[currentBaudIndex]);
  Serial.println(" baud...");

  lastBaudSwitch = millis();
  gpsDataDetected = false;
}

void pumpGPS() {
  while (GPSSerial.available()) {
    char c = GPSSerial.read();

    if (gpsDebugMode) {
      Serial.print(c);
    }

    if (c == '$' && !gpsDataDetected) {
      gpsDataDetected = true;
      Serial.println("\n\n✅✅✅ GPS DATA DETECTED! ✅✅✅");
      Serial.print("🔒 Locked at ");
      Serial.print(gpsBaudRates[currentBaudIndex]);
      Serial.println(" baud");
      Serial.println();
    }

    gps.encode(c);
  }

  if (!gpsDataDetected && millis() - lastBaudSwitch > BAUD_SWITCH_INTERVAL) {
    switchToNextBaudRate();
  }

  if (gps.location.isValid() && !gpsFixAcquired) {
    gpsFixAcquired = true;
    Serial.println("\n🎉🎉🎉 GPS FIX ACQUIRED! 🎉🎉🎉");
    Serial.print("📍 Latitude:  ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("📍 Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("🛰️  Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.println("============================================");
    Serial.println();
  }
}

// ===================== WiFi Configuration (PERSISTENT) =====================
void loadWiFiCredentials() {
  preferences.begin("wifi", true);
  wifiSSID = preferences.getString("ssid", "");
  wifiPassword = preferences.getString("pass", "");
  apPasswordSaved = preferences.getString("ap_pass", "");
  apHasPassword = (apPasswordSaved.length() > 0);
  preferences.end();

  if (wifiSSID.length() > 0) {
    Serial.println("[WiFi] Loaded saved credentials: " + wifiSSID);
  } else {
    Serial.println("[WiFi] No saved credentials found");
  }
  Serial.println("[WiFi] AP Password: " + String(apHasPassword ? "Set" : "Not set"));
}

bool saveWiFiCredentials(String ssid, String password) {
  preferences.begin("wifi", false);
  size_t writtenSSID = preferences.putString("ssid", ssid);
  size_t writtenPass = preferences.putString("pass", password);
  preferences.end();

  bool ok = (writtenSSID > 0) || (ssid.length() == 0);
  if (!ok) {
    Serial.println("[WiFi] ⚠️ Failed to save SSID to NVS");
    return false;
  }

  wifiSSID = ssid;
  wifiPassword = password;
  Serial.println("[WiFi] Credentials saved to flash: " + ssid);
  return true;
}

void saveAPPassword(String password) {
  preferences.begin("wifi", false);
  preferences.putString("ap_pass", password);
  preferences.end();
  apPasswordSaved = password;
  apHasPassword = (password.length() > 0);
  Serial.println("[WiFi] AP Password " + String(apHasPassword ? "saved" : "cleared"));
}

bool connectToWiFi() {
  if (wifiSSID.length() == 0) {
    Serial.println("[WiFi] No credentials available");
    return false;
  }

  Serial.print("[WiFi] Connecting to " + wifiSSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    apModeActive = false;
    wifiEverConnected = true;
    dnsServer.stop();
    Serial.print("[WiFi] ✅ Connected! IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    wifiConnected = false;
    Serial.println("[WiFi] ❌ Connection failed (credentials are kept for retry)");
    return false;
  }
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  if (apHasPassword && apPasswordSaved.length() > 0) {
    WiFi.softAP(AP_SSID, apPasswordSaved.c_str());
    Serial.println("[WiFi] AP with password");
  } else {
    WiFi.softAP(AP_SSID);
    Serial.println("[WiFi] AP open (no password)");
  }
  apModeActive = true;
  wifiConnected = false;
  Serial.print("[WiFi] Access Point started. SSID: ");
  Serial.print(AP_SSID);
  if (apHasPassword) {
    Serial.print(" (Password protected)");
  } else {
    Serial.print(" (Open)");
  }
  Serial.print(" IP: ");
  Serial.println(WiFi.softAPIP());

  dnsServer.start(53, "*", WiFi.softAPIP());
}

void manageWiFi() {
  if (wifiSSID.length() == 0) {
    if (!apModeActive) {
      startAccessPoint();
    }
    if (apModeActive) dnsServer.processNextRequest();
    return;
  }

  bool isConnected = (WiFi.status() == WL_CONNECTED);
  wifiConnected = isConnected;

  if (isConnected) {
    if (apModeActive) {
      Serial.println("[WiFi] Station connected while AP was active — switching to STA only");
      dnsServer.stop();
      WiFi.softAPdisconnect(true);
      apModeActive = false;
    }
  } else {
    unsigned long now = millis();
    if (now - wifiReconnectAttempt > WIFI_RECONNECT_INTERVAL) {
      wifiReconnectAttempt = now;
      Serial.println("[WiFi] Attempting reconnect to saved network...");
      connectToWiFi();

      if (!wifiConnected && !apModeActive) {
        startAccessPoint();
      }
    }
    if (apModeActive) {
      dnsServer.processNextRequest();
    }
  }
}

// ===================== MPU =====================
void initMPU() {
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  mpu.initialize();
  const int N = 200;
  long axSum = 0;
  for (int i = 0; i < N; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axSum += ax;
    delay(5);
  }
  ax_offset = ((float)axSum / N) / ACCEL_LSB_PER_G;
  ax_g_filt = 0.0f;
  currentHeadingDeg = 0.0f;
  lastGyroMs = millis();
  Serial.println("[MPU] Initialized and calibrated.");
}

// ===================== Web Server Handlers =====================
String getStatusJSON() {
  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lng = gps.location.isValid() ? gps.location.lng() : 0.0;
  double spd = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  String ts = formatTimestamp();
  String json = "{";
  json += "\"latitude\":" + String(lat, 6) + ",\"longitude\":" + String(lng, 6) + ",\"speed\":" + String(spd, 2) + ",\"timestamp\":\"" + ts + "\",";
  json += "\"brake_level\":\"" + getBrakeLevelString(lastBrakeLevel) + "\",\"deceleration_g\":" + String(lastDecel, 3) + ",";
  json += "\"queue_size\":" + String(historyCount) + ",\"last_event\":\"" + lastEventTimeStr + "\",";
  json += "\"direction\":\"" + getCurrentDirection() + "\",";
  json += "\"heading\":" + String(currentHeadingDeg, 1) + ",";
  json += "\"wifi_connected\":" + String(wifiConnected ? "true" : "false") + ",\"ap_mode\":" + String(apModeActive ? "true" : "false") + ",";
  json += "\"wifi_ssid\":\"" + wifiSSID + "\",";
  json += "\"gps_module\":" + String(gpsDataDetected ? "true" : "false") + ",";
  json += "\"mpu_ready\":" + String(mpuInitialized ? "true" : "false") + ",";
  json += "\"emergency_buzzer\":" + String(emergencyTriggered ? "true" : "false") + "}";
  return json;
}

String getWiFiStatusHTML() {
  String html = "<div class='card'>";
  html += "<h3><i class='fas fa-wifi'></i> WiFi Configuration</h3>";
  if (wifiSSID.length() > 0) {
    html += "<p style='margin-bottom:12px;color:#4CAF50;'><i class='fas fa-save'></i> Saved network: <b>" + wifiSSID + "</b></p>";
  } else {
    html += "<p style='margin-bottom:12px;color:#FF9800;'><i class='fas fa-exclamation-triangle'></i> No saved network yet.</p>";
  }
  html += "<form action='/setwifi' method='POST'>";
  html += "<div style='margin-bottom:15px;'>";
  html += "<label><i class='fas fa-network-wired'></i> SSID:</label><br>";
  html += "<input type='text' name='ssid' value='" + wifiSSID + "' placeholder='Enter WiFi SSID' required>";
  html += "</div>";
  html += "<div style='margin-bottom:15px;'>";
  html += "<label><i class='fas fa-lock'></i> Password:</label><br>";
  html += "<input type='password' name='password' value='" + wifiPassword + "' placeholder='Enter WiFi Password'>";
  html += "</div>";
  html += "<button type='submit' class='btn-primary'><i class='fas fa-save'></i> Save & Connect</button>";
  html += "</form>";
  html += "</div>";

  html += "<div class='card'>";
  html += "<h3><i class='fas fa-broadcast-tower'></i> Access Point Settings</h3>";
  html += "<form action='/setappass' method='POST'>";
  html += "<div style='margin-bottom:15px;'>";
  html += "<label><i class='fas fa-key'></i> AP Password (leave empty for open network):</label><br>";
  html += "<input type='text' name='ap_password' placeholder='Enter AP password or leave empty' value='" + apPasswordSaved + "'>";
  html += "</div>";
  html += "<button type='submit' class='btn-success'><i class='fas fa-shield-alt'></i> Set AP Password</button>";
  html += "</form>";
  html += "<div style='margin-top:15px;'>";
  html += "<p><strong>Status:</strong> ";
  if (apHasPassword) {
    html += "<span class='status-ok'><i class='fas fa-lock'></i> Password Protected</span>";
  } else {
    html += "<span style='color:#f1c40f;'><i class='fas fa-unlock'></i> Open (No Password)</span>";
  }
  html += "</p>";
  html += "</div>";
  html += "</div>";

  html += "<div class='card'>";
  html += "<h3><i class='fas fa-info-circle'></i> Connection Status</h3>";
  html += "<table>";
  html += "<tr><td><i class='fas fa-signal'></i> WiFi Status</td><td>";
  if (wifiConnected) {
    html += "<span class='status-ok'><i class='fas fa-check-circle'></i> Connected to " + wifiSSID + "</span>";
  } else if (apModeActive) {
    html += "<span style='color:#f1c40f;'><i class='fas fa-wifi'></i> Access Point Mode (retrying saved network)</span>";
  } else {
    html += "<span class='status-fail'><i class='fas fa-times-circle'></i> Disconnected</span>";
  }
  html += "</td></tr>";
  html += "<tr><td><i class='fas fa-ip'></i> IP Address</td><td>";
  if (wifiConnected) {
    html += WiFi.localIP().toString();
  } else if (apModeActive) {
    html += WiFi.softAPIP().toString();
  } else {
    html += "--";
  }
  html += "</td></tr>";
  html += "</table>";
  html += "</div>";
  return html;
}

// ===================== ROOT PAGE =====================
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>BRAKESAFE Dashboard</title>
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 512 512'><path fill='%23e94560' d='M135.2 117.4L109.1 192H402.9l-26.1-74.6C372.3 104.6 360.2 96 346.6 96H165.4c-13.6 0-25.7 8.6-30.2 21.4zM39.6 196.8L74.8 96.3C88.3 57.8 124.6 32 165.4 32H346.6c40.8 0 77.1 25.8 90.6 64.3l35.2 100.5c23.2 9.6 39.6 32.5 39.6 59.2V400v48c0 17.7-14.3 32-32 32H448c-17.7 0-32-14.3-32-32V400H96v48c0 17.7-14.3 32-32 32H32c-17.7 0-32-14.3-32-32V400 256c0-26.7 16.4-49.6 39.6-59.2zM128 288a32 32 0 1 0 -64 0 32 32 0 1 0 64 0zm288 32a32 32 0 1 0 0-64 32 32 0 1 0 0 64z'/></svg>">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
<style>
*{margin:0;padding:0;box-sizing:border-box;}
:root{--primary:#e94560;--secondary:#16213e;--bg:#1a1a2e;--card-bg:#1e2a4a;--text:#eee;--shadow:0 4px 6px rgba(0,0,0,0.3);}
body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:var(--bg);color:var(--text);display:flex;flex-wrap:wrap;min-height:100vh;}
.sidebar{width:260px;background:var(--secondary);min-height:100vh;padding:20px;position:fixed;left:0;top:0;overflow-y:auto;border-right:2px solid #2a2a4e;z-index:1000;transition:transform 0.3s ease;}
.sidebar-brand{text-align:center;padding-bottom:20px;border-bottom:2px solid #2a2a4e;margin-bottom:20px;}
.sidebar-brand i{font-size:2em;color:var(--primary);}
.sidebar-brand h2{color:#fff;font-size:1.3em;margin-top:5px;}
.nav-item{display:flex;align-items:center;padding:12px 15px;margin:5px 0;border-radius:8px;cursor:pointer;transition:all 0.3s;color:#aaa;text-decoration:none;}
.nav-item i{width:25px;font-size:1.1em;margin-right:10px;}
.nav-item:hover{background:#2a2a4e;color:#fff;}
.nav-item.active{background:var(--primary);color:#fff;}
.sidebar-footer{font-size:0.8em;color:#666;text-align:center;margin-top:30px;padding-top:20px;border-top:2px solid #2a2a4e;}
.main-content{margin-left:260px;padding:20px;flex:1;min-width:300px;width:calc(100% - 260px);}
.container{max-width:1200px;margin:0 auto;}
.page-header{margin-bottom:25px;}
.page-header h1{color:#fff;font-size:1.8em;display:flex;align-items:center;gap:10px;}
.page-header h1 i{color:var(--primary);}
.card{background:var(--card-bg);border-radius:12px;padding:20px;margin-bottom:20px;box-shadow:var(--shadow);}
.card-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px;padding-bottom:10px;border-bottom:1px solid #2a2a4e;}
.card-header h3{color:#fff;font-size:1.1em;display:flex;align-items:center;gap:10px;}
.card-header h3 i{color:var(--primary);}
.refresh-time{font-size:0.8em;color:#888;}
.stats-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;margin-bottom:20px;}
.stat-card{background:rgba(42,42,78,0.5);border-radius:8px;padding:15px;text-align:center;}
.stat-card .stat-value{font-size:1.8em;font-weight:bold;color:#fff;margin:5px 0;}
.stat-card .stat-label{color:#888;font-size:0.85em;}
.stat-card .stat-icon{font-size:2em;margin-bottom:5px;}
table{width:100%;border-collapse:collapse;}
td{padding:10px;border-bottom:1px solid #2a2a4e;}
td:first-child{font-weight:bold;color:#aaa;width:40%;}
td:last-child{color:#fff;}
.badge{display:inline-block;padding:5px 12px;border-radius:20px;font-weight:bold;font-size:0.85em;}
.badge-normal{background:#9E9E9E;color:#000;}
.badge-soft{background:#4CAF50;color:#fff;}
.badge-moderate{background:#FFEB3B;color:#000;}
.badge-hard{background:#F44336;color:#fff;}
.badge-emergency{background:#FF9800;color:#fff;animation:blink 0.8s infinite;}
@keyframes blink{0%,100%{opacity:1;}50%{opacity:0.4;}}
.status-ok{color:#4CAF50;}
.status-fail{color:#F44336;}
.status-warning{color:#FF9800;}
input,select{width:100%;padding:10px;margin:5px 0 15px 0;border-radius:6px;border:1px solid #333;background:#2a2a4e;color:#fff;font-size:14px;}
input:focus{outline:2px solid var(--primary);border-color:transparent;}
.btn{display:inline-flex;align-items:center;gap:8px;padding:10px 20px;border:none;border-radius:6px;cursor:pointer;font-size:14px;font-weight:600;transition:all 0.3s;text-decoration:none;}
.btn-primary{background:var(--primary);color:#fff;}
.btn-primary:hover{background:#c73652;transform:translateY(-2px);}
.btn-success{background:#4CAF50;color:#fff;}
.btn-success:hover{background:#43a047;transform:translateY(-2px);}
.btn-warning{background:#FF9800;color:#fff;}
.btn-warning:hover{background:#F57C00;transform:translateY(-2px);}
.btn-danger{background:#F44336;color:#fff;}
.btn-danger:hover{background:#E53935;transform:translateY(-2px);}
.btn-secondary{background:#555;color:#fff;}
.btn-secondary:hover{background:#666;transform:translateY(-2px);}
.btn-sm{padding:6px 12px;font-size:12px;}
.button-group{display:flex;gap:10px;flex-wrap:wrap;margin:10px 0;}
@media(max-width:768px){
.sidebar{transform:translateX(-100%);position:fixed;width:280px;z-index:1001;}
.sidebar.open{transform:translateX(0);}
.main-content{margin-left:0;width:100%;}
.menu-toggle{display:block !important;z-index:1002;position:relative;}
.stats-grid{grid-template-columns:repeat(auto-fit,minmax(150px,1fr));}
}
.menu-toggle{display:none;background:var(--primary);color:#fff;border:none;padding:10px 15px;border-radius:6px;font-size:1.2em;cursor:pointer;margin-bottom:15px;}
@media(min-width:769px){.menu-toggle{display:none;}}
.footer{text-align:center;padding:20px 0;color:#666;font-size:0.85em;border-top:1px solid #2a2a4e;margin-top:20px;}
</style>
</head>
<body>
<button class="menu-toggle" onclick="toggleSidebar()"><i class="fas fa-bars"></i> Menu</button>

<div class="sidebar" id="sidebar">
<div class="sidebar-brand">
<i class="fas fa-car-crash"></i>
<h2>BRAKESAFE</h2>
</div>
<a class="nav-item active" href="/"><i class="fas fa-tachometer-alt"></i> Dashboard</a>
<a class="nav-item" href="/map"><i class="fas fa-map-marked-alt"></i> Bantayan Map</a>
<a class="nav-item" href="/apa"><i class="fas fa-exclamation-triangle"></i> APA Map</a>
<a class="nav-item" href="/wifi"><i class="fas fa-wifi"></i> WiFi Settings</a>
<a class="nav-item" href="/system"><i class="fas fa-cog"></i> System Info</a>
<div class="sidebar-footer">
<i class="fas fa-microchip"></i> v1
</div>
</div>

<div class="main-content">
<div class="container">
<div class="page-header">
<h1><i class="fas fa-tachometer-alt"></i> Dashboard</h1>
</div>

<div class="stats-grid" id="statsGrid">
<div class="stat-card"><div class="stat-icon"><i class="fas fa-satellite-dish" style="color:#4CAF50;"></i></div><div class="stat-value" id="gpsStatus">--</div><div class="stat-label">GPS</div></div>
<div class="stat-card"><div class="stat-icon"><i class="fas fa-microchip" style="color:#9C27B0;"></i></div><div class="stat-value" id="mpuStatus">--</div><div class="stat-label">MPU6050</div></div>
<div class="stat-card"><div class="stat-icon"><i class="fas fa-tachometer-alt" style="color:#2196F3;"></i></div><div class="stat-value" id="speed">--</div><div class="stat-label">Speed (km/h)</div></div>
<div class="stat-card"><div class="stat-icon"><i class="fas fa-compass" style="color:#00BCD4;"></i></div><div class="stat-value" id="direction">--</div><div class="stat-label">Direction</div></div>
<div class="stat-card"><div class="stat-icon"><i class="fas fa-braille" style="color:#FF9800;"></i></div><div class="stat-value" id="brakeLevel">--</div><div class="stat-label">Brake Level</div></div>
<div class="stat-card"><div class="stat-icon"><i class="fas fa-bell" style="color:#F44336;"></i></div><div class="stat-value" id="buzzerStatus">--</div><div class="stat-label">Emergency</div></div>
</div>

<div class="card">
<div class="card-header">
<h3><i class="fas fa-info-circle"></i> Live Data</h3>
<span class="refresh-time"><i class="fas fa-sync-alt fa-spin"></i> <span id="updateTime">--</span></span>
</div>
<table>
<tr><td><i class="fas fa-map-pin"></i> Latitude</td><td id="lat">--</td></tr>
<tr><td><i class="fas fa-map-pin"></i> Longitude</td><td id="lng">--</td></tr>
<tr><td><i class="fas fa-clock"></i> Time (GPS)</td><td id="time">--</td></tr>
<tr><td><i class="fas fa-braille"></i> Deceleration</td><td id="decel">-- g</td></tr>
<tr><td><i class="fas fa-database"></i> Events Recorded</td><td id="queue">--</td></tr>
<tr><td><i class="fas fa-history"></i> Last Event</td><td id="lastEvent">--</td></tr>
<tr><td><i class="fas fa-wifi"></i> WiFi Status</td><td id="wifiStatus">--</td></tr>
</table>
</div>

<div class="card">
<div class="card-header">
<h3><i class="fas fa-download"></i> Data Export</h3>
</div>
<div class="button-group">
<button class="btn btn-primary" onclick="downloadEvents()"><i class="fas fa-file-csv"></i> Brake Events</button>
<button class="btn btn-success" onclick="downloadAPA()"><i class="fas fa-file-csv"></i> APA Data</button>
</div>
</div>

<div class="footer">
<i class="fas fa-copyright"></i> 2026 BRAKESAFE System
</div>
</div>
</div>

<script>
function toggleSidebar() {
document.getElementById('sidebar').classList.toggle('open');
}

function fetchStatus(){
fetch('/api/status', {cache:'no-store'})
.then(r=>r.json())
.then(d=>{
document.getElementById('updateTime').textContent = new Date().toLocaleTimeString();
document.getElementById('lat').textContent = d.latitude.toFixed(6);
document.getElementById('lng').textContent = d.longitude.toFixed(6);
document.getElementById('speed').textContent = d.speed.toFixed(2);
document.getElementById('time').textContent = d.timestamp;
document.getElementById('decel').textContent = d.deceleration_g.toFixed(3);
document.getElementById('direction').textContent = d.direction + ' (' + d.heading.toFixed(0) + '\u00B0)';

let lvl = d.brake_level;
let badge = document.getElementById('brakeLevel');
badge.textContent = lvl;
badge.className = 'badge badge-' + lvl.toLowerCase();

let gpsFixed = (d.latitude!==0 || d.longitude!==0);
let gEl = document.getElementById('gpsStatus');
gEl.textContent = gpsFixed ? 'Fixed' : (d.gps_module ? 'Waiting' : 'No Module');
gEl.className = 'stat-value ' + (gpsFixed ? 'status-ok' : 'status-warning');

let mEl = document.getElementById('mpuStatus');
if (d.mpu_ready) { mEl.textContent = 'Ready'; mEl.className='stat-value status-ok'; }
else { mEl.textContent = 'Disabled'; mEl.className='stat-value status-warning'; }

document.getElementById('queue').textContent = d.queue_size;
document.getElementById('lastEvent').textContent = d.last_event||'None';

let wifiText = d.wifi_connected ? ('Connected: ' + d.wifi_ssid) : (d.ap_mode ? 'AP Mode' : 'Disconnected');
let wifiClass = d.wifi_connected ? 'status-ok' : (d.ap_mode ? 'status-warning' : 'status-fail');
document.getElementById('wifiStatus').textContent = wifiText;
document.getElementById('wifiStatus').className = wifiClass;

let bEl = document.getElementById('buzzerStatus');
bEl.textContent = d.emergency_buzzer ? 'Active' : 'Inactive';
bEl.className = 'stat-value ' + (d.emergency_buzzer ? 'status-fail' : 'status-ok');
}).catch(e=>console.error(e));
}

function downloadEvents() { window.location.href = '/download/events'; }
function downloadAPA()   { window.location.href = '/download/apa'; }

fetchStatus();
setInterval(fetchStatus, 2000);

document.addEventListener('click', function(e) {
const sidebar = document.getElementById('sidebar');
const toggle = document.querySelector('.menu-toggle');
if (window.innerWidth <= 768 && sidebar.classList.contains('open') &&
!sidebar.contains(e.target) && !toggle.contains(e.target)) {
sidebar.classList.remove('open');
}
});
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ===================== BANTAYAN MAP PAGE (heatmap + track + scatter) =====================
void handleMapPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>BRAKESAFE - Bantayan Map</title>
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 512 512'><path fill='%23e94560' d='M135.2 117.4L109.1 192H402.9l-26.1-74.6C372.3 104.6 360.2 96 346.6 96H165.4c-13.6 0-25.7 8.6-30.2 21.4zM39.6 196.8L74.8 96.3C88.3 57.8 124.6 32 165.4 32H346.6c40.8 0 77.1 25.8 90.6 64.3l35.2 100.5c23.2 9.6 39.6 32.5 39.6 59.2V400v48c0 17.7-14.3 32-32 32H448c-17.7 0-32-14.3-32-32V400H96v48c0 17.7-14.3 32-32 32H32c-17.7 0-32-14.3-32-32V400 256c0-26.7 16.4-49.6 39.6-59.2zM128 288a32 32 0 1 0 -64 0 32 32 0 1 0 64 0zm288 32a32 32 0 1 0 0-64 32 32 0 1 0 0 64z'/></svg>">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<link rel="stylesheet" href="https://unpkg.com/leaflet.markercluster@1.5.3/dist/MarkerCluster.css" />
<link rel="stylesheet" href="https://unpkg.com/leaflet.markercluster@1.5.3/dist/MarkerCluster.Default.css" />
<style>
*{margin:0;padding:0;box-sizing:border-box;}
:root{--primary:#e94560;--secondary:#16213e;--bg:#1a1a2e;--card-bg:#1e2a4a;--text:#eee;}
body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:var(--bg);color:var(--text);display:flex;flex-wrap:wrap;min-height:100vh;}
.sidebar{width:260px;background:var(--secondary);min-height:100vh;padding:20px;position:fixed;left:0;top:0;overflow-y:auto;border-right:2px solid #2a2a4e;z-index:1000;transition:transform 0.3s ease;}
.sidebar-brand{text-align:center;padding-bottom:20px;border-bottom:2px solid #2a2a4e;margin-bottom:20px;}
.sidebar-brand i{font-size:2em;color:var(--primary);}
.sidebar-brand h2{color:#fff;font-size:1.3em;margin-top:5px;}
.nav-item{display:flex;align-items:center;padding:12px 15px;margin:5px 0;border-radius:8px;cursor:pointer;transition:all 0.3s;color:#aaa;text-decoration:none;}
.nav-item i{width:25px;font-size:1.1em;margin-right:10px;}
.nav-item:hover{background:#2a2a4e;color:#fff;}
.nav-item.active{background:var(--primary);color:#fff;}
.sidebar-footer{font-size:0.8em;color:#666;text-align:center;margin-top:30px;padding-top:20px;border-top:2px solid #2a2a4e;}
.main-content{margin-left:260px;padding:20px;flex:1;min-width:300px;width:calc(100% - 260px);}
.container{max-width:1200px;margin:0 auto;}
.page-header{margin-bottom:25px;}
.page-header h1{color:#fff;font-size:1.8em;display:flex;align-items:center;gap:10px;}
.page-header h1 i{color:var(--primary);}
.card{background:var(--card-bg);border-radius:12px;padding:20px;margin-bottom:20px;box-shadow:0 4px 6px rgba(0,0,0,0.3);}
.card-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px;padding-bottom:10px;border-bottom:1px solid #2a2a4e;}
.card-header h3{color:#fff;font-size:1.1em;display:flex;align-items:center;gap:10px;}
.card-header h3 i{color:var(--primary);}
.map-container{width:100%;height:600px;border-radius:12px;overflow:hidden;position:relative;}
#map{height:100%;width:100%;}
.legend-badge{position:absolute;bottom:30px;left:20px;background:rgba(26,26,46,0.92);padding:14px 20px;border-radius:12px;backdrop-filter:blur(10px);border:1px solid rgba(255,255,255,0.08);z-index:1000;min-width:190px;}
.legend-badge .dot{display:inline-block;width:14px;height:14px;border-radius:50%;margin-right:10px;border:1px solid rgba(255,255,255,0.15);}
.legend-badge .dot.soft{background:#4CAF50;}
.legend-badge .dot.moderate{background:#FFEB3B;border-color:rgba(0,0,0,0.3);}
.legend-badge .dot.hard{background:#F44336;}
.legend-badge .dot.emergency{background:#FF9800;}
.legend-badge .dot.track{background:transparent;border:none;width:26px;height:0;border-top:4px dashed #fff;display:inline-block;vertical-align:middle;border-radius:0;margin-right:10px;}
.legend-item{display:flex;align-items:center;gap:10px;margin:6px 0;color:#eee;font-size:0.85em;font-weight:500;}
.legend-title{color:#aaa;font-size:0.7em;text-transform:uppercase;letter-spacing:1.5px;margin-bottom:8px;border-bottom:1px solid rgba(255,255,255,0.06);padding-bottom:8px;}
.heat-gradient{height:14px;width:100%;border-radius:7px;margin:6px 0 4px 0;background:linear-gradient(to right,#0000ff,#00ffff,#00ff00,#ffff00,#ff0000);border:1px solid rgba(255,255,255,0.15);}
.heat-labels{display:flex;justify-content:space-between;font-size:0.7em;color:#aaa;}
.loading-overlay{position:absolute;top:0;left:0;width:100%;height:100%;background:rgba(26,26,46,0.7);display:flex;align-items:center;justify-content:center;z-index:999;backdrop-filter:blur(4px);}
.loading-overlay .spinner{width:50px;height:50px;border:4px solid rgba(255,255,255,0.1);border-top-color:#e94560;border-radius:50%;animation:spin 1s linear infinite;}
@keyframes spin{to{transform:rotate(360deg);}}
.loading-overlay p{color:#fff;margin-top:15px;font-size:0.9em;}
.menu-toggle{display:none;background:var(--primary);color:#fff;border:none;padding:10px 15px;border-radius:6px;font-size:1.2em;cursor:pointer;margin-bottom:15px;}
@media(max-width:768px){
.sidebar{transform:translateX(-100%);position:fixed;width:280px;z-index:1001;}
.sidebar.open{transform:translateX(0);}
.main-content{margin-left:0;width:100%;}
.menu-toggle{display:block !important;z-index:1002;position:relative;}
}
.leaflet-container{z-index:1;}
.footer{text-align:center;padding:20px 0;color:#666;font-size:0.85em;border-top:1px solid #2a2a4e;margin-top:20px;}
.event-counter{position:absolute;top:15px;right:15px;background:rgba(26,26,46,0.92);padding:8px 14px;border-radius:8px;backdrop-filter:blur(10px);border:1px solid rgba(255,255,255,0.08);z-index:1000;color:#fff;font-size:0.85em;font-weight:600;}
.event-counter .pulse{display:inline-block;width:8px;height:8px;background:#4CAF50;border-radius:50%;margin-right:6px;animation:pulse 1.5s infinite;}
@keyframes pulse{0%,100%{opacity:1;}50%{opacity:0.3;}}
.zoom-hint{position:absolute;top:15px;left:15px;background:rgba(233,69,96,0.92);padding:8px 14px;border-radius:8px;backdrop-filter:blur(10px);z-index:1000;color:#fff;font-size:0.8em;font-weight:600;display:none;}
.view-toggle{position:absolute;top:15px;left:50%;transform:translateX(-50%);background:rgba(26,26,46,0.92);border-radius:8px;backdrop-filter:blur(10px);border:1px solid rgba(255,255,255,0.08);z-index:1000;display:flex;overflow:hidden;}
.view-toggle button{background:transparent;border:none;color:#aaa;padding:8px 16px;font-size:0.85em;font-weight:600;cursor:pointer;transition:all 0.2s;font-family:inherit;}
.view-toggle button.active{background:var(--primary);color:#fff;}
.view-toggle button:hover:not(.active){color:#fff;background:rgba(255,255,255,0.05);}
</style>
</head>
<body>
<button class="menu-toggle" onclick="toggleSidebar()"><i class="fas fa-bars"></i> Menu</button>

<div class="sidebar" id="sidebar">
<div class="sidebar-brand">
<i class="fas fa-car-crash"></i>
<h2>BRAKESAFE</h2>
</div>
<a class="nav-item" href="/"><i class="fas fa-tachometer-alt"></i> Dashboard</a>
<a class="nav-item active" href="/map"><i class="fas fa-map-marked-alt"></i> Bantayan Map</a>
<a class="nav-item" href="/apa"><i class="fas fa-exclamation-triangle"></i> APA Map</a>
<a class="nav-item" href="/wifi"><i class="fas fa-wifi"></i> WiFi Settings</a>
<a class="nav-item" href="/system"><i class="fas fa-cog"></i> System Info</a>
<div class="sidebar-footer">
<i class="fas fa-microchip"></i> v1
</div>
</div>

<div class="main-content">
<div class="container">
<div class="page-header">
<h1><i class="fas fa-map-marked-alt"></i> Bantayan Island Map</h1>
</div>

<div class="card">
<div class="card-header">
<h3><i class="fas fa-fire"></i> Brake Event Heatmap</h3>
</div>
<div class="map-container">
<div class="loading-overlay" id="loadingOverlay">
<div><div class="spinner"></div><p>Loading map data...</p></div>
</div>
<div id="map"></div>
<div class="event-counter" id="eventCounter"><span class="pulse"></span><span id="eventCount">0</span> events</div>
<div class="zoom-hint" id="zoomHint">Zoomed in — track hidden</div>
<div class="view-toggle" id="viewToggle">
<button id="btnHeat" class="active" onclick="setView('heat')"><i class="fas fa-fire"></i> Heatmap</button>
<button id="btnMarkers" onclick="setView('markers')"><i class="fas fa-map-marker-alt"></i> Markers</button>
</div>
<div class="legend-badge">
<div class="legend-title" id="legendTitle">Heatmap Density</div>
<div id="legendHeat">
<div class="heat-gradient"></div>
<div class="heat-labels"><span>Low</span><span>Medium</span><span>High</span></div>
<div class="legend-item" style="margin-top:10px;border-top:1px solid rgba(255,255,255,0.08);padding-top:8px;"><span class="dot track"></span> Vehicle Track</div>
</div>
<div id="legendMarkers" style="display:none;">
<div class="legend-item"><span class="dot soft"></span> Soft Brake</div>
<div class="legend-item"><span class="dot moderate"></span> Moderate Brake</div>
<div class="legend-item"><span class="dot hard"></span> Hard Brake</div>
<div class="legend-item"><span class="dot emergency"></span> Emergency Brake</div>
<div class="legend-item" style="border-top:1px solid rgba(255,255,255,0.08);padding-top:8px;margin-top:8px;"><span class="dot track"></span> Vehicle Track</div>
</div>
</div>
</div>
</div>

<div class="footer">
<i class="fas fa-copyright"></i> 2026 BRAKESAFE System
</div>
</div>
</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://unpkg.com/leaflet.markercluster@1.5.3/dist/leaflet.markercluster.js"></script>
<script src="https://unpkg.com/leaflet.heat@0.2.0/dist/leaflet-heat.js"></script>
<script>
function toggleSidebar() {
document.getElementById('sidebar').classList.toggle('open');
}

let map = null;
let markerGroup = null;
let heatLayer = null;
let trackLayer = null;
let lastEventSignature = '';
let cachedData = [];
let currentView = 'heat';
const SCATTER_ZOOM = 13;

function setView(v) {
  currentView = v;
  document.getElementById('btnHeat').classList.toggle('active', v === 'heat');
  document.getElementById('btnMarkers').classList.toggle('active', v === 'markers');
  document.getElementById('legendHeat').style.display    = (v === 'heat')    ? 'block' : 'none';
  document.getElementById('legendMarkers').style.display = (v === 'markers') ? 'block' : 'none';
  document.getElementById('legendTitle').textContent =
    (v === 'heat') ? 'Heatmap Density' : 'Brake Events';
  redraw();
}

function computeIntensity(event) {
  switch(event.level) {
    case 'Soft':      return 0.35;
    case 'Moderate':  return 0.6;
    case 'Hard':      return 0.85;
    case 'Emergency': return 1.0;
    default:          return 0.3;
  }
}

function redraw() {
  if (!map) return;

  if (heatLayer)    { map.removeLayer(heatLayer);   heatLayer = null; }
  if (trackLayer)   { map.removeLayer(trackLayer);  trackLayer = null; }
  if (markerGroup)  markerGroup.clearLayers();

  const data = cachedData;
  if (!data || data.length === 0) return;

  const points = [];

  if (currentView === 'heat') {
    const heatPoints = data.map(ev => [ev.lat, ev.lng, computeIntensity(ev)]);
    heatLayer = L.heatLayer(heatPoints, {
      radius: 45,
      blur: 35,
      maxZoom: 17,
      max: 1.0,
      minOpacity: 0.35,
      gradient: {
        0.0: '#0000ff',
        0.25: '#00ffff',
        0.5: '#00ff00',
        0.7: '#ffff00',
        0.9: '#ff8800',
        1.0: '#ff0000'
      }
    }).addTo(map);
  }

  if (currentView === 'markers') {
    const layers = [];
    data.forEach((event, i) => {
      let color = '#9E9E9E';
      let label = 'Normal';
      let radius = 7;
      switch(event.level) {
        case 'Soft':      color='#4CAF50'; label='Soft';      radius=7;  break;
        case 'Moderate':  color='#FFEB3B'; label='Moderate';  radius=8;  break;
        case 'Hard':      color='#F44336'; label='Hard';      radius=9;  break;
        case 'Emergency': color='#FF9800'; label='Emergency'; radius=10; break;
      }
      const marker = L.circleMarker([event.lat, event.lng], {
        radius: radius,
        fillColor: color,
        color: '#fff',
        weight: 2,
        opacity: 1,
        fillOpacity: 0.85
      });
      marker.bindPopup(`
<b>${label} Brake</b> <span style="color:#888;font-size:0.85em;">#${i+1}</span><br>
<i class="fas fa-map-pin"></i> Lat: ${event.lat.toFixed(6)}<br>
<i class="fas fa-map-pin"></i> Lng: ${event.lng.toFixed(6)}<br>
<i class="fas fa-tachometer-alt"></i> Speed: ${event.speed.toFixed(2)} km/h<br>
<i class="fas fa-braille"></i> Decel: ${event.decel.toFixed(2)} g<br>
<i class="fas fa-clock"></i> Time: ${event.timestamp}<br>
<i class="fas fa-compass"></i> Direction: ${event.direction || 'N'}
`);
      layers.push(marker);
    });
    markerGroup.addLayers(layers);
  }

  data.forEach(ev => points.push([ev.lat, ev.lng]));
  if (points.length >= 2) {
    const glow = L.polyline(points, {
      color: '#ffffff', weight: 10, opacity: 0.08,
      lineCap: 'round', lineJoin: 'round', interactive: false
    });
    const main = L.polyline(points, {
      color: '#ffffff', weight: 2.5, opacity: 0.85,
      dashArray: '10,10', lineCap: 'round', lineJoin: 'round', interactive: false
    });
    const center = L.polyline(points, {
      color: '#e94560', weight: 1.5, opacity: 0.7,
      lineCap: 'round', lineJoin: 'round', interactive: false
    });
    trackLayer = L.layerGroup([glow, main, center]).addTo(map);
  }

  applyZoomRule(map.getZoom());
}

function refreshMarkers() {
  fetch('/api/events?limit=5000', {cache:'no-store'})
  .then(r=>r.json())
  .then(data=>{
    // Reject (0,0) rows defensively
    data = data.filter(ev => Math.abs(ev.lat) > 0.000001 || Math.abs(ev.lng) > 0.000001);

    const sig = data.length + '|' + (data.length ? data[data.length-1].timestamp : '');
    if (sig === lastEventSignature) return;
    lastEventSignature = sig;

    cachedData = data;
    document.getElementById('eventCount').textContent = data.length;
    redraw();
  })
  .catch(e=>console.error(e));
}

function applyZoomRule(zoom) {
  if (!trackLayer) return;
  const hint = document.getElementById('zoomHint');
  if (zoom >= SCATTER_ZOOM) {
    if (map.hasLayer(trackLayer)) map.removeLayer(trackLayer);
    hint.style.display = 'block';
    setTimeout(() => { hint.style.display = 'none'; }, 1800);
  } else {
    if (!map.hasLayer(trackLayer)) map.addLayer(trackLayer);
  }
}

function initMap() {
  document.getElementById('loadingOverlay').style.display = 'none';

  map = L.map('map').setView([11.2200, 123.7600], 11);

  L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Dark_Gray_Base/MapServer/tile/{z}/{y}/{x}', {
    attribution: '&copy; Esri, HERE, Garmin, &copy; OpenStreetMap contributors',
    maxZoom: 16
  }).addTo(map);

  markerGroup = L.markerClusterGroup({
    maxClusterRadius: 50,
    disableClusteringAtZoom: 16,
    spiderfyOnMaxZoom: true,
    showCoverageOnHover: false,
    chunkedLoading: true,
    chunkInterval: 100,
    chunkDelay: 30
  });
  map.addLayer(markerGroup);

  map.on('zoomend', function() {
    applyZoomRule(map.getZoom());
  });

  refreshMarkers();
  setInterval(refreshMarkers, 5000);
}

document.addEventListener('DOMContentLoaded', function() {
  initMap();
});

document.addEventListener('click', function(e) {
  const sidebar = document.getElementById('sidebar');
  const toggle = document.querySelector('.menu-toggle');
  if (window.innerWidth <= 768 && sidebar.classList.contains('open') &&
      !sidebar.contains(e.target) && !toggle.contains(e.target)) {
    sidebar.classList.remove('open');
  }
});
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ===================== APA PAGE =====================
void handleAPAPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>BRAKESAFE - APA Map</title>
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 512 512'><path fill='%23e94560' d='M135.2 117.4L109.1 192H402.9l-26.1-74.6C372.3 104.6 360.2 96 346.6 96H165.4c-13.6 0-25.7 8.6-30.2 21.4zM39.6 196.8L74.8 96.3C88.3 57.8 124.6 32 165.4 32H346.6c40.8 0 77.1 25.8 90.6 64.3l35.2 100.5c23.2 9.6 39.6 32.5 39.6 59.2V400v48c0 17.7-14.3 32-32 32H448c-17.7 0-32-14.3-32-32V400H96v48c0 17.7-14.3 32-32 32H32c-17.7 0-32-14.3-32-32V400 256c0-26.7 16.4-49.6 39.6-59.2zM128 288a32 32 0 1 0 -64 0 32 32 0 1 0 64 0zm288 32a32 32 0 1 0 0-64 32 32 0 1 0 0 64z'/></svg>">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://cdn.jsdelivr.net/npm/sweetalert2@11"></script>
<style>
*{margin:0;padding:0;box-sizing:border-box;}
:root{--primary:#e94560;--secondary:#16213e;--bg:#1a1a2e;--card-bg:#1e2a4a;--text:#eee;}
body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:var(--bg);color:var(--text);display:flex;flex-wrap:wrap;min-height:100vh;}
.sidebar{width:260px;background:var(--secondary);min-height:100vh;padding:20px;position:fixed;left:0;top:0;overflow-y:auto;border-right:2px solid #2a2a4e;z-index:1000;transition:transform 0.3s ease;}
.sidebar-brand{text-align:center;padding-bottom:20px;border-bottom:2px solid #2a2a4e;margin-bottom:20px;}
.sidebar-brand i{font-size:2em;color:var(--primary);}
.sidebar-brand h2{color:#fff;font-size:1.3em;margin-top:5px;}
.nav-item{display:flex;align-items:center;padding:12px 15px;margin:5px 0;border-radius:8px;cursor:pointer;transition:all 0.3s;color:#aaa;text-decoration:none;}
.nav-item i{width:25px;font-size:1.1em;margin-right:10px;}
.nav-item:hover{background:#2a2a4e;color:#fff;}
.nav-item.active{background:var(--primary);color:#fff;}
.sidebar-footer{font-size:0.8em;color:#666;text-align:center;margin-top:30px;padding-top:20px;border-top:2px solid #2a2a4e;}
.main-content{margin-left:260px;padding:20px;flex:1;min-width:300px;width:calc(100% - 260px);}
.container{max-width:1200px;margin:0 auto;}
.page-header{margin-bottom:25px;}
.page-header h1{color:#fff;font-size:1.8em;display:flex;align-items:center;gap:10px;}
.page-header h1 i{color:var(--primary);}
.card{background:var(--card-bg);border-radius:12px;padding:20px;margin-bottom:20px;box-shadow:0 4px 6px rgba(0,0,0,0.3);}
.card-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px;padding-bottom:10px;border-bottom:1px solid #2a2a4e;}
.card-header h3{color:#fff;font-size:1.1em;display:flex;align-items:center;gap:10px;}
.card-header h3 i{color:var(--primary);}
.map-container{width:100%;height:550px;border-radius:12px;overflow:hidden;position:relative;}
#map{height:100%;width:100%;}
.legend-badge{position:absolute;bottom:30px;left:20px;background:rgba(26,26,46,0.92);padding:14px 20px;border-radius:12px;backdrop-filter:blur(10px);border:1px solid rgba(255,255,255,0.08);z-index:1000;}
.legend-badge .dot{display:inline-block;width:14px;height:14px;border-radius:50%;margin-right:10px;border:1px solid rgba(255,255,255,0.15);}
.legend-badge .dot.accident{background:#ef4444;}
.legend-item{display:flex;align-items:center;gap:10px;margin:6px 0;color:#eee;font-size:0.85em;font-weight:500;}
.legend-title{color:#aaa;font-size:0.7em;text-transform:uppercase;letter-spacing:1.5px;margin-bottom:8px;border-bottom:1px solid rgba(255,255,255,0.06);padding-bottom:8px;}
.loading-overlay{position:absolute;top:0;left:0;width:100%;height:100%;background:rgba(26,26,46,0.7);display:flex;align-items:center;justify-content:center;z-index:999;backdrop-filter:blur(4px);}
.loading-overlay .spinner{width:50px;height:50px;border:4px solid rgba(255,255,255,0.1);border-top-color:#e94560;border-radius:50%;animation:spin 1s linear infinite;}
@keyframes spin{to{transform:rotate(360deg);}}
.loading-overlay p{color:#fff;margin-top:15px;font-size:0.9em;}
.apa-list{max-height:300px;overflow-y:auto;}
.apa-item{background:rgba(42,42,78,0.5);padding:10px;margin:5px 0;border-radius:6px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px;}
.apa-item .apa-reason{color:#fff;flex:1;}
.apa-item .apa-coords{color:#888;font-size:0.85em;}
.apa-item .apa-remove{cursor:pointer;color:#F44336;padding:5px 10px;border-radius:4px;transition:background 0.3s;}
.apa-item .apa-remove:hover{background:#2a2a4e;}
input,textarea{width:100%;padding:10px;margin:5px 0 15px 0;border-radius:6px;border:1px solid #333;background:#2a2a4e;color:#fff;font-size:14px;}
input:focus,textarea:focus{outline:2px solid var(--primary);border-color:transparent;}
.btn{display:inline-flex;align-items:center;gap:8px;padding:10px 20px;border:none;border-radius:6px;cursor:pointer;font-size:14px;font-weight:600;transition:all 0.3s;text-decoration:none;}
.btn-primary{background:var(--primary);color:#fff;}
.btn-primary:hover{background:#c73652;transform:translateY(-2px);}
.btn-danger{background:#F44336;color:#fff;}
.btn-danger:hover{background:#E53935;transform:translateY(-2px);}
.btn-success{background:#4CAF50;color:#fff;}
.btn-success:hover{background:#43a047;transform:translateY(-2px);}
.btn-secondary{background:#555;color:#fff;}
.btn-secondary:hover{background:#666;transform:translateY(-2px);}
.btn-sm{padding:6px 12px;font-size:12px;}
.menu-toggle{display:none;background:var(--primary);color:#fff;border:none;padding:10px 15px;border-radius:6px;font-size:1.2em;cursor:pointer;margin-bottom:15px;}
@media(max-width:768px){
.sidebar{transform:translateX(-100%);position:fixed;width:280px;z-index:1001;}
.sidebar.open{transform:translateX(0);}
.main-content{margin-left:0;width:100%;}
.menu-toggle{display:block !important;z-index:1002;position:relative;}
}
.leaflet-container{z-index:1;}
.footer{text-align:center;padding:20px 0;color:#666;font-size:0.85em;border-top:1px solid #2a2a4e;margin-top:20px;}
.apa-edit-modal{position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.85);z-index:9999;display:none;align-items:center;justify-content:center;}
.apa-edit-modal .modal-content{background:#1e2a4a;padding:25px;border-radius:12px;max-width:500px;width:90%;border:1px solid rgba(255,255,255,0.1);box-shadow:0 10px 40px rgba(0,0,0,0.5);}
.apa-edit-modal .modal-content h3{color:#fff;margin-bottom:15px;display:flex;align-items:center;gap:10px;}
.apa-edit-modal .modal-content textarea{width:100%;padding:10px;background:#2a2a4e;color:#fff;border:1px solid #333;border-radius:6px;margin:10px 0;min-height:80px;font-size:14px;resize:vertical;font-family:inherit;}
.apa-edit-modal .modal-content textarea:focus{outline:2px solid var(--primary);border-color:transparent;}
.apa-edit-modal .modal-content .info-row{display:flex;gap:10px;margin:5px 0;color:#aaa;font-size:0.85em;background:rgba(0,0,0,0.3);padding:8px 12px;border-radius:4px;align-items:center;}
.apa-edit-modal .modal-content .info-row i{width:20px;color:#888;}
.apa-edit-modal .modal-content .info-row .value{color:#eee;}
.apa-edit-modal .modal-content .btn-group{display:flex;gap:10px;margin-top:15px;}
.apa-edit-modal .modal-content .btn-group .btn{flex:1;justify-content:center;}
</style>
</head>
<body>
<button class="menu-toggle" onclick="toggleSidebar()"><i class="fas fa-bars"></i> Menu</button>

<div class="sidebar" id="sidebar">
<div class="sidebar-brand">
<i class="fas fa-car-crash"></i>
<h2>BRAKESAFE</h2>
</div>
<a class="nav-item" href="/"><i class="fas fa-tachometer-alt"></i> Dashboard</a>
<a class="nav-item" href="/map"><i class="fas fa-map-marked-alt"></i> Bantayan Map</a>
<a class="nav-item active" href="/apa"><i class="fas fa-exclamation-triangle"></i> APA Map</a>
<a class="nav-item" href="/wifi"><i class="fas fa-wifi"></i> WiFi Settings</a>
<a class="nav-item" href="/system"><i class="fas fa-cog"></i> System Info</a>
<div class="sidebar-footer">
<i class="fas fa-microchip"></i> v1
</div>
</div>

<div class="main-content">
<div class="container">
<div class="page-header">
<h1><i class="fas fa-exclamation-triangle"></i> Accident Prone Areas</h1>
</div>

<div class="card">
<div class="card-header">
<h3><i class="fas fa-plus-circle"></i> Add Accident Prone Area</h3>
</div>
<form id="apaForm">
<div style="margin-bottom:15px;">
<label><i class="fas fa-map-pin"></i> Click on map to get coordinates</label>
<div class="map-container">
<div class="loading-overlay" id="loadingOverlay">
<div><div class="spinner"></div><p>Loading map...</p></div>
</div>
<div id="map"></div>
<div class="legend-badge">
<div class="legend-title">APA Locations</div>
<div class="legend-item"><span class="dot accident"></span> Accident Prone Area</div>
</div>
</div>
</div>

<div style="display:grid;grid-template-columns:1fr 1fr;gap:15px;">
<div>
<label><i class="fas fa-latitude"></i> Latitude:</label>
<input type="text" id="lat" name="lat" placeholder="11.1500" required>
</div>
<div>
<label><i class="fas fa-longitude-alt"></i> Longitude:</label>
<input type="text" id="lng" name="lng" placeholder="123.7200" required>
</div>
</div>

<div>
<label><i class="fas fa-comment"></i> Reason:</label>
<textarea id="reason" name="reason" rows="2" placeholder="Why is this area accident prone?" required></textarea>
</div>
<button type="button" class="btn btn-success" onclick="addAPA()"><i class="fas fa-save"></i> Add APA</button>
</form>
</div>

<div class="card">
<div class="card-header">
<h3><i class="fas fa-list"></i> Accident Prone Areas List</h3>
<span id="apaCount">0 locations</span>
</div>
<div class="apa-list" id="apaList">
<p style="color:#888;">Loading...</p>
</div>
</div>

<div class="footer">
<i class="fas fa-copyright"></i> 2026 BRAKESAFE System
</div>
</div>
</div>

<div class="apa-edit-modal" id="apaEditModal">
<div class="modal-content">
<h3><i class="fas fa-edit" style="color:#e94560;"></i> Edit Accident Prone Area</h3>
<input type="hidden" id="editApaIndex">
<div class="info-row"><i class="fas fa-map-pin"></i> <span class="value" id="editCoords">--</span></div>
<div class="info-row"><i class="fas fa-clock"></i> <span class="value" id="editTimestamp">--</span></div>
<div style="margin-top:10px;">
<label style="color:#aaa;font-size:0.85em;display:block;margin-bottom:5px;"><i class="fas fa-comment"></i> Reason / Description</label>
<textarea id="editReason" placeholder="Describe why this is an accident prone area..."></textarea>
</div>
<div class="btn-group">
<button class="btn btn-success" onclick="saveAPAEdit()"><i class="fas fa-save"></i> Save</button>
<button class="btn btn-danger" onclick="deleteAPAFromModal()"><i class="fas fa-trash"></i> Delete</button>
<button class="btn btn-secondary" onclick="closeAPAEdit()"><i class="fas fa-times"></i> Cancel</button>
</div>
</div>
</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
function toggleSidebar() {
document.getElementById('sidebar').classList.toggle('open');
}

let map = null;
let markers = [];
let selectionMarker = null;
let currentEditIndex = -1;

function addAPA() {
const lat = document.getElementById('lat').value;
const lng = document.getElementById('lng').value;
const reason = document.getElementById('reason').value;

if (!lat || !lng || !reason) {
Swal.fire({icon:'warning',title:'Missing Fields',text:'Please fill in all fields',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'});
return;
}

Swal.fire({title:'Adding APA...',text:'Please wait',allowOutsideClick:false,didOpen:()=>{Swal.showLoading();}});

fetch('/addapa', {
method: 'POST',
headers: {'Content-Type': 'application/x-www-form-urlencoded'},
body: 'lat=' + encodeURIComponent(lat) + '&lng=' + encodeURIComponent(lng) + '&reason=' + encodeURIComponent(reason)
})
.then(r => r.json())
.then(d => {
if (d.success) {
Swal.fire({icon:'success',title:'APA Added!',text:'Accident prone area added',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'}).then(()=>location.reload());
} else {
Swal.fire({icon:'error',title:'Error',text:d.message||'Failed',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'});
}
})
.catch(e=>Swal.fire({icon:'error',title:'Error',text:e.message,background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'}));
}

function openAPAEdit(index, lat, lng, reason, timestamp) {
currentEditIndex = index;
document.getElementById('editApaIndex').value = index;
document.getElementById('editCoords').textContent = lat + ', ' + lng;
document.getElementById('editTimestamp').textContent = timestamp || 'Unknown';
document.getElementById('editReason').value = reason;
document.getElementById('apaEditModal').style.display = 'flex';
}

function closeAPAEdit() {
document.getElementById('apaEditModal').style.display = 'none';
currentEditIndex = -1;
}

function saveAPAEdit() {
const index = parseInt(document.getElementById('editApaIndex').value);
const reason = document.getElementById('editReason').value;
if (reason.trim() === '') {
Swal.fire({icon:'warning',title:'Empty Reason',text:'Please enter a reason',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'});
return;
}
fetch('/updateapa/' + index, {
method: 'POST',
headers: {'Content-Type': 'application/x-www-form-urlencoded'},
body: 'reason=' + encodeURIComponent(reason)
})
.then(r => r.json())
.then(d => {
if (d.success) {
closeAPAEdit();
Swal.fire({icon:'success',title:'APA Updated!',text:'Saved',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'}).then(()=>location.reload());
} else {
Swal.fire({icon:'error',title:'Error',text:d.message||'Failed',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'});
}
})
.catch(e=>Swal.fire({icon:'error',title:'Error',text:e.message,background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'}));
}

function deleteAPAFromModal() {
const index = parseInt(document.getElementById('editApaIndex').value);
Swal.fire({
title:'Are you sure?',text:'This action cannot be undone!',icon:'warning',
background:'#1e2a4a',color:'#eee',showCancelButton:true,
confirmButtonColor:'#e94560',cancelButtonColor:'#555',
confirmButtonText:'Yes, delete it!',cancelButtonText:'Cancel'
}).then((result) => {
if (result.isConfirmed) {
fetch('/removeapa/' + index, {method: 'POST'})
.then(r => r.json())
.then(d => {
if (d.success) {
closeAPAEdit();
Swal.fire({icon:'success',title:'Deleted!',text:'Removed',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'}).then(()=>location.reload());
} else {
Swal.fire({icon:'error',title:'Error',text:d.message||'Failed',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'});
}
})
.catch(e=>Swal.fire({icon:'error',title:'Error',text:e.message,background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'}));
}
});
}

function loadAPA() {
fetch('/api/apa')
.then(r=>r.json())
.then(data=> {
const list = document.getElementById('apaList');
document.getElementById('apaCount').textContent = data.length + ' locations';
if (data.length === 0) {
list.innerHTML = '<p style="color:#888;">No accident prone areas added yet.</p>';
return;
}
let html = '';
data.forEach((apa, index) => {
html += `
<div class="apa-item">
<span class="apa-reason"><i class="fas fa-exclamation-triangle" style="color:#9C27B0;"></i> ${apa.reason}</span>
<span class="apa-coords">${apa.lat.toFixed(6)}, ${apa.lng.toFixed(6)}</span>
<span class="apa-remove" onclick="removeAPA(${index})"><i class="fas fa-trash-alt"></i></span>
</div>
`;
});
list.innerHTML = html;
})
.catch(e=>console.error(e));
}

function removeAPA(index) {
Swal.fire({
title:'Are you sure?',text:'This action cannot be undone!',icon:'warning',
background:'#1e2a4a',color:'#eee',showCancelButton:true,
confirmButtonColor:'#e94560',cancelButtonColor:'#555',
confirmButtonText:'Yes, delete it!',cancelButtonText:'Cancel'
}).then((result) => {
if (result.isConfirmed) {
fetch('/removeapa/' + index, {method: 'POST'})
.then(r => r.json())
.then(d => {
if (d.success) {
Swal.fire({icon:'success',title:'Deleted!',text:'Removed',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'}).then(()=>location.reload());
} else {
Swal.fire({icon:'error',title:'Error',text:d.message||'Failed',background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'});
}
})
.catch(e=>Swal.fire({icon:'error',title:'Error',text:e.message,background:'#1e2a4a',color:'#eee',confirmButtonColor:'#e94560'}));
}
});
}

function loadMapMarkers() {
fetch('/api/apa')
.then(r=>r.json())
.then(data=> {
data.forEach((apa, index) => {
let marker = L.circleMarker([apa.lat, apa.lng], {
radius: 9, fillColor: '#ef4444', color: '#fff', weight: 2, opacity: 1, fillOpacity: 0.85
}).addTo(map);
marker.bindPopup(`
<b><i class="fas fa-exclamation-triangle" style="color:#ef4444;"></i> Accident Prone Area</b><br>
<i class="fas fa-map-pin"></i> ${apa.lat.toFixed(6)}, ${apa.lng.toFixed(6)}<br>
<i class="fas fa-comment"></i> ${apa.reason}<br>
<i class="fas fa-clock"></i> ${apa.timestamp}<br>
<div style="display:flex;gap:5px;margin-top:8px;">
<button class="btn btn-sm btn-warning" onclick="openAPAEdit(${index}, '${apa.lat.toFixed(6)}', '${apa.lng.toFixed(6)}', '${apa.reason.replace(/'/g, "\\'")}', '${apa.timestamp}')" style="padding:4px 12px;font-size:12px;flex:1;"><i class="fas fa-edit"></i> Edit</button>
<button class="btn btn-sm btn-danger" onclick="removeAPA(${index})" style="padding:4px 12px;font-size:12px;flex:1;"><i class="fas fa-trash"></i> Delete</button>
</div>
`);
markers.push(marker);
});
})
.catch(e=>console.error(e));
}

function initMap() {
document.getElementById('loadingOverlay').style.display = 'none';

map = L.map('map').setView([11.2200, 123.7600], 12);

L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Dark_Gray_Base/MapServer/tile/{z}/{y}/{x}', {
attribution: '&copy; Esri, HERE, Garmin, &copy; OpenStreetMap contributors',
maxZoom: 16
}).addTo(map);

map.on('click', function(e) {
const lat = e.latlng.lat.toFixed(6);
const lng = e.latlng.lng.toFixed(6);
document.getElementById('lat').value = lat;
document.getElementById('lng').value = lng;
if (selectionMarker) map.removeLayer(selectionMarker);
selectionMarker = L.circleMarker([lat, lng], {
radius: 8, color: '#22d3ee', fillColor: '#0ea5e9', fillOpacity: 0.9, weight: 2
}).addTo(map);
selectionMarker.bindPopup(`
<b><i class="fas fa-map-pin" style="color:#0ea5e9;"></i> New APA Location</b><br>
Lat: ${lat}<br>Lng: ${lng}<br>Click "Add APA" to save
`).openPopup();
});

loadMapMarkers();
}

document.addEventListener('DOMContentLoaded', function() {
initMap();
loadAPA();
});

document.addEventListener('click', function(e) {
const sidebar = document.getElementById('sidebar');
const toggle = document.querySelector('.menu-toggle');
if (window.innerWidth <= 768 && sidebar.classList.contains('open') &&
!sidebar.contains(e.target) && !toggle.contains(e.target)) {
sidebar.classList.remove('open');
}
});
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ===================== WIFI PAGE =====================
void handleWiFiPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>BRAKESAFE - WiFi Settings</title>
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 512 512'><path fill='%23e94560' d='M135.2 117.4L109.1 192H402.9l-26.1-74.6C372.3 104.6 360.2 96 346.6 96H165.4c-13.6 0-25.7 8.6-30.2 21.4zM39.6 196.8L74.8 96.3C88.3 57.8 124.6 32 165.4 32H346.6c40.8 0 77.1 25.8 90.6 64.3l35.2 100.5c23.2 9.6 39.6 32.5 39.6 59.2V400v48c0 17.7-14.3 32-32 32H448c-17.7 0-32-14.3-32-32V400H96v48c0 17.7-14.3 32-32 32H32c-17.7 0-32-14.3-32-32V400 256c0-26.7 16.4-49.6 39.6-59.2zM128 288a32 32 0 1 0 -64 0 32 32 0 1 0 64 0zm288 32a32 32 0 1 0 0-64 32 32 0 1 0 0 64z'/></svg>">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
<style>
*{margin:0;padding:0;box-sizing:border-box;}
:root{--primary:#e94560;--secondary:#16213e;--bg:#1a1a2e;--card-bg:#1e2a4a;--text:#eee;}
body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:var(--bg);color:var(--text);display:flex;flex-wrap:wrap;min-height:100vh;}
.sidebar{width:260px;background:var(--secondary);min-height:100vh;padding:20px;position:fixed;left:0;top:0;overflow-y:auto;border-right:2px solid #2a2a4e;z-index:1000;transition:transform 0.3s ease;}
.sidebar-brand{text-align:center;padding-bottom:20px;border-bottom:2px solid #2a2a4e;margin-bottom:20px;}
.sidebar-brand i{font-size:2em;color:var(--primary);}
.sidebar-brand h2{color:#fff;font-size:1.3em;margin-top:5px;}
.nav-item{display:flex;align-items:center;padding:12px 15px;margin:5px 0;border-radius:8px;cursor:pointer;transition:all 0.3s;color:#aaa;text-decoration:none;}
.nav-item i{width:25px;font-size:1.1em;margin-right:10px;}
.nav-item:hover{background:#2a2a4e;color:#fff;}
.nav-item.active{background:var(--primary);color:#fff;}
.sidebar-footer{font-size:0.8em;color:#666;text-align:center;margin-top:30px;padding-top:20px;border-top:2px solid #2a2a4e;}
.main-content{margin-left:260px;padding:20px;flex:1;min-width:300px;width:calc(100% - 260px);}
.container{max-width:800px;margin:0 auto;}
.page-header{margin-bottom:25px;}
.page-header h1{color:#fff;font-size:1.8em;display:flex;align-items:center;gap:10px;}
.page-header h1 i{color:var(--primary);}
.card{background:var(--card-bg);border-radius:12px;padding:20px;margin-bottom:20px;box-shadow:0 4px 6px rgba(0,0,0,0.3);}
.card-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px;padding-bottom:10px;border-bottom:1px solid #2a2a4e;}
.card-header h3{color:#fff;font-size:1.1em;display:flex;align-items:center;gap:10px;}
.card-header h3 i{color:var(--primary);}
.status-ok{color:#4CAF50;}
.status-fail{color:#F44336;}
.status-warning{color:#FF9800;}
input{width:100%;padding:10px;margin:5px 0 15px 0;border-radius:6px;border:1px solid #333;background:#2a2a4e;color:#fff;font-size:14px;}
input:focus{outline:2px solid var(--primary);border-color:transparent;}
.btn{display:inline-flex;align-items:center;gap:8px;padding:10px 20px;border:none;border-radius:6px;cursor:pointer;font-size:14px;font-weight:600;transition:all 0.3s;text-decoration:none;}
.btn-primary{background:var(--primary);color:#fff;}
.btn-primary:hover{background:#c73652;transform:translateY(-2px);}
.btn-success{background:#4CAF50;color:#fff;}
.btn-success:hover{background:#43a047;transform:translateY(-2px);}
.menu-toggle{display:none;background:var(--primary);color:#fff;border:none;padding:10px 15px;border-radius:6px;font-size:1.2em;cursor:pointer;margin-bottom:15px;}
table{width:100%;border-collapse:collapse;}
td{padding:8px;border-bottom:1px solid #2a2a4e;}
td:first-child{font-weight:bold;color:#aaa;width:40%;}
td:last-child{color:#fff;}
@media(max-width:768px){
.sidebar{transform:translateX(-100%);position:fixed;width:280px;z-index:1001;}
.sidebar.open{transform:translateX(0);}
.main-content{margin-left:0;width:100%;}
.menu-toggle{display:block !important;z-index:1002;position:relative;}
}
.footer{text-align:center;padding:20px 0;color:#666;font-size:0.85em;border-top:1px solid #2a2a4e;margin-top:20px;}
.net-item{padding:10px 12px;background:#2a2a4e;margin:4px 0;border-radius:6px;cursor:pointer;display:flex;justify-content:space-between;align-items:center;gap:10px;transition:background 0.2s;}
.net-item:hover{background:#3a3a5e;}
.net-item .ssid{color:#fff;flex:1;word-break:break-all;}
.net-item .rssi{color:#888;font-size:0.85em;white-space:nowrap;}
</style>
</head>
<body>
<button class="menu-toggle" onclick="toggleSidebar()"><i class="fas fa-bars"></i> Menu</button>

<div class="sidebar" id="sidebar">
<div class="sidebar-brand">
<i class="fas fa-car-crash"></i>
<h2>BRAKESAFE</h2>
</div>
<a class="nav-item" href="/"><i class="fas fa-tachometer-alt"></i> Dashboard</a>
<a class="nav-item" href="/map"><i class="fas fa-map-marked-alt"></i> Bantayan Map</a>
<a class="nav-item" href="/apa"><i class="fas fa-exclamation-triangle"></i> APA Map</a>
<a class="nav-item active" href="/wifi"><i class="fas fa-wifi"></i> WiFi Settings</a>
<a class="nav-item" href="/system"><i class="fas fa-cog"></i> System Info</a>
<div class="sidebar-footer">
<i class="fas fa-microchip"></i> v1
</div>
</div>

<div class="main-content">
<div class="container">
<div class="page-header">
<h1><i class="fas fa-wifi"></i> WiFi Settings</h1>
</div>

)rawliteral";
  html += getWiFiStatusHTML();

  html += R"rawliteral(
<div class="card">
<div class="card-header">
<h3><i class="fas fa-search"></i> Available Networks</h3>
</div>
<button class="btn btn-primary" onclick="scanNetworks()"><i class="fas fa-sync-alt"></i> Scan Networks</button>
<div id="networkList" style="margin-top:15px;"></div>
</div>

<div class="footer">
<i class="fas fa-copyright"></i> 2026 BRAKESAFE System
</div>
</div>
</div>

<script>
function toggleSidebar() {
document.getElementById('sidebar').classList.toggle('open');
}

function scanNetworks() {
const list = document.getElementById('networkList');
list.innerHTML = '<p style="color:#888;"><i class="fas fa-spinner fa-spin"></i> Scanning...</p>';
let attempts = 0;
const maxAttempts = 40;

function poll() {
attempts++;
fetch('/scanwifi', {cache:'no-store'})
.then(r=>r.json())
.then(data=>{
if (data.status === 'scanning' && attempts < maxAttempts) {
setTimeout(poll, 500);
return;
}
if (data.networks && data.networks.length > 0) {
let html = '';
data.networks.forEach(net => {
const esc = net.ssid.replace(/\\/g,'\\\\').replace(/'/g, "\\'");
const lock = net.enc ? '<i class="fas fa-lock" style="color:#FF9800;margin-right:6px;"></i>'
                     : '<i class="fas fa-lock-open" style="color:#4CAF50;margin-right:6px;"></i>';
html += '<div class="net-item" onclick="document.querySelector(\'input[name=ssid]\').value=\''+esc+'\';window.scrollTo({top:0,behavior:\'smooth\'});">'
     +   '<span class="ssid">'+lock+net.ssid+'</span>'
     +   '<span class="rssi">'+net.rssi+' dBm</span>'
     + '</div>';
});
list.innerHTML = html;
} else if (data.status === 'scanning') {
list.innerHTML = '<p style="color:#888;">Scan timed out. Try again.</p>';
} else {
list.innerHTML = '<p style="color:#888;">No networks found</p>';
}
})
.catch(e=>{
list.innerHTML = '<p class="status-fail"><i class="fas fa-exclamation-circle"></i> Failed to scan</p>';
});
}
poll();
}

document.addEventListener('click', function(e) {
const sidebar = document.getElementById('sidebar');
const toggle = document.querySelector('.menu-toggle');
if (window.innerWidth <= 768 && sidebar.classList.contains('open') &&
!sidebar.contains(e.target) && !toggle.contains(e.target)) {
sidebar.classList.remove('open');
}
});
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ===================== SYSTEM PAGE =====================
void handleSystemPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>BRAKESAFE - System Info</title>
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 512 512'><path fill='%23e94560' d='M135.2 117.4L109.1 192H402.9l-26.1-74.6C372.3 104.6 360.2 96 346.6 96H165.4c-13.6 0-25.7 8.6-30.2 21.4zM39.6 196.8L74.8 96.3C88.3 57.8 124.6 32 165.4 32H346.6c40.8 0 77.1 25.8 90.6 64.3l35.2 100.5c23.2 9.6 39.6 32.5 39.6 59.2V400v48c0 17.7-14.3 32-32 32H448c-17.7 0-32-14.3-32-32V400H96v48c0 17.7-14.3 32-32 32H32c-17.7 0-32-14.3-32-32V400 256c0-26.7 16.4-49.6 39.6-59.2zM128 288a32 32 0 1 0 -64 0 32 32 0 1 0 64 0zm288 32a32 32 0 1 0 0-64 32 32 0 1 0 0 64z'/></svg>">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
<style>
*{margin:0;padding:0;box-sizing:border-box;}
:root{--primary:#e94560;--secondary:#16213e;--bg:#1a1a2e;--card-bg:#1e2a4a;--text:#eee;}
body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:var(--bg);color:var(--text);display:flex;flex-wrap:wrap;min-height:100vh;}
.sidebar{width:260px;background:var(--secondary);min-height:100vh;padding:20px;position:fixed;left:0;top:0;overflow-y:auto;border-right:2px solid #2a2a4e;z-index:1000;transition:transform 0.3s ease;}
.sidebar-brand{text-align:center;padding-bottom:20px;border-bottom:2px solid #2a2a4e;margin-bottom:20px;}
.sidebar-brand i{font-size:2em;color:var(--primary);}
.sidebar-brand h2{color:#fff;font-size:1.3em;margin-top:5px;}
.nav-item{display:flex;align-items:center;padding:12px 15px;margin:5px 0;border-radius:8px;cursor:pointer;transition:all 0.3s;color:#aaa;text-decoration:none;}
.nav-item i{width:25px;font-size:1.1em;margin-right:10px;}
.nav-item:hover{background:#2a2a4e;color:#fff;}
.nav-item.active{background:var(--primary);color:#fff;}
.sidebar-footer{font-size:0.8em;color:#666;text-align:center;margin-top:30px;padding-top:20px;border-top:2px solid #2a2a4e;}
.main-content{margin-left:260px;padding:20px;flex:1;min-width:300px;width:calc(100% - 260px);}
.container{max-width:800px;margin:0 auto;}
.page-header{margin-bottom:25px;}
.page-header h1{color:#fff;font-size:1.8em;display:flex;align-items:center;gap:10px;}
.page-header h1 i{color:var(--primary);}
.card{background:var(--card-bg);border-radius:12px;padding:20px;margin-bottom:20px;box-shadow:0 4px 6px rgba(0,0,0,0.3);}
.card-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px;padding-bottom:10px;border-bottom:1px solid #2a2a4e;}
.card-header h3{color:#fff;font-size:1.1em;display:flex;align-items:center;gap:10px;}
.card-header h3 i{color:var(--primary);}
.status-ok{color:#4CAF50;}
.status-fail{color:#F44336;}
.status-warning{color:#FF9800;}
table{width:100%;border-collapse:collapse;}
td{padding:10px;border-bottom:1px solid #2a2a4e;}
td:first-child{font-weight:bold;color:#aaa;width:40%;}
td:last-child{color:#fff;}
.btn{display:inline-flex;align-items:center;gap:8px;padding:10px 20px;border:none;border-radius:6px;cursor:pointer;font-size:14px;font-weight:600;transition:all 0.3s;text-decoration:none;}
.btn-primary{background:var(--primary);color:#fff;}
.btn-primary:hover{background:#c73652;transform:translateY(-2px);}
.btn-danger{background:#F44336;color:#fff;}
.btn-danger:hover{background:#E53935;transform:translateY(-2px);}
.btn-warning{background:#FF9800;color:#fff;}
.btn-warning:hover{background:#F57C00;transform:translateY(-2px);}
.btn-group{display:flex;gap:10px;flex-wrap:wrap;margin:10px 0;}
.menu-toggle{display:none;background:var(--primary);color:#fff;border:none;padding:10px 15px;border-radius:6px;font-size:1.2em;cursor:pointer;margin-bottom:15px;}
@media(max-width:768px){
.sidebar{transform:translateX(-100%);position:fixed;width:280px;z-index:1001;}
.sidebar.open{transform:translateX(0);}
.main-content{margin-left:0;width:100%;}
.menu-toggle{display:block !important;z-index:1002;position:relative;}
}
.footer{text-align:center;padding:20px 0;color:#666;font-size:0.85em;border-top:1px solid #2a2a4e;margin-top:20px;}
</style>
</head>
<body>
<button class="menu-toggle" onclick="toggleSidebar()"><i class="fas fa-bars"></i> Menu</button>

<div class="sidebar" id="sidebar">
<div class="sidebar-brand">
<i class="fas fa-car-crash"></i>
<h2>BRAKESAFE</h2>
</div>
<a class="nav-item" href="/"><i class="fas fa-tachometer-alt"></i> Dashboard</a>
<a class="nav-item" href="/map"><i class="fas fa-map-marked-alt"></i> Bantayan Map</a>
<a class="nav-item" href="/apa"><i class="fas fa-exclamation-triangle"></i> APA Map</a>
<a class="nav-item" href="/wifi"><i class="fas fa-wifi"></i> WiFi Settings</a>
<a class="nav-item active" href="/system"><i class="fas fa-cog"></i> System Info</a>
<div class="sidebar-footer">
<i class="fas fa-microchip"></i> v1
</div>
</div>

<div class="main-content">
<div class="container">
<div class="page-header">
<h1><i class="fas fa-cog"></i> System Information</h1>
</div>

<div class="card">
<div class="card-header">
<h3><i class="fas fa-info-circle"></i> Device Details</h3>
</div>
<table>
<tr><td><i class="fas fa-code-branch"></i> Firmware Version</td><td>v1</td></tr>
<tr><td><i class="fas fa-fingerprint"></i> Chip ID</td><td>)rawliteral"
                + String((uint32_t)ESP.getEfuseMac()) + R"rawliteral(</td></tr>
<tr><td><i class="fas fa-memory"></i> Flash Size</td><td>)rawliteral"
                + String(ESP.getFlashChipSize() / (1024 * 1024)) + R"rawliteral( MB</td></tr>
<tr><td><i class="fas fa-ram"></i> Free Heap</td><td>)rawliteral"
                + String(ESP.getFreeHeap() / 1024) + R"rawliteral( KB</td></tr>
<tr><td><i class="fas fa-clock"></i> Uptime</td><td id="uptime">--</td></tr>
<tr><td><i class="fas fa-wifi"></i> WiFi Mode</td><td id="wifiMode">--</td></tr>
<tr><td><i class="fas fa-network-wired"></i> Saved SSID</td><td>)rawliteral"
                + (wifiSSID.length() > 0 ? wifiSSID : String("(none)")) + R"rawliteral(</td></tr>
<tr><td><i class="fas fa-ip"></i> IP Address</td><td id="ipAddress">--</td></tr>
<tr><td><i class="fas fa-ethernet"></i> MAC Address</td><td>)rawliteral"
                + WiFi.macAddress() + R"rawliteral(</td></tr>
<tr><td><i class="fas fa-database"></i> Events Recorded</td><td>)rawliteral"
                + String(historyCount) + R"rawliteral(</td></tr>
<tr><td><i class="fas fa-satellite-dish"></i> GPS Module</td><td id="gpsModule">--</td></tr>
<tr><td><i class="fas fa-microchip"></i> MPU6050</td><td id="mpuStatus">--</td></tr>
<tr><td><i class="fas fa-compass"></i> Heading</td><td id="heading">--</td></tr>
</table>
</div>

<div class="card">
<div class="card-header">
<h3><i class="fas fa-tools"></i> Actions</h3>
</div>
<div class="btn-group">
<button class="btn btn-warning" onclick="if(confirm('Restart device?')){window.location.href='/restart'}"><i class="fas fa-sync-alt"></i> Restart Device</button>
<button class="btn btn-danger" onclick="if(confirm('Reset WiFi settings?')){window.location.href='/resetwifi'}"><i class="fas fa-trash-alt"></i> Reset WiFi</button>
</div>
</div>

<div class="card">
<div class="card-header">
<h3><i class="fas fa-broom"></i> Clear Data</h3>
</div>
<div class="btn-group">
<button class="btn btn-danger" onclick="clearEvents()"><i class="fas fa-trash"></i> Clear All Brake Events</button>
</div>
</div>

<div class="footer">
<i class="fas fa-copyright"></i> 2026 BRAKESAFE System
</div>
</div>
</div>

<script>
function toggleSidebar() {
document.getElementById('sidebar').classList.toggle('open');
}

function clearEvents() {
if (!confirm('Delete ALL brake events? This cannot be undone!')) return;
fetch('/clear/events', {method:'POST'})
.then(r=>r.json())
.then(d=>{
if (d.success) {
alert('All brake events cleared.');
location.reload();
} else {
alert('Failed: ' + (d.message || 'unknown error'));
}
})
.catch(e=>alert('Error: ' + e.message));
}

const bootTime = Date.now();
function updateSystem() {
fetch('/api/status', {cache:'no-store'})
.then(r=>r.json())
.then(d=>{
const uptime = Math.floor((Date.now()-bootTime)/1000);
const hours = Math.floor(uptime / 3600);
const minutes = Math.floor((uptime % 3600) / 60);
const seconds = uptime % 60;
document.getElementById('uptime').textContent = hours + 'h ' + minutes + 'm ' + seconds + 's';
document.getElementById('wifiMode').textContent = d.wifi_connected ? 'Station' : (d.ap_mode ? 'Access Point (retrying saved)' : 'Disconnected');
document.getElementById('ipAddress').textContent = d.wifi_connected ? window.location.hostname : (d.ap_mode ? '192.168.4.1' : '--');

const gEl = document.getElementById('gpsModule');
gEl.textContent = d.gps_module ? 'Detected \u2705' : 'Not detected';
gEl.className = d.gps_module ? 'status-ok' : 'status-warning';

const mEl = document.getElementById('mpuStatus');
mEl.textContent = d.mpu_ready ? 'Active \u2705' : 'Disabled';
mEl.className = d.mpu_ready ? 'status-ok' : 'status-warning';

const hEl = document.getElementById('heading');
hEl.textContent = d.direction + ' (' + d.heading.toFixed(1) + '\u00B0)';
})
.catch(e=>console.error(e));
}
updateSystem();
setInterval(updateSystem, 5000);

document.addEventListener('click', function(e) {
const sidebar = document.getElementById('sidebar');
const toggle = document.querySelector('.menu-toggle');
if (window.innerWidth <= 768 && sidebar.classList.contains('open') &&
!sidebar.contains(e.target) && !toggle.contains(e.target)) {
sidebar.classList.remove('open');
}
});
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ===================== API Handlers =====================
void handleSetWiFi() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");

  if (ssid.length() == 0) {
    server.send(400, "application/json", "{\"success\":false,\"message\":\"SSID required\"}");
    return;
  }

  bool saved = saveWiFiCredentials(ssid, password);
  if (!saved) {
    server.send(500, "application/json", "{\"success\":false,\"message\":\"Failed to save credentials\"}");
    return;
  }

  bool ok = connectToWiFi();
  if (ok) {
    if (apModeActive) {
      apModeActive = false;
      dnsServer.stop();
    }
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Saved and connected!\"}");
  } else {
    if (!apModeActive) startAccessPoint();
    server.send(200, "application/json",
      "{\"success\":true,\"message\":\"Credentials saved. Could not connect right now — will retry automatically.\"}");
  }
}

void handleSetAPPassword() {
  String password = server.arg("ap_password");
  saveAPPassword(password);
  startAccessPoint();
  server.send(200, "application/json", "{\"success\":true,\"message\":\"AP Password Updated!\"}");
}

void handleScanWiFi() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_FAILED) {
    WiFi.scanNetworks(true, false);
    server.send(200, "application/json", "{\"status\":\"scanning\",\"networks\":[]}");
    return;
  }
  if (n == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "{\"status\":\"scanning\",\"networks\":[]}");
    return;
  }
  String json = "{\"status\":\"done\",\"networks\":[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    String ssid = WiFi.SSID(i);
    ssid.replace("\\", "\\\\");
    ssid.replace("\"", "\\\"");
    json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + ",\"enc\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? 0 : 1) + "}";
  }
  json += "]}";
  WiFi.scanDelete();
  server.send(200, "application/json", json);
}

void handleGetEvents() {
  if (!LittleFS.exists(EVENTS_FILE)) {
    server.send(200, "application/json", "[]");
    return;
  }

  int limit = 0;
  if (server.hasArg("limit")) {
    limit = server.arg("limit").toInt();
  }
  int skip = (limit > 0 && historyCount > limit) ? (historyCount - limit) : 0;

  File f = LittleFS.open(EVENTS_FILE, "r");
  if (!f) {
    server.send(500, "application/json", "{\"error\":\"file open failed\"}");
    return;
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("[");

  int idx = 0;
  bool first = true;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (idx++ < skip) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);
    int c5 = line.indexOf(',', c4 + 1);
    int c6 = line.indexOf(',', c5 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0 || c5 < 0 || c6 < 0) continue;

    double lat   = line.substring(0, c1).toFloat();
    double lng   = line.substring(c1 + 1, c2).toFloat();

    // Reject (0,0) rows
    if (fabs(lat) < 0.000001 && fabs(lng) < 0.000001) continue;

    float  speed = line.substring(c2 + 1, c3).toFloat();
    float  decel = line.substring(c3 + 1, c4).toFloat();
    int    lvl   = line.substring(c4 + 1, c5).toInt();
    String ts    = line.substring(c5 + 1, c6);
    String dir   = line.substring(c6 + 1);

    String obj = first ? "" : ",";
    first = false;
    obj += "{\"lat\":" + String(lat, 6);
    obj += ",\"lng\":" + String(lng, 6);
    obj += ",\"speed\":" + String(speed, 2);
    obj += ",\"decel\":" + String(decel, 2);
    obj += ",\"level\":\"" + getBrakeLevelString((BrakeLevel)lvl) + "\"";
    obj += ",\"timestamp\":\"" + ts + "\"";
    obj += ",\"direction\":\"" + dir + "\"}";
    server.sendContent(obj);

    delay(0);
  }

  server.sendContent("]");
  server.sendContent("");
  f.close();
}

void handleGetAPA() {
  String json = "[";
  for (size_t i = 0; i < accidentProneAreas.size(); i++) {
    if (i > 0) json += ",";
    String reason = accidentProneAreas[i].reason;
    reason.replace("\\", "\\\\");
    reason.replace("\"", "\\\"");
    json += "{\"lat\":" + String(accidentProneAreas[i].lat, 6) + ",\"lng\":" + String(accidentProneAreas[i].lng, 6) + ",\"reason\":\"" + reason + "\",\"timestamp\":\"" + accidentProneAreas[i].timestamp + "\"}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleAddAPA() {
  String latStr = server.arg("lat");
  String lngStr = server.arg("lng");
  String reason = server.arg("reason");

  if (latStr.length() > 0 && lngStr.length() > 0 && reason.length() > 0) {
    APA newAPA;
    newAPA.lat = latStr.toFloat();
    newAPA.lng = lngStr.toFloat();
    newAPA.reason = reason;
    newAPA.timestamp = formatTimestamp();
    newAPA.barangay = getBarangay(newAPA.lat, newAPA.lng);
    newAPA.municipality = getMunicipality(newAPA.lat, newAPA.lng);
    newAPA.province = getProvince(newAPA.lat, newAPA.lng);
    newAPA.direction = getCurrentDirection();
    newAPA.autoAdded = false;
    accidentProneAreas.push_back(newAPA);
    persistAPA();
    server.send(200, "application/json", "{\"success\":true,\"message\":\"APA Added!\"}");
  } else {
    server.send(400, "application/json", "{\"success\":false,\"message\":\"All fields required\"}");
  }
}

void handleRemoveAPA() {
  String indexStr = server.pathArg(0);
  int index = indexStr.toInt();
  Serial.printf("[APA] Delete request - URI: %s, Index: %d\n", server.uri().c_str(), index);

  if (index >= 0 && index < (int)accidentProneAreas.size()) {
    accidentProneAreas.erase(accidentProneAreas.begin() + index);
    persistAPA();
    server.send(200, "application/json", "{\"success\":true,\"message\":\"APA Deleted!\"}");
  } else {
    server.send(404, "application/json",
                "{\"success\":false,\"message\":\"APA not found at index " + String(index) + "\"}");
  }
}

void handleUpdateAPA() {
  String indexStr = server.pathArg(0);
  int index = indexStr.toInt();
  String reason = server.arg("reason");
  Serial.printf("[APA] Update request - URI: %s, Index: %d, Reason: %s\n",
                server.uri().c_str(), index, reason.c_str());

  if (index >= 0 && index < (int)accidentProneAreas.size() && reason.length() > 0) {
    accidentProneAreas[index].reason = reason;
    persistAPA();
    server.send(200, "application/json", "{\"success\":true,\"message\":\"APA Updated!\"}");
  } else {
    server.send(400, "application/json",
                "{\"success\":false,\"message\":\"Invalid request - index: " + String(index) + "\"}");
  }
}

void handleClearEvents() {
  if (LittleFS.exists(EVENTS_FILE)) {
    LittleFS.remove(EVENTS_FILE);
  }
  historyCount = 0;
  recentCount = 0;
  lastEventTimeStr = "None";
  Serial.println("[Events] All brake events cleared (file deleted)");
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Events cleared\"}");
}

void handleDownloadEvents() {
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=brake_events.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("Latitude,Longitude,Speed(km/h),Deceleration(g),Brake Level,Timestamp,Direction\n");

  if (LittleFS.exists(EVENTS_FILE)) {
    File f = LittleFS.open(EVENTS_FILE, "r");
    if (f) {
      while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        int c3 = line.indexOf(',', c2 + 1);
        int c4 = line.indexOf(',', c3 + 1);
        int c5 = line.indexOf(',', c4 + 1);
        if (c5 < 0) continue;

        // Skip 0,0 rows
        float lat0 = line.substring(0, c1).toFloat();
        float lng0 = line.substring(c1 + 1, c2).toFloat();
        if (fabs(lat0) < 0.000001 && fabs(lng0) < 0.000001) continue;

        int lvl = line.substring(c4 + 1, c5).toInt();
        String out = line.substring(0, c4 + 1)
                   + getBrakeLevelString((BrakeLevel)lvl)
                   + line.substring(c5);
        server.sendContent(out + "\n");
        delay(0);
      }
      f.close();
    }
  }
  server.sendContent("");
}

void handleDownloadAPA() {
  String csv = "Latitude,Longitude,Reason,Timestamp\n";
  for (size_t i = 0; i < accidentProneAreas.size(); i++) {
    csv += String(accidentProneAreas[i].lat, 6) + "," + String(accidentProneAreas[i].lng, 6) + ",";
    csv += accidentProneAreas[i].reason + ",";
    csv += accidentProneAreas[i].timestamp + "\n";
  }
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=apa_data_" + String(millis()) + ".csv");
  server.send(200, "text/csv", csv);
}

void handleRestart() {
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Restarting...\"}");
  delay(500);
  ESP.restart();
}

void handleResetWiFi() {
  saveWiFiCredentials("", "");
  saveAPPassword("");
  server.send(200, "application/json", "{\"success\":true,\"message\":\"WiFi Reset\"}");
  delay(500);
  ESP.restart();
}

void handleStatus() {
  server.send(200, "application/json", getStatusJSON());
}

void loadAPAFromPreferences() {
  preferences.begin("apa", true);
  String apaData = preferences.getString("apa_data", "");
  preferences.end();

  if (apaData.length() > 0) {
    int start = 0;
    int end = apaData.indexOf(';');
    while (end != -1) {
      String entry = apaData.substring(start, end);
      int comma1 = entry.indexOf(',');
      int comma2 = entry.indexOf(',', comma1 + 1);
      int comma3 = entry.indexOf(',', comma2 + 1);
      int comma4 = entry.indexOf(',', comma3 + 1);
      int comma5 = entry.indexOf(',', comma4 + 1);
      int comma6 = entry.indexOf(',', comma5 + 1);
      int comma7 = entry.indexOf(',', comma6 + 1);
      int comma8 = entry.indexOf(',', comma7 + 1);

      if (comma1 != -1 && comma2 != -1 && comma3 != -1) {
        APA apa;
        apa.lat = entry.substring(0, comma1).toFloat();
        apa.lng = entry.substring(comma1 + 1, comma2).toFloat();
        apa.reason = entry.substring(comma2 + 1, comma3);
        apa.timestamp = (comma4 != -1) ? entry.substring(comma3 + 1, comma4) : "";
        apa.barangay = (comma4 != -1) ? entry.substring(comma4 + 1, comma5) : "Unknown";
        apa.municipality = (comma5 != -1) ? entry.substring(comma5 + 1, comma6) : "Unknown";
        apa.province = (comma6 != -1) ? entry.substring(comma6 + 1, comma7) : "Cebu";
        apa.direction = (comma7 != -1) ? entry.substring(comma7 + 1, comma8) : "N";
        apa.autoAdded = (comma8 != -1) ? (entry.substring(comma8 + 1) == "1") : false;
        accidentProneAreas.push_back(apa);
      }

      start = end + 1;
      end = apaData.indexOf(';', start);
    }
    Serial.printf("[APA] Loaded %d accident prone areas\n", accidentProneAreas.size());
  }
}

void startWebServer() {
  server.on("/", handleRoot);
  server.on("/map", handleMapPage);
  server.on("/apa", handleAPAPage);
  server.on("/wifi", handleWiFiPage);
  server.on("/system", handleSystemPage);
  server.on("/api/status", handleStatus);
  server.on("/api/events", handleGetEvents);
  server.on("/api/apa", handleGetAPA);
  server.on("/setwifi", HTTP_POST, handleSetWiFi);
  server.on("/setappass", HTTP_POST, handleSetAPPassword);
  server.on("/scanwifi", handleScanWiFi);
  server.on("/addapa", HTTP_POST, handleAddAPA);
  server.on(UriBraces("/removeapa/{}"), HTTP_POST, handleRemoveAPA);
  server.on(UriBraces("/updateapa/{}"), HTTP_POST, handleUpdateAPA);
  server.on("/clear/events", HTTP_POST, handleClearEvents);
  server.on("/download/events", handleDownloadEvents);
  server.on("/download/apa", handleDownloadAPA);
  server.on("/restart", handleRestart);
  server.on("/resetwifi", handleResetWiFi);

  server.onNotFound([]() {
    server.send(404, "application/json", "{\"success\":false,\"message\":\"Not Found\"}");
  });

  server.begin();
  Serial.print("Web server started at http://");
  if (apModeActive) {
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(WiFi.localIP());
  }
}

// ===================== Main Processing =====================
void readAndProcessAccel() {
  if (!mpuInitialized) return;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long nowMs = millis();
  float dt = (nowMs - lastGyroMs) / 1000.0f;
  lastGyroMs = nowMs;
  if (dt > 0.05f) dt = 0.05f;

  float gz_dps = gz / GYRO_LSB_PER_DPS;
  currentHeadingDeg += gz_dps * dt;
  while (currentHeadingDeg >= 360.0f) currentHeadingDeg -= 360.0f;
  while (currentHeadingDeg < 0.0f)    currentHeadingDeg += 360.0f;

  float ax_g = ((float)ax / ACCEL_LSB_PER_G) - ax_offset;
  ax_g_filt = AX_ALPHA * ax_g + (1.0f - AX_ALPHA) * ax_g_filt;
  float decel_g = ax_g_filt < 0 ? (-ax_g_filt) : 0.0f;
  BrakeLevel lvl = classifyBrake(decel_g);
  applyIndicators(lvl);
  lastBrakeLevel = lvl;
  lastDecel = decel_g;

  unsigned long now = millis();
  bool shouldRecord = (lvl == BRAKE_SOFT || lvl == BRAKE_MODERATE ||
                       lvl == BRAKE_HARD || lvl == BRAKE_EMERGENCY);

  uint32_t cooldown = (lvl == BRAKE_HARD || lvl == BRAKE_EMERGENCY)
                      ? EVENT_COOLDOWN_HARD_MS
                      : EVENT_COOLDOWN_SOFT_MS;

  if (shouldRecord && (now - lastEventMs >= cooldown)) {

    // ── Reject if GPS has no fix (would otherwise land at 0,0) ──
    if (!gps.location.isValid()) {
      Serial.printf("[Event] %s brake SKIPPED — no GPS fix (would be 0,0)\n",
                    getBrakeLevelString(lvl).c_str());
      lastEventMs = now;
      return;
    }

    double evLat = gps.location.lat();
    double evLng = gps.location.lng();
    if (fabs(evLat) < 0.000001 && fabs(evLng) < 0.000001) {
      Serial.println("[Event] Skipped — coordinates are 0,0");
      lastEventMs = now;
      return;
    }

    BrakeEvent event;
    event.lat = evLat;
    event.lng = evLng;
    event.speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
    event.decel = decel_g;
    event.level = lvl;
    event.timestamp = formatTimestamp();
    event.sent = true;
    event.brakeLevelStr = getBrakeLevelString(lvl);
    event.barangay = getBarangay(event.lat, event.lng);
    event.municipality = getMunicipality(event.lat, event.lng);
    event.province = getProvince(event.lat, event.lng);
    event.direction = getCurrentDirection();

    if (lvl == BRAKE_EMERGENCY) {
      checkEmergencyBrakeAPA(event.lat, event.lng);
      autoAddAPA(event.lat, event.lng, "Emergency brake at " + event.barangay);
    }

    recordEvent(event);
    lastEventMs = now;
    lastEventTimeStr = event.timestamp;

    Serial.printf("[Event] %s brake @ %s (%.3f g) — total %d\n",
                  getBrakeLevelString(lvl).c_str(),
                  event.direction.c_str(),
                  decel_g, historyCount);
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  setLeds(false, false, false);
  buzzerOn(false);

  Serial.println("\n========================================");
  Serial.println("   BRAKESAFE SYSTEM v1");
  Serial.println("   Timezone: UTC+8 (Philippines)");
  Serial.println("   Storage : LittleFS (events, append-only)");
  Serial.println("   WiFi    : Persistent (saved on flash)");
  Serial.println("   Map     : Heatmap + typhoon track + scatter");
  Serial.println("========================================\n");

  if (!LittleFS.begin(true)) {
    Serial.println("[LittleFS] ⚠️ Mount failed — events will NOT persist!");
  } else {
    Serial.println("[LittleFS] Mounted successfully");
  }

  loadWiFiCredentials();
  loadAPAFromPreferences();
  purgeZeroZeroEvents();
  loadEventsFromFile();

  initGPS();

  bool connected = false;
  if (wifiSSID.length() > 0) {
    connected = connectToWiFi();
  }
  if (!connected) {
    startAccessPoint();
  }

  startWebServer();
  Serial.println("=== BRAKESAFE Firmware Ready ===");
}

// ===================== Loop =====================
void loop() {
  pumpGPS();

  if (gpsDataDetected && !mpuInitialized) {
    Serial.println("[MPU] GPS module detected → initializing MPU6050...");
    initMPU();
    mpuInitialized = true;
  }

  if (apaAlertActive) {
    if (millis() - apaAlertStartTime > APA_ALERT_DURATION) {
      apaAlertActive = false;
      buzzerOn(false);
    } else {
      if ((millis() - apaAlertStartTime) % 500 < 250) {
        buzzerOn(true);
      } else {
        buzzerOn(false);
      }
    }
  }

  manageWiFi();
  server.handleClient();

  if (mpuInitialized) {
    unsigned long now = millis();
    if (now - lastAccelSampleMs >= ACCEL_SAMPLE_PERIOD_MS) {
      lastAccelSampleMs = now;
      readAndProcessAccel();
    }
  }

  static unsigned long lastQueuePrint = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastQueuePrint > 5000) {
    lastQueuePrint = nowMs;
    Serial.printf("[Events] %d total on disk (last %d cached in RAM)\n", historyCount, recentCount);
    Serial.printf("[WiFi] %s | SSID: %s\n",
                  wifiConnected ? "Connected" : (apModeActive ? "AP Mode (retrying)" : "Disconnected"),
                  wifiSSID.length() > 0 ? wifiSSID.c_str() : "(none)");
    Serial.printf("[GPS] %s | Satellites: %d | Module: %s\n",
                  gps.location.isValid() ? "FIXED ✅" : "Waiting ⏳",
                  gps.satellites.value(),
                  gpsDataDetected ? "Detected" : "Not detected");
    Serial.printf("[MPU] %s | Heading: %.1f° (%s)\n",
                  mpuInitialized ? "Active" : "Disabled (waiting for GPS)",
                  currentHeadingDeg, getCurrentDirection().c_str());
    if (accidentProneAreas.size() > 0) {
      Serial.printf("[APA] %d areas monitored\n", accidentProneAreas.size());
    }
  }
}
