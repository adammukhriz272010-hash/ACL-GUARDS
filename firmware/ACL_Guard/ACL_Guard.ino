/*
  ACL-GUARD — Smart Knee Support
  Kumpulan Vanguard, SMK Darulaman Heights 1

  The ESP32 reads the knee angle (two MPU6050 or a hinge potentiometer).
  If the knee bends past the safe limit for more than WARN_DELAY_MS,
  the brace vibrates (pulsing) and the red LED blinks.

  Phone dashboard:  WiFi "ACL-GUARD"  ->  http://192.168.4.1
  Live data:        WebSocket port 81, JSON every 50 ms
                    {"a":32.5,"lim":45,"w":0,"auto":1,"err":""}
  Commands:         LIMIT:45   VIB:700   ZERO   AUTO:1 / AUTO:0

  Libraries: "esp32" board package by Espressif, "WebSockets" by Markus Sattler.

  This is a school prototype, NOT a certified medical device.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include "dashboard.h"

// ======================= SETTINGS =======================

// 1 = two MPU6050 (thigh + shin), 2 = 10k potentiometer on the hinge (GPIO 34)
#define SENSOR_MODE 1

// WiFi hotspot made by the brace. Password must be at least 8 characters.
const char* AP_SSID = "ACL-GUARD";
const char* AP_PASS = "acl12345";

// Potentiometer calibration (only for SENSOR_MODE 2).
// Read the "pot raw" values in Serial Monitor with the leg straight (0°)
// and bent to 90°, then write them here.
#define POT_AT_0   1000
#define POT_AT_90  2600

// Pins
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_MOTOR      25
#define PIN_LED_RED    26
#define PIN_LED_GREEN  27
#define PIN_POT        34

// MPU6050 addresses (AD0 -> GND = 0x68, AD0 -> 3V3 = 0x69)
#define ADDR_THIGH 0x68
#define ADDR_SHIN  0x69

// Warning behaviour
#define DEFAULT_LIMIT      45     // degrees
#define LIMIT_MIN          5
#define LIMIT_MAX          150
#define WARN_DELAY_MS      300    // must stay past the limit this long
#define HYSTERESIS_DEG     3.0f   // warning stops below (limit - this)
#define PULSE_ON_MS        250    // vibration pulse while warning
#define PULSE_OFF_MS       150
#define MANUAL_VIB_MAX_MS  5000

// Timing / filtering
#define SAMPLE_INTERVAL_MS 10     // 100 Hz sensor reading
#define SEND_INTERVAL_MS   50     // 20 Hz to the phone
#define FILTER_ALPHA       0.96f  // complementary filter (gyro weight)

// ========================================================

WebServer server(80);
WebSocketsServer ws(81);
Preferences prefs;

// Settings that are saved in flash
int   limitDeg   = DEFAULT_LIMIT;
float zeroOffset = 0;      // raw angle when the leg is straight
bool  autoVib    = true;
bool  prefsDirty = false;
uint32_t prefsChangedAt = 0;

// Live state
float kneeAngle = 0;       // degrees, 0 = straight
float rawAngle  = 0;       // before zero offset
bool  sensorOk  = false;
char  errMsg[64] = "";

bool     warning      = false;
bool     overLimit    = false;
uint32_t overSince    = 0;
uint32_t warnStart    = 0;
uint32_t manualVibEnd = 0;
bool     manualVibOn  = false;

uint32_t lastSample = 0, lastSend = 0, lastRetry = 0, lastDebug = 0;

static float wrap180(float a) {
  while (a > 180.0f)   a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// ------------------------- MPU6050 -------------------------
#if SENSOR_MODE == 1

struct Segment {
  uint8_t     addr;
  const char* name;
  const char* errText;
  bool        ok;
  bool        hasAngle;
  uint8_t     fails;
  float       angle;    // segment angle in the leg's plane (degrees)
  float       gyroBias; // deg/s
};

Segment thigh = { ADDR_THIGH, "Thigh", "Sensor paha (0x68) tidak dikesan", false, false, 0, 0, 0 };
Segment shin  = { ADDR_SHIN,  "Shin",  "Sensor betis (0x69) tidak dikesan", false, false, 0, 0, 0 };

static bool mpuWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuInit(uint8_t addr) {
  if (!mpuWrite(addr, 0x6B, 0x01)) return false;  // wake up, gyro X clock
  delay(5);
  mpuWrite(addr, 0x19, 0x09);   // sample rate 100 Hz
  mpuWrite(addr, 0x1A, 0x03);   // low-pass filter ~44 Hz
  mpuWrite(addr, 0x1B, 0x08);   // gyro ±500 °/s  (65.5 LSB per °/s)
  mpuWrite(addr, 0x1C, 0x00);   // accel ±2 g     (16384 LSB per g)
  return true;
}

// Reads accel X/Y (in g) and gyro Z (in °/s). Board Z points out of the leg,
// so bending the knee rotates each board around its Z axis.
static bool mpuRead(uint8_t addr, float& ax, float& ay, float& az, float& gz) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  ax = (int16_t)((b[0] << 8) | b[1]) / 16384.0f;
  ay = (int16_t)((b[2] << 8) | b[3]) / 16384.0f;
  az = (int16_t)((b[4] << 8) | b[5]) / 16384.0f;
  gz = (int16_t)((b[12] << 8) | b[13]) / 65.5f;
  return true;
}

static void calibrateGyro(Segment& s) {
  // Average the gyro for ~0.3 s. Only keep it if the brace was still.
  float sum = 0, sumSq = 0;
  int n = 0;
  for (int i = 0; i < 60; i++) {
    float ax, ay, az, gz;
    if (mpuRead(s.addr, ax, ay, az, gz)) { sum += gz; sumSq += gz * gz; n++; }
    delay(5);
  }
  if (n < 30) return;
  float mean = sum / n;
  float var  = sumSq / n - mean * mean;
  if (var < 4.0f && fabsf(mean) < 10.0f) s.gyroBias = mean;
}

static bool segmentBegin(Segment& s) {
  s.ok = mpuInit(s.addr);
  s.fails = 0;
  s.hasAngle = false;
  if (s.ok) calibrateGyro(s);
  return s.ok;
}

static void segmentUpdate(Segment& s, float dt) {
  if (!s.ok) return;
  float ax, ay, az, gz;
  if (!mpuRead(s.addr, ax, ay, az, gz)) {
    if (++s.fails >= 5) { s.ok = false; s.hasAngle = false; }
    return;
  }
  s.fails = 0;
  gz -= s.gyroBias;

  float accAngle = atan2f(-ay, ax) * RAD_TO_DEG;
  float mag = sqrtf(ax * ax + ay * ay + az * az);

  if (!s.hasAngle) {
    s.angle = accAngle;
    s.hasAngle = true;
    return;
  }
  float predicted = s.angle + gz * dt;
  // Trust the accelerometer only when it mostly sees gravity (not a kick or a bump)
  float k = (mag > 0.75f && mag < 1.25f) ? (1.0f - FILTER_ALPHA) : 0.0f;
  s.angle = wrap180(predicted + k * wrap180(accAngle - predicted));
}

static void sensorsBegin() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Serial.printf("Thigh MPU6050 (0x68): %s\n", segmentBegin(thigh) ? "OK" : "NOT FOUND");
  Serial.printf("Shin MPU6050 (0x69): %s\n",  segmentBegin(shin)  ? "OK" : "NOT FOUND");
}

static void sensorsUpdate(float dt) {
  segmentUpdate(thigh, dt);
  segmentUpdate(shin, dt);

  if (!thigh.ok)      strlcpy(errMsg, thigh.errText, sizeof(errMsg));
  else if (!shin.ok)  strlcpy(errMsg, shin.errText,  sizeof(errMsg));
  else                errMsg[0] = 0;

  sensorOk = thigh.ok && shin.ok && thigh.hasAngle && shin.hasAngle;
  if (sensorOk) {
    rawAngle  = wrap180(thigh.angle - shin.angle);
    // fabs: works for the left or the right leg without changing anything
    kneeAngle = fabsf(wrap180(rawAngle - zeroOffset));
  }
}

static void sensorsRetry() {
  // A loose wire can come back: try to find missing sensors every 2 s
  if (!thigh.ok && segmentBegin(thigh)) Serial.println("Thigh MPU6050 (0x68): OK (reconnected)");
  if (!shin.ok  && segmentBegin(shin))  Serial.println("Shin MPU6050 (0x69): OK (reconnected)");
}

static void sensorsDebug() {
  Serial.printf("knee %.1f°  (thigh %.1f, shin %.1f)  limit %d  %s\n",
                kneeAngle, thigh.angle, shin.angle, limitDeg, warning ? "WARNING" : "ok");
}

// ---------------------- Potentiometer ----------------------
#else

float potAngle = 0;
int   potRaw   = 0;
bool  potHasValue = false;

static void sensorsBegin() {
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_POT, ADC_11db);
  Serial.println("Sensor mode: potentiometer on GPIO 34");
}

static void sensorsUpdate(float) {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(PIN_POT);
  potRaw = sum / 8;
  float a = (potRaw - POT_AT_0) * 90.0f / (float)(POT_AT_90 - POT_AT_0);
  potAngle = potHasValue ? potAngle + 0.3f * (a - potAngle) : a;
  potHasValue = true;

  rawAngle  = potAngle;
  kneeAngle = fabsf(potAngle - zeroOffset);
  if (kneeAngle > 180) kneeAngle = 180;
  sensorOk  = true;
  errMsg[0] = 0;
}

static void sensorsRetry() {}

static void sensorsDebug() {
  Serial.printf("pot raw %d  ->  knee %.1f°  limit %d  %s\n",
                potRaw, kneeAngle, limitDeg, warning ? "WARNING" : "ok");
}

#endif

// ------------------------- Warning -------------------------

static void updateWarning(uint32_t now) {
  if (!sensorOk) {
    warning = false;
    overLimit = false;
    return;
  }
  if (kneeAngle > limitDeg) {
    if (!overLimit) { overLimit = true; overSince = now; }
    if (!warning && now - overSince >= WARN_DELAY_MS) {
      warning = true;
      warnStart = now;
      Serial.printf("WARNING: knee %.1f° > limit %d°\n", kneeAngle, limitDeg);
    }
  } else {
    overLimit = false;
    if (warning && kneeAngle < limitDeg - HYSTERESIS_DEG) warning = false;
  }
}

static void updateOutputs(uint32_t now) {
  bool motor = false;
  if (manualVibOn) {
    if ((int32_t)(now - manualVibEnd) < 0) motor = true;
    else manualVibOn = false;
  }
  if (!motor && warning && autoVib) {
    motor = ((now - warnStart) % (PULSE_ON_MS + PULSE_OFF_MS)) < PULSE_ON_MS;
  }
  digitalWrite(PIN_MOTOR, motor ? HIGH : LOW);

  bool red, green;
  if (warning)        { red = (now / 125) % 2; green = false; }   // fast blink
  else if (!sensorOk) { red = (now / 500) % 2; green = false; }   // slow blink = sensor problem
  else                { red = false;           green = true;  }
  digitalWrite(PIN_LED_RED,   red   ? HIGH : LOW);
  digitalWrite(PIN_LED_GREEN, green ? HIGH : LOW);
}

// ------------------------ Settings -------------------------

static void loadPrefs() {
  prefs.begin("aclguard", true);
  limitDeg   = prefs.getInt("lim", DEFAULT_LIMIT);
  zeroOffset = prefs.getFloat("zero", 0);
  autoVib    = prefs.getBool("auto", true);
  prefs.end();
  limitDeg = constrain(limitDeg, LIMIT_MIN, LIMIT_MAX);
}

static void markPrefsDirty() {
  prefsDirty = true;
  prefsChangedAt = millis();
}

// Save 1 s after the last change, so dragging the slider doesn't wear out the flash
static void savePrefsIfNeeded(uint32_t now) {
  if (!prefsDirty || now - prefsChangedAt < 1000) return;
  prefs.begin("aclguard", false);
  prefs.putInt("lim", limitDeg);
  prefs.putFloat("zero", zeroOffset);
  prefs.putBool("auto", autoVib);
  prefs.end();
  prefsDirty = false;
  Serial.println("Settings saved");
}

// ------------------------ Network --------------------------

static void sendState(int client = -1) {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"a\":%.1f,\"lim\":%d,\"w\":%d,\"auto\":%d,\"err\":\"%s\"}",
           sensorOk ? kneeAngle : 0.0f, limitDeg, warning ? 1 : 0, autoVib ? 1 : 0, errMsg);
  if (client >= 0) ws.sendTXT((uint8_t)client, buf);
  else             ws.broadcastTXT(buf);
}

static void handleCommand(const String& cmd) {
  Serial.printf("Command: %s\n", cmd.c_str());
  if (cmd.startsWith("LIMIT:")) {
    limitDeg = constrain(cmd.substring(6).toInt(), LIMIT_MIN, LIMIT_MAX);
    warning = false;
    overLimit = false;
    markPrefsDirty();
  } else if (cmd.startsWith("VIB:")) {
    int ms = constrain(cmd.substring(4).toInt(), 50, MANUAL_VIB_MAX_MS);
    manualVibOn  = true;
    manualVibEnd = millis() + ms;
  } else if (cmd == "ZERO") {
    if (sensorOk) {
      zeroOffset = rawAngle;
      kneeAngle  = 0;
      warning = false;
      overLimit = false;
      markPrefsDirty();
      Serial.println("Calibrated: leg straight = 0°");
    }
  } else if (cmd.startsWith("AUTO:")) {
    autoVib = cmd.substring(5).toInt() != 0;
    markPrefsDirty();
  }
  sendState();
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("Phone %u connected\n", num);
      sendState(num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("Phone %u disconnected\n", num);
      break;
    case WStype_TEXT: {
      String cmd;
      cmd.reserve(length);
      for (size_t i = 0; i < length; i++) cmd += (char)payload[i];
      cmd.trim();
      cmd.toUpperCase();
      handleCommand(cmd);
      break;
    }
    default:
      break;
  }
}

static void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
}

static void handleNotFound() {
  // Any other address (phone connectivity checks etc.) goes to the dashboard
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

// ------------------------- Main ----------------------------

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ACL-GUARD ===");

  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);

  loadPrefs();
  Serial.printf("Limit %d°, auto vibration %s\n", limitDeg, autoVib ? "ON" : "OFF");

  sensorsBegin();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);
  Serial.printf("WiFi \"%s\" ready. Open http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();

  ws.begin();
  ws.onEvent(onWsEvent);

  // Startup buzz + both LEDs, so you know the motor and LEDs work
  digitalWrite(PIN_MOTOR, HIGH);
  digitalWrite(PIN_LED_RED, HIGH);
  digitalWrite(PIN_LED_GREEN, HIGH);
  delay(250);
  digitalWrite(PIN_MOTOR, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);

  lastSample = millis();
}

void loop() {
  server.handleClient();
  ws.loop();

  uint32_t now = millis();

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    float dt = (now - lastSample) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    lastSample = now;
    sensorsUpdate(dt);
    updateWarning(now);
  }

  updateOutputs(now);

  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;
    if (ws.connectedClients() > 0) sendState();
  }

  if (now - lastRetry >= 2000) {
    lastRetry = now;
    sensorsRetry();
  }

  if (now - lastDebug >= 500) {
    lastDebug = now;
    sensorsDebug();
  }

  savePrefsIfNeeded(now);
}
