/*
  =====================================================================
   ACL-GUARD  —  Smart Knee Support (SMK Darulaman Heights 1, Vanguard)
  =====================================================================
   Board   : ESP32 Dev Module (ESP32-WROOM-32 / DevKit V1)
   Sensors : 2 x MPU6050  (thigh = 0x68, shin = 0x69)
             -- or a rotary potentiometer on the brace hinge (see SENSOR_MODE)
   Outputs : Coin vibration motor (through NPN transistor / MOSFET)
             Red LED (warning) + Green LED (safe)
   Phone   : ESP32 makes its own WiFi "ACL-GUARD". Join it on the phone,
             open http://192.168.4.1  -> live dashboard.
             Live data + commands go over a WebSocket on port 81.

   Needs ONE extra library (Arduino IDE > Library Manager):
     "WebSockets" by Markus Sattler (Links2004)
   Everything else (WiFi, WebServer, Wire, Preferences) comes with the
   ESP32 board package.

   How the angle is measured (SENSOR_MODE 1):
     Each MPU6050 gives the tilt of its leg segment in the side (sagittal)
     plane: accelerometer (gravity direction) fused with the gyroscope
     (complementary filter). Knee angle = difference between thigh tilt
     and shin tilt. Press "Calibrate" in the app with the leg straight
     to set 0°.
  =====================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Preferences.h>
#include "dashboard.h"          // the phone web page (generated from dashboard/index.html)

// ------------------------- SETTINGS ---------------------------------
#define SENSOR_MODE   1         // 1 = two MPU6050 (thigh + shin)   2 = potentiometer on hinge

const char* WIFI_SSID = "ACL-GUARD";
const char* WIFI_PASS = "acl12345";   // at least 8 characters

// Pins (ESP32 DevKit V1)
const int PIN_SDA     = 21;
const int PIN_SCL     = 22;
const int PIN_MOTOR   = 25;     // -> 1k resistor -> transistor base / MOSFET gate
const int PIN_LED_RED = 26;
const int PIN_LED_GRN = 27;
const int PIN_POT     = 34;     // only for SENSOR_MODE 2 (input-only ADC pin)

// Potentiometer calibration (SENSOR_MODE 2): ADC reading at 0° and at 90°
int POT_AT_0  = 1200;
int POT_AT_90 = 2900;

// Warning behaviour
const uint32_t OVER_HOLD_MS   = 300;   // must stay over the limit this long before warning
const float    HYSTERESIS_DEG = 3.0;   // warning clears when angle < limit - 3°
const uint16_t BUZZ_ON_MS     = 400;   // vibration pulse pattern while over limit
const uint16_t BUZZ_OFF_MS    = 300;
const uint32_t SEND_EVERY_MS  = 50;    // 20 updates per second to the phone

// ------------------------- GLOBALS ----------------------------------
WebServer        http(80);
WebSocketsServer ws(81);
Preferences      prefs;

const uint8_t MPU_THIGH = 0x68;   // AD0 -> GND (or not connected)
const uint8_t MPU_SHIN  = 0x69;   // AD0 -> 3.3V

bool  thighOk = false, shinOk = false;
uint8_t  thighFails = 0, shinFails = 0;       // bad reads in a row
uint32_t thighRetry = 0, shinRetry = 0;       // last time we tried to wake a missing sensor
float thighAng = 0, shinAng = 0;  // fused segment tilt (deg)
float zeroOffset = 0;             // relative angle when leg is straight
float kneeAngle = 0;              // final knee flexion (deg), 0 = straight
float limitDeg  = 45;             // safe max flexion (set from the app)
bool  autoVibrate = true;

bool     warning = false;
uint32_t overSince = 0;
uint32_t manualVibUntil = 0;      // "Vibrate brace" button from the app
uint32_t lastSend = 0, lastIMU = 0, lastPrint = 0;

// ------------------------- HELPERS ----------------------------------
float wrap180(float a) {
  while (a > 180)  a -= 360;
  while (a < -180) a += 360;
  return a;
}

bool mpuWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuInit(uint8_t addr) {
  if (!mpuWrite(addr, 0x6B, 0x00)) return false;   // wake up (PWR_MGMT_1)
  mpuWrite(addr, 0x1A, 0x03);                      // DLPF ~44 Hz
  mpuWrite(addr, 0x1B, 0x08);                      // gyro  ±500 °/s  -> 65.5 LSB/(°/s)
  mpuWrite(addr, 0x1C, 0x00);                      // accel ±2 g      -> 16384 LSB/g
  return true;
}

// Reads ax, ay (g) and gz (°/s). Returns false if the sensor did not answer.
bool mpuRead(uint8_t addr, float &ax, float &ay, float &gz) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  int16_t rax = (b[0] << 8) | b[1];
  int16_t ray = (b[2] << 8) | b[3];
  int16_t rgz = (b[12] << 8) | b[13];
  ax = rax / 16384.0;
  ay = ray / 16384.0;
  gz = rgz / 65.5;
  return true;
}

// Complementary filter for one leg segment.
// Mount each board on the OUTER side of the leg, chip facing outwards,
// X-axis along the leg. Both boards must face the same way.
void fuse(float &angle, float ax, float ay, float gz, float dt) {
  float accAng = atan2f(-ay, ax) * 57.29578f;      // tilt from gravity
  angle = wrap180(angle + gz * dt);                // integrate gyro
  angle = wrap180(angle + 0.03f * wrap180(accAng - angle)); // pull toward accel
}

// Reads one sensor and updates its tilt. One bad read is ignored; after 5 bad
// reads in a row (~50 ms) the sensor counts as missing, and we try to wake it
// up again once a second (not 100 times a second, which slowed the loop).
void readSegment(uint8_t addr, float &ang, bool &ok, uint8_t &fails, uint32_t &lastTry, float dt) {
  float ax, ay, gz;
  if (!ok) {
    if (millis() - lastTry < 1000) return;
    lastTry = millis();
    if (!mpuInit(addr) || !mpuRead(addr, ax, ay, gz)) return;
    ang = atan2f(-ay, ax) * 57.29578f;             // restart the filter from gravity
    ok = true;
    fails = 0;
    Serial.printf("MPU6050 (0x%02X): OK again\n", addr);
    return;
  }
  if (mpuRead(addr, ax, ay, gz)) {
    fails = 0;
    fuse(ang, ax, ay, gz, dt);
  } else if (++fails >= 5) {
    ok = false;
    lastTry = millis();
    Serial.printf("MPU6050 (0x%02X): lost - check wiring\n", addr);
  }
}

// ------------------------- SENSOR UPDATE ----------------------------
void readKnee() {
  uint32_t now = micros();
  static uint32_t lastUs = now;
  float dt = (now - lastUs) / 1e6f;
  lastUs = now;
  if (dt <= 0 || dt > 0.2f) dt = 0.01f;

#if SENSOR_MODE == 1
  readSegment(MPU_THIGH, thighAng, thighOk, thighFails, thighRetry, dt);
  readSegment(MPU_SHIN,  shinAng,  shinOk,  shinFails,  shinRetry,  dt);

  if (thighOk && shinOk) {                         // keep the last angle if a sensor is missing
    float rel = wrap180(thighAng - shinAng);
    float k = fabsf(wrap180(rel - zeroOffset));
    kneeAngle = 0.8f * kneeAngle + 0.2f * k;       // light smoothing
  }
#else
  thighOk = shinOk = true;
  int raw = analogRead(PIN_POT);
  float k = (raw - POT_AT_0) * 90.0f / (float)(POT_AT_90 - POT_AT_0) - zeroOffset;
  if (k < 0) k = 0;
  kneeAngle = 0.8f * kneeAngle + 0.2f * k;
#endif
}

void calibrateZero() {
#if SENSOR_MODE == 1
  if (!(thighOk && shinOk)) {                      // angle is not live: don't save a wrong zero
    Serial.println("Calibrate ignored: a sensor is missing");
    return;
  }
  zeroOffset = wrap180(thighAng - shinAng);
#else
  zeroOffset = 0;
  int raw = analogRead(PIN_POT);
  zeroOffset = (raw - POT_AT_0) * 90.0f / (float)(POT_AT_90 - POT_AT_0);
#endif
  kneeAngle = 0;
  prefs.putFloat("zero", zeroOffset);
  Serial.printf("Calibrated. zeroOffset = %.1f\n", zeroOffset);
}

// ------------------------- WARNING + MOTOR --------------------------
void updateWarning() {
  uint32_t now = millis();
  bool sensorsOk = thighOk && shinOk;
  if (!sensorsOk) {
    // The angle is frozen while a sensor is missing: don't keep buzzing on an old value.
    // The red LED blinks slowly instead and the app shows which sensor is missing.
    warning = false;
    overSince = 0;
  } else if (kneeAngle > limitDeg) {
    if (overSince == 0) overSince = now;
    if (now - overSince >= OVER_HOLD_MS) warning = true;
  } else {
    overSince = 0;
    if (kneeAngle < limitDeg - HYSTERESIS_DEG) warning = false;
  }

  // Motor: manual pulse from app, or pulsing pattern while warning
  bool motor = false;
  if (now < manualVibUntil) {
    motor = true;
  } else if (warning && autoVibrate) {
    motor = (now % (BUZZ_ON_MS + BUZZ_OFF_MS)) < BUZZ_ON_MS;
  }
  digitalWrite(PIN_MOTOR, motor ? HIGH : LOW);

  // LEDs: red blinks while warning, green on while safe
  digitalWrite(PIN_LED_RED, warning ? ((now / 150) % 2) : (!sensorsOk ? ((now / 600) % 2) : LOW));
  digitalWrite(PIN_LED_GRN, (!warning && sensorsOk) ? HIGH : LOW);
}

// ------------------------- WEBSOCKET --------------------------------
// Commands from the app:  LIMIT:45   VIB:700   ZERO   AUTO:1 / AUTO:0
void onWsEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) {
    Serial.printf("Phone %u connected\n", client);
  } else if (type == WStype_TEXT) {
    String cmd = String((char*)payload).substring(0, len);
    cmd.trim();
    if (cmd.startsWith("LIMIT:")) {
      float v = cmd.substring(6).toFloat();
      if (v >= 5 && v <= 150) { limitDeg = v; prefs.putFloat("limit", limitDeg); }
    } else if (cmd.startsWith("VIB:")) {
      uint32_t ms = constrain(cmd.substring(4).toInt(), 50, 3000);
      manualVibUntil = millis() + ms;
    } else if (cmd == "ZERO") {
      calibrateZero();
    } else if (cmd.startsWith("AUTO:")) {
      autoVibrate = cmd.substring(5).toInt() == 1;
      prefs.putBool("auto", autoVibrate);
    }
    Serial.println("CMD " + cmd);
  }
}

void sendState() {
  const char* err = !thighOk ? "thigh" : (!shinOk ? "shin" : "");
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"a\":%.1f,\"lim\":%.0f,\"w\":%d,\"auto\":%d,\"err\":\"%s\"}",
           kneeAngle, limitDeg, warning ? 1 : 0, autoVibrate ? 1 : 0, err);
  ws.broadcastTXT(buf);
}

// ------------------------- SETUP / LOOP -----------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);

  prefs.begin("aclguard", false);
  limitDeg    = prefs.getFloat("limit", 45);
  zeroOffset  = prefs.getFloat("zero", 0);
  autoVibrate = prefs.getBool("auto", true);

#if SENSOR_MODE == 1
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  thighOk = mpuInit(MPU_THIGH);
  shinOk  = mpuInit(MPU_SHIN);
  Serial.printf("Thigh MPU6050 (0x68): %s\n", thighOk ? "OK" : "NOT FOUND");
  Serial.printf("Shin MPU6050 (0x69): %s\n",  shinOk  ? "OK" : "NOT FOUND");
  // start both filters from the accelerometer so they settle instantly
  float ax, ay, gz;
  if (mpuRead(MPU_THIGH, ax, ay, gz)) thighAng = atan2f(-ay, ax) * 57.29578f;
  if (mpuRead(MPU_SHIN,  ax, ay, gz)) shinAng  = atan2f(-ay, ax) * 57.29578f;
#else
  analogReadResolution(12);
#endif

  // Startup buzz so you know the motor works
  digitalWrite(PIN_MOTOR, HIGH); delay(200); digitalWrite(PIN_MOTOR, LOW);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);                            // faster, steadier live data to the phone
  Serial.print("WiFi: "); Serial.print(WIFI_SSID);
  Serial.print("   Open http://"); Serial.println(WiFi.softAPIP());

  http.on("/", []() { http.send(200, "text/html", INDEX_HTML); });
  http.onNotFound([]() { http.sendHeader("Location", "/"); http.send(302); });
  http.begin();

  ws.begin();
  ws.onEvent(onWsEvent);
}

void loop() {
  http.handleClient();
  ws.loop();

  uint32_t now = millis();
  if (now - lastIMU >= 10) {          // 100 Hz sensor reading
    lastIMU = now;
    readKnee();
    updateWarning();
  }
  if (now - lastSend >= SEND_EVERY_MS) {
    lastSend = now;
    sendState();
  }
  if (now - lastPrint >= 500) {       // Serial Monitor (115200): live values twice a second
    lastPrint = now;
#if SENSOR_MODE == 2
    // Use these "pot raw" numbers for POT_AT_0 (leg straight) and POT_AT_90 (bent 90 deg)
    Serial.printf("pot raw %4d   knee %5.1f deg   limit %.0f%s\n",
                  analogRead(PIN_POT), kneeAngle, limitDeg, warning ? "   WARNING" : "");
#else
    Serial.printf("knee %5.1f deg   limit %.0f%s%s%s\n", kneeAngle, limitDeg,
                  warning ? "   WARNING" : "",
                  thighOk ? "" : "   [thigh sensor missing]", shinOk ? "" : "   [shin sensor missing]");
#endif
  }
}
