/*
 * PROJECT: TerraVolts — Smart Solar-Diesel Hybrid Dewatering System for Copper Mines
 * TEAM: Terra World Dynamics (Team Lead: Sathyajith G)
 * EVENT: L&T SMARTATHON 2.0 (IEEE Madras Section / IEEE IAS)
 * BOARD: PCBCUPID GLYPH-C6 (ESP32-C6)
 * VERSION: 28.1 — Proportional LDR Solar Tracking + Debounced Solar/Diesel Failover
 */

#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Firebase_ESP_Client.h>
#include <ESP32Servo.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include "time.h"

// ==========================================
// 1. CREDENTIALS (Replace with your own)
// ==========================================
#define WIFI_SSID        "YOUR_WIFI_SSID"
#define WIFI_PASSWORD    "YOUR_WIFI_PASSWORD"
#define FIREBASE_API_KEY "YOUR_FIREBASE_API_KEY"
#define FIREBASE_DB_URL  "https://your-project-default-rtdb.firebaseio.com"
#define FIREBASE_EMAIL   "your-email@example.com"
#define FIREBASE_PASS    "YOUR_FIREBASE_PASSWORD"

// ==========================================
// 2. PIN MAPPING (PCBCUPID GLYPH-C6)
// ==========================================
#define SOLAR_VOLT_PIN    0    // A0  -> GPIO0  (Solar Voltage Divider)
#define WATER_LEVEL_PIN   1    // A1  -> GPIO1  (Mine Sump Water Level Sensor)
#define LDR1_PIN          2    // A2  -> GPIO2  (Sunflower Solar Tracker LDR 1)
#define LDR2_PIN          3    // A3  -> GPIO3  (Sunflower Solar Tracker LDR 2)
#define I2C_SDA_PIN       4    // SDA -> GPIO4  (16x2 I2C LCD)
#define I2C_SCL_PIN       5    // SCL -> GPIO5  (16x2 I2C LCD)
#define FLOW_PIN          6    // D6  -> GPIO6  (YF-S201 Flow Meter)
#define SERVO_PIN         7    // D7  -> GPIO7  (SG90 Solar Tracking Servo)
#define LED_GREEN_PIN     8    // D8  -> GPIO8  (System Running Indicator)
#define LED_RED_PIN       9    // D9  -> GPIO9  (Shutdown / Fault Indicator)
#define BUZZER_PIN        14   // D14 -> GPIO14 (Audio Alert Buzzer)
#define RELAY_SOLAR_PIN   18   // D18 -> GPIO18 (Primary Solar Pump Relay)
#define RELAY_DIESEL_PIN  23   // D23 -> GPIO23 (Backup Diesel Generator Relay)

// ==========================================
// 3. VOLTAGE SENSOR & FAILOVER THRESHOLDS
// ==========================================
#define ADC_REF              3.3f
#define ADC_RES              4095.0f
#define VOLT_DIVIDER         5.0f
#define ADC_NOISE_FLOOR      120

#define SOLAR_V_THRESHOLD    2.0f    // Switch to Diesel below this voltage
#define SOLAR_V_RECOVER      2.5f    // Recover to Solar above this voltage
#define SOLAR_V_FULL         12.6f
#define SOLAR_V_CUTOFF       1.5f
#define RELAY_CONFIRM_MS     2000UL  // Anti-chatter confirmation delay

// ==========================================
// 4. WATER LEVEL SENSOR THRESHOLDS
// ==========================================
#define WATER_WET_VAL        1200
#define WATER_DRY_VAL        1000
#define DRY_CONFIRM_MS       4000UL

// ==========================================
// 5. SUNFLOWER SOLAR TRACKER CONSTANTS
// ==========================================
#define SERVO_CENTER         90
#define SERVO_MIN            0
#define SERVO_MAX            180
#define LDR_ERROR            5
#define TRACKER_INTERVAL     20UL

int getServoStep(int diff) {
  if (diff >= 800) return 5;
  if (diff >= 400) return 3;
  if (diff >= 100) return 2;
  return 1;
}

// ==========================================
// 6. TIMING & FLOW CALIBRATION CONSTANTS
// ==========================================
#define RELAY_ON             LOW
#define RELAY_OFF            HIGH
#define FLOW_CALIBRATION     7.5f     // YF-S201 pulses per second per L/min
#define FIREBASE_INTERVAL    5000UL
#define LCD_ROTATE_MS        1500UL
#define HISTORY_INTERVAL     300000UL

// ==========================================
// 7. GLOBAL OBJECTS & STATE
// ==========================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
FirebaseData      fbdo;
FirebaseAuth      fbAuth;
FirebaseConfig    fbConfig;
Servo             sg90;

volatile uint32_t pulseCount     = 0;

float  g_solarVolt    = 0.0f;
float  g_flowRate     = 0.0f;
float  g_totalLiters  = 0.0f;
bool   g_waterPresent = false;
int    g_rawWater     = 0;

bool   g_firebaseReady = false;
bool   g_usingDiesel   = false;
bool   g_motorAllowed  = true;
bool   g_shutdown      = false;
bool   g_prevShutdown  = false;
String g_status        = "BOOTING";

unsigned long relayBelowSince  = 0;
unsigned long relayAboveSince  = 0;

int  g_servoPos  = SERVO_CENTER;
int  g_ldr1      = 0;
int  g_ldr2      = 0;

unsigned long notWetSince = 0;
bool          wasAboveWet = false;

unsigned long lastLCDRotate = 0;
int           lcdScreen     = 0;

unsigned long lastFirebaseMillis = 0;
unsigned long lastFlowMillis     = 0;
unsigned long lastHistoryMillis  = 0;
unsigned long lastTrackerMs      = 0;

// ==========================================
// 8. INTERRUPT — YF-S201 FLOW METER
// ==========================================
void IRAM_ATTR flowPulseCounter() {
  pulseCount++;
}

// ==========================================
// 9. OVERSAMPLED SOLAR VOLTAGE READING
// ==========================================
float readSolarVoltage() {
  long vSum = 0;
  for (int i = 0; i < 50; i++) {
    vSum += analogRead(SOLAR_VOLT_PIN);
    delayMicroseconds(200);
  }
  float avgADC = vSum / 50.0f;
  if (avgADC < ADC_NOISE_FLOOR) return 0.0f;
  return (avgADC * (ADC_REF / ADC_RES)) * VOLT_DIVIDER;
}

// ==========================================
// 10. LED & BUZZER ALERT SYSTEM
// ==========================================
void updateLEDs() {
  bool justShutdown = (g_shutdown && !g_prevShutdown);
  bool justResumed  = (!g_shutdown && g_prevShutdown);

  if (g_shutdown) {
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN,   HIGH);

    if (justShutdown) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1500);
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN,   LOW);
    digitalWrite(BUZZER_PIN,    LOW);

    if (justResumed) {
      digitalWrite(BUZZER_PIN, HIGH); delay(200);
      digitalWrite(BUZZER_PIN, LOW);  delay(150);
      digitalWrite(BUZZER_PIN, HIGH); delay(200);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
  g_prevShutdown = g_shutdown;
}

// ==========================================
// 11. SUNFLOWER-STYLE LDR SOLAR TRACKER
// ==========================================
void updateSolarTracker() {
  if (g_shutdown) return;

  unsigned long now = millis();
  if (now - lastTrackerMs < TRACKER_INTERVAL) return;
  lastTrackerMs = now;

  long raw1 = 0, raw2 = 0;
  const int samples = 5;
  for (int i = 0; i < samples; i++) {
    raw1 += analogRead(LDR1_PIN);
    raw2 += analogRead(LDR2_PIN);
    delayMicroseconds(200);
  }
  g_ldr1 = raw1 / samples;
  g_ldr2 = raw2 / samples;

  int diff = abs(g_ldr1 - g_ldr2);
  const int bias = 2;

  if (diff > LDR_ERROR) {
    int step = getServoStep(diff);
    if (g_ldr1 > g_ldr2 + bias) {
      g_servoPos = max(SERVO_MIN, g_servoPos - step);
    } else if (g_ldr2 > g_ldr1 + bias) {
      g_servoPos = min(SERVO_MAX, g_servoPos + step);
    }
    sg90.write(g_servoPos);
  } else if (diff > 2) {
    if (g_ldr1 > g_ldr2 + 1) {
      g_servoPos = max(SERVO_MIN, g_servoPos - 1);
    } else if (g_ldr2 > g_ldr1 + 1) {
      g_servoPos = min(SERVO_MAX, g_servoPos + 1);
    }
    sg90.write(g_servoPos);
  }
}

// ==========================================
// 12. NTP TIME & NETWORK INITIALIZATION
// ==========================================
void syncTime() {
  configTime(19800, 0, "pool.ntp.org", "time.nist.gov");
  struct tm t;
  int attempts = 0;
  while (!getLocalTime(&t) && attempts < 20) {
    delay(500);
    attempts++;
  }
}

String getTimestamp() {
  struct tm t;
  if (!getLocalTime(&t)) {
    unsigned long s = millis() / 1000;
    return "up" + String(s / 3600) + "h" + String((s / 60) % 60) + "m";
  }
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &t);
  return String(buf);
}

String getMonthKey() {
  struct tm t;
  if (!getLocalTime(&t)) return "unknown";
  char buf[10];
  strftime(buf, sizeof(buf), "%Y-%m", &t);
  return String(buf);
}

void connectWiFi() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("WiFi Connect... ");

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(250);
    attempts++;
  }

  lcd.setCursor(0, 1);
  if (WiFi.status() == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    while (ip.length() < 16) ip += ' ';
    lcd.print(ip.substring(0, 16));
  } else {
    lcd.print("Offline Mode    ");
  }
  delay(800);
}

void initFirebase() {
  if (WiFi.status() != WL_CONNECTED) return;

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Firebase...     ");

  fbConfig.api_key                  = FIREBASE_API_KEY;
  fbConfig.database_url             = FIREBASE_DB_URL;
  fbAuth.user.email                 = FIREBASE_EMAIL;
  fbAuth.user.password              = FIREBASE_PASS;
  fbConfig.token_status_callback    = tokenStatusCallback;
  fbConfig.timeout.serverResponse   = 10 * 1000;
  fbConfig.timeout.socketConnection = 10 * 1000;

  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);

  unsigned long authWait = millis();
  while (!Firebase.ready() && millis() - authWait < 5000) {
    delay(200);
  }

  g_firebaseReady = Firebase.ready();
  lcd.setCursor(0, 1);
  lcd.print(g_firebaseReady ? "Cloud: Online   " : "Cloud: Offline  ");
  delay(600);
}

// ==========================================
// 13. REMOTE APP EMERGENCY SHUTDOWN
// ==========================================
void checkShutdownCommand() {
  if (!g_firebaseReady || !Firebase.ready() || WiFi.status() != WL_CONNECTED) return;

  if (Firebase.RTDB.getBool(&fbdo, "/pumpSystem/control/shutdown")) {
    bool cmd = fbdo.boolData();
    if (cmd != g_shutdown) {
      g_shutdown = cmd;
      if (g_shutdown) {
        g_servoPos = SERVO_CENTER;
        sg90.write(g_servoPos);
      }
    }
  }
}

// ==========================================
// 14. SENSOR ACQUISITION & HYBRID RELAY LOGIC
// ==========================================
void readSensors() {
  if (g_shutdown) return;

  g_solarVolt = readSolarVoltage();

  int wSum = 0;
  for (int i = 0; i < 5; i++) {
    wSum += analogRead(WATER_LEVEL_PIN);
    delay(2);
  }
  g_rawWater = wSum / 5;

  unsigned long now = millis();
  if (g_rawWater > WATER_WET_VAL) {
    g_waterPresent = true;
    wasAboveWet    = true;
    notWetSince    = 0;
  } else if (g_rawWater < WATER_DRY_VAL) {
    g_waterPresent = false;
    wasAboveWet    = false;
    notWetSince    = 0;
  } else {
    if (wasAboveWet && notWetSince == 0) notWetSince = now;
    if (notWetSince > 0 && (now - notWetSince >= DRY_CONFIRM_MS)) {
      g_waterPresent = false;
      wasAboveWet    = false;
      notWetSince    = 0;
    }
  }

  if (now - lastFlowMillis >= 1000UL) {
    detachInterrupt(digitalPinToInterrupt(FLOW_PIN));
    uint32_t count = pulseCount;
    pulseCount     = 0;
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowPulseCounter, FALLING);
    g_flowRate     = (float)count / FLOW_CALIBRATION;
    g_totalLiters += (g_flowRate / 60.0f);
    lastFlowMillis = now;
  }
}

void runLogic() {
  if (!g_waterPresent || g_shutdown) {
    digitalWrite(RELAY_SOLAR_PIN, RELAY_OFF);
    digitalWrite(RELAY_DIESEL_PIN, RELAY_OFF);
    g_usingDiesel = false;
    relayBelowSince = 0;
    relayAboveSince = 0;
    g_status = g_shutdown ? "SHUTDOWN" : "OFF (DRY)";
    return;
  }

  unsigned long now = millis();

  // Immediate failover from Solar to Diesel when solar voltage drops
  if (g_solarVolt < SOLAR_V_THRESHOLD) {
    relayAboveSince = 0;
    if (!g_usingDiesel) {
      digitalWrite(RELAY_SOLAR_PIN, RELAY_OFF);
      delay(50);
      digitalWrite(RELAY_DIESEL_PIN, RELAY_ON);
      g_usingDiesel = true;
    }
    g_status = "DIESEL";
  }
  // Confirmed recovery back to Solar power
  else if (g_solarVolt >= SOLAR_V_RECOVER) {
    if (g_usingDiesel) {
      if (relayAboveSince == 0) relayAboveSince = now;
      if (now - relayAboveSince >= RELAY_CONFIRM_MS) {
        digitalWrite(RELAY_DIESEL_PIN, RELAY_OFF);
        delay(50);
        digitalWrite(RELAY_SOLAR_PIN, RELAY_ON);
        g_usingDiesel = false;
        g_status = "SOLAR";
        relayAboveSince = 0;
      } else {
        g_status = "SOLAR READY";
      }
    } else {
      relayAboveSince = 0;
      digitalWrite(RELAY_DIESEL_PIN, RELAY_OFF);
      digitalWrite(RELAY_SOLAR_PIN, RELAY_ON);
      g_status = "SOLAR";
    }
  } else {
    g_status = g_usingDiesel ? "DIESEL" : "SOLAR";
  }
}

// ==========================================
// 15. LOCAL LCD & FIREBASE TELEMETRY
// ==========================================
void updateLCD() {
  unsigned long now = millis();

  if (g_shutdown) {
    static unsigned long lastBlink = 0;
    static bool blinkOn = false;
    if (now - lastBlink >= 800) {
      blinkOn = !blinkOn;
      lastBlink = now;
      lcd.setCursor(0, 0);
      lcd.print(blinkOn ? "SYSTEM SHUTDOWN " : "                ");
      lcd.setCursor(0, 1);
      lcd.print("  App Controlled");
    }
    return;
  }

  bool inWarningZone = (!g_usingDiesel) &&
                       (g_solarVolt >= SOLAR_V_THRESHOLD) &&
                       (g_solarVolt <= (SOLAR_V_THRESHOLD + 1.0f));

  int totalScreens = inWarningZone ? 5 : 4;

  if (now - lastLCDRotate >= LCD_ROTATE_MS) {
    lcdScreen = (lcdScreen + 1) % totalScreens;
    lastLCDRotate = now;
    lcd.clear();
  }

  if (!inWarningZone && lcdScreen >= 4) {
    lcdScreen = 0;
    lastLCDRotate = now;
    lcd.clear();
  }

  auto pad16 = [](String s) -> String {
    while (s.length() < 16) s += ' ';
    return s.substring(0, 16);
  };

  bool motorRunning = g_waterPresent && (g_status == "SOLAR" || g_status == "DIESEL");

  switch (lcdScreen) {
    case 0:
      lcd.setCursor(0, 0); lcd.print(pad16("Water:"));
      lcd.setCursor(0, 1); lcd.print(pad16(g_waterPresent ? "  Present" : "  Absent"));
      break;
    case 1:
      lcd.setCursor(0, 0); lcd.print(pad16("Motor:"));
      lcd.setCursor(0, 1); lcd.print(pad16(motorRunning ? "  Running" : "  Not Running"));
      break;
    case 2:
      lcd.setCursor(0, 0); lcd.print(pad16("Supply Source:"));
      lcd.setCursor(0, 1);
      lcd.print(pad16(motorRunning ? (g_usingDiesel ? "  Diesel" : "  Solar") : "  No Source"));
      break;
    case 3:
      lcd.setCursor(0, 0); lcd.print(pad16("Solar Voltage:"));
      lcd.setCursor(0, 1); lcd.print(pad16("  " + String(g_solarVolt, 1) + " V"));
      break;
    case 4:
      lcd.setCursor(0, 0); lcd.print(pad16(">>  WARNING  <<"));
      lcd.setCursor(0, 1); lcd.print(pad16("Diesel Soon " + String(g_solarVolt, 1) + "V"));
      break;
  }
}

void updateFirebase() {
  if (!g_firebaseReady || !Firebase.ready() || WiFi.status() != WL_CONNECTED) return;

  bool motorRunning = (g_status == "SOLAR" || g_status == "DIESEL");

  FirebaseJson json;
  json.set("solarVoltage", g_solarVolt);
  json.set("flow",         g_flowRate);
  json.set("total",        g_totalLiters);
  json.set("water",        g_waterPresent);
  json.set("waterRaw",     g_rawWater);
  json.set("status",       g_status);
  json.set("usingDiesel",  g_usingDiesel);
  json.set("motorAllowed", g_motorAllowed);
  json.set("motorRunning", motorRunning);
  json.set("motorStatus",  g_status);
  json.set("dieselActive", g_usingDiesel);
  json.set("shutdown",     g_shutdown);
  json.set("servoAngle",   g_servoPos);
  json.set("ldr1",         g_ldr1);
  json.set("ldr2",         g_ldr2);
  json.set("uptimeSec",    (int)(millis() / 1000));

  Firebase.RTDB.setJSON(&fbdo, "/pumpSystem/liveData", &json);
}

void logHistory() {
  if (!g_firebaseReady || !Firebase.ready() || WiFi.status() != WL_CONNECTED) return;

  String path = "/pumpSystem/history/" + getMonthKey() + "/" + getTimestamp();

  FirebaseJson record;
  record.set("solarVoltage", g_solarVolt);
  record.set("flowRate",     g_flowRate);
  record.set("totalLitres",  g_totalLiters);
  record.set("water",        g_waterPresent);
  record.set("status",       g_status);
  record.set("usingDiesel",  g_usingDiesel);
  record.set("shutdown",     g_shutdown);
  record.set("servoAngle",   g_servoPos);
  record.set("timestamp",    getTimestamp());
  record.set("uptimeSec",    (int)(millis() / 1000));

  Firebase.RTDB.setJSON(&fbdo, path.c_str(), &record);
}

// ==========================================
// 16. SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_SOLAR_PIN,  OUTPUT_OPEN_DRAIN);
  pinMode(RELAY_DIESEL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(RELAY_SOLAR_PIN,  RELAY_OFF);
  digitalWrite(RELAY_DIESEL_PIN, RELAY_OFF);

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN,   OUTPUT);
  pinMode(BUZZER_PIN,    OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN,   LOW);
  digitalWrite(BUZZER_PIN,    LOW);

  delay(1000);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(LDR1_PIN, INPUT);
  pinMode(LDR2_PIN, INPUT);

  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowPulseCounter, FALLING);

  sg90.setPeriodHertz(50);
  sg90.attach(SERVO_PIN, 500, 2400);
  sg90.write(SERVO_CENTER);
  delay(500);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("TERRAVOLTS v28  ");
  lcd.setCursor(0, 1); lcd.print("Starting...     ");
  delay(500);

  connectWiFi();
  initFirebase();
  syncTime();

  lastFlowMillis     = millis();
  lastFirebaseMillis = millis();
  lastLCDRotate      = millis();
  lastHistoryMillis  = millis();
  lastTrackerMs      = millis();

  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH); delay(100);
  digitalWrite(BUZZER_PIN, LOW);  delay(100);
  digitalWrite(BUZZER_PIN, HIGH); delay(100);
  digitalWrite(BUZZER_PIN, LOW);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("System Ready!   ");
  lcd.setCursor(0, 1);
  lcd.print(g_firebaseReady ? "Cloud: Online   " : "Cloud: Offline  ");
  delay(800);
}

void loop() {
  checkShutdownCommand();
  updateLEDs();
  updateSolarTracker();
  readSensors();
  runLogic();
  updateLCD();

  if (millis() - lastFirebaseMillis >= FIREBASE_INTERVAL) {
    updateFirebase();
    lastFirebaseMillis = millis();
  }

  if (millis() - lastHistoryMillis >= HISTORY_INTERVAL) {
    logHistory();
    lastHistoryMillis = millis();
  }

  delay(500);
}
