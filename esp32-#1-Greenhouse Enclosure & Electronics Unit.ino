/*
  ESP32 #1 - Irrigation node (offline)
  Solar Automatic Plant Watering System

  Board: ESP32 Dev Module (ESP32-WROOM-32, NOT WROVER)
  Libraries (Library Manager): Adafruit SSD1306, Adafruit GFX
            (Preferences, esp_task_wdt, WiFi are built into the ESP32 core)

  Pins (from the blueprint, Section 8)
    GPIO 32  M1 soil moisture AOUT (ADC1)
    GPIO 33  M2 soil moisture AOUT (ADC1)
    GPIO 34  Water-level probe signal (ADC1, input only)
    GPIO 27  Water-level probe supply (high only during a reading)
    GPIO 26  Relay IN (high = pump on, 10k pull-down to GND)
    GPIO 17  TX2 -> ESP32 #2 GPIO 26
    GPIO 16  RX2 <- ESP32 #2 GPIO 25
    GPIO 21/22  OLED SDA/SCL (0x3C)

  Behaviour
    - Wi-Fi and Bluetooth are switched off for the life of the sketch.
    - Every 10 s: read M1, M2 (10-sample average) and the reservoir probe.
    - Pump starts when average moisture < 35 %, with three guards:
        min 30 min between cycles, max 60 s per cycle, 10 min/day total.
    - Pump stops at > 55 %, at 60 s, when the reservoir is empty,
      when both sensors fail, or when the daily cap is reached.
    - Status JSON line to ESP32 #2 every 60 s (this is the heartbeat),
      plus an event line after every watering cycle.
    - Watchdog restarts the board if the loop hangs.
    - Daily pump counters survive a reboot (Preferences / flash).
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

// ============================== SETTINGS ==================================

// Set true while calibrating: prints raw readings, never runs the pump.
const bool CAL_MODE = false;

// ---- Pins ----
const uint8_t PIN_M1        = 32;
const uint8_t PIN_M2        = 33;
const uint8_t PIN_LEVEL_SIG = 34;
const uint8_t PIN_LEVEL_PWR = 27;
const uint8_t PIN_RELAY     = 26;
const uint8_t PIN_UART_TX   = 17;   // TX2
const uint8_t PIN_UART_RX   = 16;   // RX2
const uint8_t PIN_SDA       = 21;
const uint8_t PIN_SCL       = 22;

// ---- Moisture calibration (raw 12-bit ADC) ----
// Capacitive sensors read HIGH when dry and LOW when wet.
// dry value = 0 %, wet value = 100 %
/* TO BE UPDATED VALUES ON CALIBRATION */
const int M1_DRY = 2800, M1_WET = 1200;
const int M2_DRY = 2800, M2_WET = 1200;

// ---- Watering rules and timings (Section 10) ----
// UL = Time in secondds(s), microseconds (MS)
const float    PUMP_START_MLEVEL   = 35.0;               // Pump starts when 35% moisture
const float    PUMP_STOP_MLEVEL   = 55.0;                // Pump stops at 55% moisture
const uint32_t READ_INTERVAL_MS  = 10UL * 1000;          // idle read interval
const uint32_t PUMP_READ_MS      = 2UL * 1000;           // read interval while pumping
const uint32_t MAX_CYCLE_S       = 60;                   // max per cycle
const uint32_t MIN_GAP_MS        = 30UL * 60 * 1000;     // min gap between cycles
const uint32_t DAILY_CAP_S       = 10UL * 60;            // total pump time per day
const uint32_t STATUS_INTERVAL_MS = 60UL * 1000;         // heartbeat to #2
const uint32_t LEVEL_INTERVAL_MS = 10UL * 1000;          // reservoir check
const uint32_t POST_CHECK_MS     = 5UL * 60 * 1000;      // settled moisture after a cycle

// ---- Reservoir probe ----
const bool LEVEL_PROBE_FITTED   = true; // Set false if no level probe is fitted (system then assumes water present).

// Probe signal raw reading ABOVE this = water present.
// Assumes the signal pin is pulled to GND (e.g. 10k to GND) and the probe
// connects GPIO27 to GPIO34 through the water. Check in CAL_MODE.
const int  LEVEL_WET_THRESHOLD  = 1500; // Assumed max Level of Water
const uint8_t EMPTY_STREAK_LIMIT = 2;   // consecutive empty reads before "LOW WATER"

// ---- Sensor sanity limits (raw) ----
const int RAW_MIN_OK = 100;     // below = shorted / unplugged
const int RAW_MAX_OK = 4000;    // above = floating / unplugged

// ---- Watchdog ----
const uint32_t WDT_TIMEOUT_S = 15;  // If system freezes for 15s reset loop

// ---- OLED ---- CHANGE 0x3c to 0x3c if OLED BLANK
const uint8_t OLED_ADDR = 0x3C;               //address where OLED data is stored 
Adafruit_SSD1306 display(128, 64, &Wire, -1); //
bool oledOk = false;

// ============================== STATE =====================================

Preferences prefs;

float    m1Pct = NAN, m2Pct = NAN, avgPct = NAN;
int      m1Raw = 0,   m2Raw = 0;
bool     m1Ok = false, m2Ok = false;
bool     sensorFault = false;           // true when BOTH sensors are bad
bool     reservoirHasWater = true;
bool     lowWater = false;
uint8_t  emptyStreak = 0;

bool     pumpOn = false;
uint32_t pumpStartMs = 0;
float    moistBefore = NAN;
uint32_t lastCycleEndMs = 0;
bool     hasCycled = false;

uint32_t pumpSecToday = 0;
uint16_t cyclesToday  = 0;
uint32_t secIntoDay   = 0;              // rolling 24 h clock (no RTC on this board)

uint32_t lastReadMs = 0, lastLevelMs = 0, lastStatusMs = 0, lastTickMs = 0;
bool     postCheckPending = false;
uint32_t postCheckAtMs = 0;

String   rxLine;

// ============================== HELPERS ===================================

void relayOff() { digitalWrite(PIN_RELAY, LOW); }
void relayOn()  { digitalWrite(PIN_RELAY, HIGH); }

int readAveraged(uint8_t pin, uint8_t samples = 10) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return sum / samples;
}

float rawToPercent(int raw, int dry, int wet) {
  float pct = (float)(dry - raw) * 100.0f / (float)(dry - wet);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

bool rawIsSane(int raw) { return raw > RAW_MIN_OK && raw < RAW_MAX_OK; }

// Powers the probe for ~200 ms only, so it cannot corrode.
bool readReservoir() {
  if (!LEVEL_PROBE_FITTED) return true;
  digitalWrite(PIN_LEVEL_PWR, HIGH);
  delay(200);
  int raw = analogRead(PIN_LEVEL_SIG);
  digitalWrite(PIN_LEVEL_PWR, LOW);
  if (CAL_MODE) { Serial.printf("level raw = %d\n", raw); }
  return raw > LEVEL_WET_THRESHOLD;
}

void saveCounters() {
  prefs.putUInt("pumpSec", pumpSecToday);
  prefs.putUShort("cycles", cyclesToday);
  prefs.putUInt("dayClock", secIntoDay);
}

void newDay() {
  pumpSecToday = 0;
  cyclesToday = 0;
  secIntoDay = 0;
  saveCounters();
}

// Float -> JSON-safe number (NaN becomes -1)
float jf(float v) { return isnan(v) ? -1.0f : v; }

// ============================== SENSING ===================================

void readSoil() {
  m1Raw = readAveraged(PIN_M1);
  m2Raw = readAveraged(PIN_M2);
  m1Ok = rawIsSane(m1Raw);
  m2Ok = rawIsSane(m2Raw);
  m1Pct = m1Ok ? rawToPercent(m1Raw, M1_DRY, M1_WET) : NAN;
  m2Pct = m2Ok ? rawToPercent(m2Raw, M2_DRY, M2_WET) : NAN;

  if (m1Ok && m2Ok)      avgPct = (m1Pct + m2Pct) / 2.0f;
  else if (m1Ok)         avgPct = m1Pct;     // one probe left: use it
  else if (m2Ok)         avgPct = m2Pct;
  else                   avgPct = NAN;

  sensorFault = !(m1Ok || m2Ok);
}

void checkReservoir() {
  reservoirHasWater = readReservoir();
  if (reservoirHasWater) { emptyStreak = 0; lowWater = false; }
  else {
    if (emptyStreak < 255) emptyStreak++;
    if (emptyStreak >= EMPTY_STREAK_LIMIT) lowWater = true;
  }
  lastLevelMs = millis();
}

// ============================== UART TO #2 ================================

void sendStatus() {
  char buf[256];
  uint32_t capLeft = (pumpSecToday >= DAILY_CAP_S) ? 0 : (DAILY_CAP_S - pumpSecToday);
  snprintf(buf, sizeof(buf),
    "{\"node\":1,\"type\":\"status\",\"up\":%lu,\"m1\":%.1f,\"m2\":%.1f,\"avg\":%.1f,"
    "\"m1raw\":%d,\"m2raw\":%d,\"water\":%d,\"low\":%d,\"pump\":%d,\"fault\":%d,"
    "\"cycles\":%u,\"pumpSec\":%lu,\"capLeft\":%lu}",
    (unsigned long)(millis() / 1000), jf(m1Pct), jf(m2Pct), jf(avgPct),
    m1Raw, m2Raw, reservoirHasWater ? 1 : 0, lowWater ? 1 : 0, pumpOn ? 1 : 0,
    sensorFault ? 1 : 0, (unsigned)cyclesToday,
    (unsigned long)pumpSecToday, (unsigned long)capLeft);
  Serial2.println(buf);
  Serial.println(buf);
  lastStatusMs = millis();
}

void sendCycleEvent(uint32_t durS, float after, const char* reason) {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"node\":1,\"type\":\"cycle\",\"up\":%lu,\"dur\":%lu,\"before\":%.1f,\"after\":%.1f,"
    "\"reason\":\"%s\",\"cycles\":%u,\"pumpSec\":%lu}",
    (unsigned long)(millis() / 1000), (unsigned long)durS,
    jf(moistBefore), jf(after), reason,
    (unsigned)cyclesToday, (unsigned long)pumpSecToday);
  Serial2.println(buf);
  Serial.println(buf);
}

void sendPostCheck() {
  char buf[160];
  snprintf(buf, sizeof(buf),
    "{\"node\":1,\"type\":\"settled\",\"up\":%lu,\"avg\":%.1f}",
    (unsigned long)(millis() / 1000), jf(avgPct));
  Serial2.println(buf);
  Serial.println(buf);
}

// Commands from #2 (one per line). "DAY" = start a new day (midnight sync).
void handleLine(const String& line) {
  if (line.indexOf("DAY") >= 0) {
    newDay();
    Serial.println("Daily counters reset by #2");
  }
}

void pollSerial2() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') { handleLine(rxLine); rxLine = ""; }
    else if (c != '\r' && rxLine.length() < 80) rxLine += c;
  }
}

// ============================== PUMP CONTROL ==============================

void startPump() {
  moistBefore = avgPct;
  pumpStartMs = millis();
  pumpOn = true;
  cyclesToday++;
  relayOn();
  Serial.printf("PUMP ON  (avg %.1f %%)\n", jf(avgPct));
}

void stopPump(const char* reason) {
  relayOff();
  pumpOn = false;
  uint32_t durS = (millis() - pumpStartMs) / 1000;
  pumpSecToday += durS;
  lastCycleEndMs = millis();
  hasCycled = true;
  saveCounters();
  Serial.printf("PUMP OFF (%s, %lu s)\n", reason, (unsigned long)durS);
  sendCycleEvent(durS, avgPct, reason);
  postCheckPending = true;
  postCheckAtMs = millis() + POST_CHECK_MS;
}

void controlLogic() {
  if (CAL_MODE) { relayOff(); return; }

  uint32_t now = millis();

  if (pumpOn) {
    uint32_t elapsedS = (now - pumpStartMs) / 1000;
    if (lowWater)                                      stopPump("low_water");
    else if (sensorFault)                              stopPump("sensor_fault");
    else if (avgPct >= PUMP_STOP_MLEVEL)                 stopPump("moist_ok");
    else if (elapsedS >= MAX_CYCLE_S)                  stopPump("max_cycle");
    else if (pumpSecToday + elapsedS >= DAILY_CAP_S)   stopPump("daily_cap");
    return;
  }

  // Pump is off: decide whether to start
  if (lowWater || sensorFault) return;
  if (pumpSecToday >= DAILY_CAP_S) return;
  if (hasCycled && (now - lastCycleEndMs) < MIN_GAP_MS) return;
  if (avgPct < PUMP_START_MLEVEL) startPump();
}

// ============================== OLED ======================================

void drawOled() {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("IRRIGATION #1");
  display.setCursor(0, 12);
  display.printf("M1 %5.1f%%  M2 %5.1f%%", jf(m1Pct), jf(m2Pct));
  display.setCursor(0, 24);
  display.printf("Avg %5.1f%%", jf(avgPct));
  display.setCursor(0, 36);
  display.printf("Pump:%s Water:%s", pumpOn ? "ON " : "off", lowWater ? "LOW" : "ok");
  display.setCursor(0, 48);
  display.printf("Today %lus/%lus c%u", (unsigned long)pumpSecToday,
                 (unsigned long)DAILY_CAP_S, (unsigned)cyclesToday);
  if (sensorFault) { display.setCursor(96, 0); display.print("FAULT"); }
  display.display();
}

// ============================== SETUP / LOOP ==============================

void setup() {
  // Pump OFF before anything else
  pinMode(PIN_RELAY, OUTPUT);
  relayOff();
  pinMode(PIN_LEVEL_PWR, OUTPUT);
  digitalWrite(PIN_LEVEL_PWR, LOW);

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  // Radios off: this board never waits on a network, and it saves power
  WiFi.mode(WIFI_OFF);
  btStop();

  // ADC1 only, full 0-3.3 V range
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_M1, ADC_11db);
  analogSetPinAttenuation(PIN_M2, ADC_11db);
  analogSetPinAttenuation(PIN_LEVEL_SIG, ADC_11db);

  // OLED
  Wire.begin(PIN_SDA, PIN_SCL);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(CAL_MODE ? "CALIBRATION MODE" : "Irrigation #1 boot");
    display.display();
  }

  // Counters that survive a reboot
  prefs.begin("irrig", false);
  pumpSecToday = prefs.getUInt("pumpSec", 0);
  cyclesToday  = prefs.getUShort("cycles", 0);
  secIntoDay   = prefs.getUInt("dayClock", 0);

  // Watchdog (works on Arduino-ESP32 core 2.x and 3.x)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg;
  wdtCfg.timeout_ms = WDT_TIMEOUT_S * 1000;
  wdtCfg.idle_core_mask = 0;
  wdtCfg.trigger_panic = true;
  esp_task_wdt_reconfigure(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);

  // First readings
  readSoil();
  checkReservoir();
  checkReservoir();   // second read so a dry probe is confirmed straight away
  drawOled();
  sendStatus();

  lastReadMs = lastTickMs = millis();
  Serial.println("Irrigation node ready");
}

void loop() {
  esp_task_wdt_reset();
  uint32_t now = millis();

  pollSerial2();

  // 1-second tick: rolling day clock and OLED refresh
  if (now - lastTickMs >= 1000) {
    lastTickMs += 1000;
    secIntoDay++;
    if (secIntoDay >= 86400UL) newDay();
    else if (secIntoDay % 600 == 0) prefs.putUInt("dayClock", secIntoDay);
    drawOled();
  }

  // Reservoir check every 10 s
  if (now - lastLevelMs >= LEVEL_INTERVAL_MS) checkReservoir();

  // Soil reading + control decision
  uint32_t interval = pumpOn ? PUMP_READ_MS : READ_INTERVAL_MS;
  if (now - lastReadMs >= interval) {
    lastReadMs = now;
    readSoil();
    if (CAL_MODE) {
      Serial.printf("M1 raw=%d (%.1f%%)  M2 raw=%d (%.1f%%)\n",
                    m1Raw, jf(m1Pct), m2Raw, jf(m2Pct));
    }
    controlLogic();
  }

  // Heartbeat / status to ESP32 #2 every 60 s
  if (now - lastStatusMs >= STATUS_INTERVAL_MS) sendStatus();

  // Settled moisture reading a few minutes after a cycle
  if (postCheckPending && (int32_t)(now - postCheckAtMs) >= 0) {
    postCheckPending = false;
    sendPostCheck();
  }

  delay(10);
}
