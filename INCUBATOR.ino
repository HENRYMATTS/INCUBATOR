#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Encoder.h>
#include <U8x8lib.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
const uint8_t ac_sense         = A0;
const uint8_t battery_voltage  = A1;

const uint8_t clk              = 2;
const uint8_t Dt               = 3;
const uint8_t sw               = 4;

const uint8_t humidifier       = 7;
const uint8_t relay            = 8;
const uint8_t heater_fan       = 9;
const uint8_t heater           = 10;
const uint8_t humidifier_fan   = 11;
const uint8_t buzzer           = A2;

// ============================================================
// VOLTAGE DIVIDER
// ============================================================
const float   DIVIDER_MULTIPLIER    = 15.7059f;
const float   ADC_REFERENCE_VOLTAGE = 1.1f;
const uint8_t ADC_AVG_SAMPLES       = 32;

float MULTIMETER_MEASURED_VOLTAGE = 11.4f;
float VOLTAGE_CORRECTION_FACTOR   = 1.0f;
float ADC_MEASURED_VOLTAGE        = 0.0f;

const float RAIL_PRESENT_THRESHOLD_V = 3.5f;

// ============================================================
// SENSOR STATE
// ============================================================
float    g_tempC     = NAN;
float    g_humPct    = NAN;
bool     g_sensorOk  = false;
float    g_batteryV  = NAN;

// ============================================================
// HEATER BANG-BANG CONTROL
// ============================================================
const float   TEMP_SETPOINT_C   = 37.5f;
const float   TEMP_CUTOFF_C     = 40.0f;
const float   TEMP_RECOVER_C    = 36.0f;
const float   TEMP_HYST_C       = 0.5f;

const uint8_t FAN_PWM_MIN       = 30;
const uint8_t FAN_PWM_MAX       = 38;

const uint32_t HEATER_WARMUP_MS = 3000;

bool     heaterOn       = false;
bool     heaterWarmedUp = false;
bool     inCooldown     = false;

// ============================================================
// SETPOINT / UI
// ============================================================
const uint8_t EEPROM_ADDR_SETPOINT_RH  = 0;
const uint8_t EEPROM_ADDR_RESET_COUNT  = 1;   // [NEW]
const uint8_t EEPROM_ADDR_HEALTHY_FLAG = 2;   // [NEW]
const uint8_t EEPROM_ADDR_SHT_SETUP_FLAG = 3;

const int     SETPOINT_RH_MIN         = 0;
const int     SETPOINT_RH_MAX         = 100;
const int     SETPOINT_RH_DEFAULT     = 55;

const uint32_t SENSOR_PERIOD_MS   = 2000;
const uint32_t DISPLAY_PERIOD_MS  = 500;
const uint32_t BUTTON_DEBOUNCE_MS = 40;
const uint32_t SAVED_FLASH_MS     = 800;


// SHT setup-flag phase timer
const uint32_t SHT_FLAG_PHASE1_MS = 50UL * 60UL * 1000UL;  // 50 min @ flag=1
const uint32_t SHT_FLAG_PHASE2_MS = 15UL * 60UL * 1000UL;  // 15 min @ flag=0

uint8_t  setupFlagCurrent  = 1;    // cached RAM copy
uint32_t phaseTimerStartMs = 0;

// ============================================================
// WATCHDOG RESET COUNTER   [NEW]
// ============================================================
const uint8_t  WDT_CRASH_THRESHOLD = 3;
const uint32_t HEALTHY_TIME_MS     = 600000UL;   // 10 minutes

uint8_t  resetCountCurrent = 0;
bool     healthyMarked     = false;
uint32_t healthyStartMs    = 0;

const float HUM_HYST_PCT = 1.5f;
bool humidifierOn = false;

enum UiState : uint8_t { UI_IDLE, UI_EDITING, UI_SAVED_FLASH };

int      setpointRH      = SETPOINT_RH_DEFAULT;
int      editRH          = SETPOINT_RH_DEFAULT;
UiState  uiState         = UI_IDLE;
uint32_t savedFlashUntil = 0;
long     lastEncoderPos  = 0;

bool     buttonStable     = HIGH;
bool     buttonLastRead   = HIGH;
uint32_t buttonLastChange = 0;

bool     forceDisplayRefresh = false;

uint32_t g_lastSensorRead  = 0;
uint32_t g_lastDisplayUpd  = 0;

// ============================================================
// HEARTBEAT
// ============================================================
uint8_t  heartbeatIdx     = 0;
uint32_t heartbeatLastMs  = 0;
const uint32_t HEARTBEAT_PERIOD_MS = 250;

// ============================================================
// HUMIDIFIER FAN CYCLE
// ============================================================
const uint32_t HUMIDIFIER_FAN_WAIT_MS = 6500;
const uint32_t HUMIDIFIER_FAN_ON_MS   = 1800;

enum HumidFanState : uint8_t { HF_IDLE, HF_WAIT, HF_ON };
HumidFanState humidFanState       = HF_IDLE;
uint32_t      humidFanPhaseStart  = 0;
bool          humidifierFanOn     = false;

// ============================================================
// HEATER FAN PWM TRACKING
// ============================================================
const uint8_t PWM_VALUE       = 65;
uint8_t       heaterFanPWM    = 0;

// ============================================================
// SHT31 PERIODIC SELF-HEAT (23-hour cycle)
// ============================================================
const uint32_t SHT_HEAT_INTERVAL_MS = 23UL * 3600UL * 1000UL;
const uint32_t SHT_HEAT_DURATION_MS = 5000;
const uint32_t SHT_COOL_DURATION_MS = 60000;
const uint32_t SHT_SETUP_COOL_MS    = 60000;

enum ShtHeaterState : uint8_t { SH_IDLE, SH_HEATING, SH_COOLING };
ShtHeaterState shtHeaterState    = SH_IDLE;
uint32_t       shtPhaseStartMs   = 0;
uint32_t       shtLastCompleteMs = 0;

// ============================================================
// OVER-HUMIDITY DETECTION
// ============================================================
const float    OVER_HUM_TRIGGER_OFFSET = 10.0f;
const float    OVER_HUM_RECOVER_OFFSET = 3.0f;
const uint32_t OVER_HUM_TRIGGER_MS     = 30000UL;

bool     overHumLatched    = false;
bool     overHumRecovered  = false;
uint32_t overHumTimerStart = 0;

// ============================================================
// OVER-TEMPERATURE DETECTION
// ============================================================
bool     overTempLatched = false;
bool     overTempActive  = false;
const uint8_t FAN_PWM_OVERTEMP = 255;

// ============================================================
// OBJECTS
// ============================================================
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Encoder knob(clk, Dt);
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

// ============================================================
// SAFE DELAY — feeds WDT during long waits
// ============================================================
void safeDelay(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    wdt_reset();
    delay(50);
  }
}

// ============================================================
// ADC HELPERS
// ============================================================
float readADC(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_AVG_SAMPLES; i++) sum += analogRead(pin);
  uint16_t raw = (uint16_t)(sum / ADC_AVG_SAMPLES);
  return ((raw / 1023.0f) * ADC_REFERENCE_VOLTAGE) * DIVIDER_MULTIPLIER;
}

float calculateVoltageCorrectionFactor() {
  ADC_MEASURED_VOLTAGE = readADC(ac_sense);
  if (ADC_MEASURED_VOLTAGE < 0.10f) return 1.0f;
  float factor = MULTIMETER_MEASURED_VOLTAGE / ADC_MEASURED_VOLTAGE;
  if (factor < 0.80f || factor > 1.20f) {
    Serial.println(F("WARN: correction factor out of range"));
  }
  return factor;
}

// ============================================================
// SENSOR
// ============================================================
bool readSensors() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if (isnan(t) || isnan(h)) { g_sensorOk = false; return false; }
  g_tempC = t; g_humPct = h; g_sensorOk = true;
  return true;
}

// ============================================================
// UI STATE MACHINE
// ============================================================
void enterEditMode() {
  uiState = UI_EDITING;
  editRH = setpointRH;
  knob.write(0);
  lastEncoderPos = 0;
  forceDisplayRefresh = true;
}

void saveAndExitEdit() {
  setpointRH = editRH;
  EEPROM.update(EEPROM_ADDR_SETPOINT_RH, (uint8_t)setpointRH);
  uiState = UI_SAVED_FLASH;
  savedFlashUntil = millis() + SAVED_FLASH_MS;
  forceDisplayRefresh = true;
}

void handleButtonPress() {
  if (uiState == UI_IDLE) {
    enterEditMode();
  } else if (uiState == UI_EDITING) {
    saveAndExitEdit();
  }
}

void handleButton() {
  bool reading = digitalRead(sw);
  if (reading != buttonLastRead) {
    buttonLastChange = millis();
    buttonLastRead = reading;
  }
  if ((millis() - buttonLastChange) > BUTTON_DEBOUNCE_MS) {
    if (reading != buttonStable) {
      buttonStable = reading;
      if (buttonStable == LOW) handleButtonPress();
    }
  }
}

void handleEncoder() {
  if (uiState != UI_EDITING) return;
  long newPos = knob.read() / 4;
  if (newPos != lastEncoderPos) {
    int diff = (int)(newPos - lastEncoderPos);
    editRH += diff;
    if (editRH < SETPOINT_RH_MIN) editRH = SETPOINT_RH_MIN;
    if (editRH > SETPOINT_RH_MAX) editRH = SETPOINT_RH_MAX;
    lastEncoderPos = newPos;
    forceDisplayRefresh = true;
  }
}

// ============================================================
// DISPLAY — single source of truth
// ============================================================
void displayReadings() {
  u8x8.setFont(u8x8_font_7x14B_1x2_r);
  if (uiState == UI_EDITING) {
    u8x8.drawString(0, 0, "  SET HUMIDITY  ");
  } else {
    u8x8.drawString(0, 0, "   INCUBATOR    ");
  }

  u8x8.setFont(u8x8_font_chroma48medium8_r);

  // [NEW] Crash-loop indicator @ at top-right corner
  u8x8.drawString(15, 0, (resetCountCurrent >= WDT_CRASH_THRESHOLD) ? "@" : " ");

  // Temp row
  u8x8.drawString(1, 2, "Temp:           ");
  if (g_sensorOk) {
    u8x8.setCursor(7, 2);
    u8x8.print(g_tempC, 2);
    u8x8.print(" C");
  } else {
    u8x8.drawString(7, 2, "---.-- C");
  }
  u8x8.drawString(15, 2, heaterOn ? "*" : " ");

  // Hum row
  u8x8.drawString(1, 3, "Hum:            ");
  if (g_sensorOk) {
    u8x8.setCursor(7, 3);
    u8x8.print(g_humPct, 2);
    u8x8.print(" %");
  } else {
    u8x8.drawString(7, 3, "---.-- %");
  }
  u8x8.drawString(15, 3, humidifierOn ? "o" : " ");

  // Setpoint row
  char buf[17];
  if (uiState == UI_EDITING) {
    snprintf(buf, sizeof(buf), ">Set: %3d %% <", editRH);
  } else {
    snprintf(buf, sizeof(buf), " Set: %3d %%  ", setpointRH);
  }
  u8x8.drawString(0, 4, buf);

  // Status row
  if (shtHeaterState != SH_IDLE) {
    u8x8.drawString(1, 5, "Sensor: CAL     ");
  } else if (uiState == UI_EDITING) {
    u8x8.drawString(0, 5, "Turn, then press");
  } else if (uiState == UI_SAVED_FLASH) {
    u8x8.drawString(0, 5, "    SAVED!      ");
  } else if (!g_sensorOk) {
    u8x8.drawString(1, 5, "Sensor: ERROR   ");
  } else if (digitalRead(relay) == LOW) {
    u8x8.drawString(0, 5, "Bat:            ");
    u8x8.setCursor(5, 5);
    if (isnan(g_batteryV)) {
      u8x8.print("---.--");
    } else {
      u8x8.print(g_batteryV, 2);
    }
    u8x8.print("V");
  } else {
    u8x8.drawString(1, 5, "Sensor: OK      ");
  }

  // Bottom row
  const char* hfText;
  if (overHumLatched) {
    hfText = "OH";
  } else {
    hfText = humidifierFanOn ? "ON" : "OFF";
  }

  char fanBuf[17];
  if (overTempLatched) {
    snprintf(fanBuf, sizeof(fanBuf), "TF=OT  HF=%-3s  ", hfText);
  } else {
    snprintf(fanBuf, sizeof(fanBuf), "TF=%03d HF=%-3s  ", heaterFanPWM, hfText);
  }
  u8x8.drawString(0, 7, fanBuf);
}

// ============================================================
// HUMIDIFIER CONTROL
// ============================================================
void handleHumidifier() {
  if (!g_sensorOk) {
    if (humidifierOn) {
      humidifierOn = false;
      digitalWrite(humidifier, LOW);
      forceDisplayRefresh = true;
    }
    return;
  }

  if (!humidifierOn && g_humPct <= (setpointRH - HUM_HYST_PCT)) {
    humidifierOn = true;
    digitalWrite(humidifier, HIGH);
    forceDisplayRefresh = true;
  }
  else if (humidifierOn && g_humPct >= setpointRH) {
    humidifierOn = false;
    digitalWrite(humidifier, LOW);
    forceDisplayRefresh = true;
  }
}

// ============================================================
// HUMIDIFIER FAN CYCLE
// ============================================================
void handleHumidifierFan(uint32_t now) {
  if (overHumLatched && !overHumRecovered) {
    if (humidifierFanOn) {
      digitalWrite(humidifier_fan, LOW);
      humidifierFanOn = false;
      forceDisplayRefresh = true;
    }
    humidFanState = HF_IDLE;
    return;
  }

  if (!humidifierOn) {
    if (humidifierFanOn) {
      digitalWrite(humidifier_fan, LOW);
      humidifierFanOn = false;
      forceDisplayRefresh = true;
    }
    humidFanState = HF_IDLE;
    return;
  }

  switch (humidFanState) {
    case HF_IDLE:
      humidFanState      = HF_WAIT;
      humidFanPhaseStart = now;
      if (humidifierFanOn) {
        digitalWrite(humidifier_fan, LOW);
        humidifierFanOn = false;
        forceDisplayRefresh = true;
      }
      break;

    case HF_WAIT:
      if (now - humidFanPhaseStart >= HUMIDIFIER_FAN_WAIT_MS) {
        humidFanState      = HF_ON;
        humidFanPhaseStart = now;
        digitalWrite(humidifier_fan, HIGH);
        humidifierFanOn = true;
        forceDisplayRefresh = true;
      }
      break;

    case HF_ON:
      if (now - humidFanPhaseStart >= HUMIDIFIER_FAN_ON_MS) {
        humidFanState      = HF_WAIT;
        humidFanPhaseStart = now;
        digitalWrite(humidifier_fan, LOW);
        humidifierFanOn = false;
        forceDisplayRefresh = true;
      }
      break;
  }
}

// ============================================================
// OVER-HUMIDITY DETECTION
// ============================================================
void handleOverHumidity(uint32_t now) {
  if (!g_sensorOk) return;

  if (!overHumLatched) {
    if (g_humPct >= (setpointRH + OVER_HUM_TRIGGER_OFFSET)) {
      if (overHumTimerStart == 0) overHumTimerStart = now;
      if (now - overHumTimerStart >= OVER_HUM_TRIGGER_MS) {
        overHumLatched = true;
        Serial.println(F("ALARM: Over-humidity — humidifier stuck?"));
        forceDisplayRefresh = true;
      }
    } else {
      overHumTimerStart = 0;
    }
  }

  if (overHumLatched && !overHumRecovered) {
    if (g_humPct <= (setpointRH + OVER_HUM_RECOVER_OFFSET)) {
      overHumRecovered = true;
      Serial.println(F("INFO: Over-humidity recovered — fan allowed"));
      forceDisplayRefresh = true;
    }
  }
}

// ============================================================
// OVER-TEMPERATURE DETECTION
// ============================================================
void handleOverTemp() {
  if (!g_sensorOk) return;

  if (!overTempActive && g_tempC >= TEMP_CUTOFF_C) {
    overTempActive  = true;
    overTempLatched = true;

    analogWrite(heater_fan, FAN_PWM_OVERTEMP);
    heaterFanPWM = FAN_PWM_OVERTEMP;

    if (heaterOn) {
      heaterOn = false;
      digitalWrite(heater, LOW);
    }

    Serial.print(F("ALARM: Over-temperature — "));
    Serial.println(g_tempC, 2);
    forceDisplayRefresh = true;
  }
  else if (overTempActive && g_tempC < TEMP_SETPOINT_C) {
    overTempActive = false;

    analogWrite(heater_fan, PWM_VALUE);
    heaterFanPWM = PWM_VALUE;

    Serial.println(F("INFO: Over-temperature recovered — normal resume"));
    forceDisplayRefresh = true;
  }
}

// ============================================================
// HEARTBEAT
// ============================================================
void handleHeartbeat(uint32_t now) {
  if (now - heartbeatLastMs < HEARTBEAT_PERIOD_MS) return;
  heartbeatLastMs = now;
  static const char spins[] = {'|', '/', '-', '\\'};
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  char c[2] = { spins[heartbeatIdx], 0 };
  u8x8.drawString(15, 7, c);
  heartbeatIdx = (heartbeatIdx + 1) & 3;
}

// ============================================================
// HEATER BANG-BANG
// ============================================================
void handleHeater() {
  if (overTempActive) {
    if (heaterOn) {
      heaterOn = false;
      digitalWrite(heater, LOW);
      forceDisplayRefresh = true;
    }
    return;
  }

  if (!g_sensorOk) {
    if (heaterOn) {
      heaterOn = false;
      digitalWrite(heater, LOW);
      forceDisplayRefresh = true;
    }
    return;
  }

  if (!heaterOn && g_tempC <= (TEMP_SETPOINT_C - TEMP_HYST_C)) {
    heaterOn = true;
    digitalWrite(heater, HIGH);
    forceDisplayRefresh = true;
  }
  else if (heaterOn && g_tempC >= TEMP_SETPOINT_C) {
    heaterOn = false;
    digitalWrite(heater, LOW);
    forceDisplayRefresh = true;
  }
}

// ============================================================
// SHT31 PERIODIC SELF-HEAT
// ============================================================
void handleShtPeriodicHeater(uint32_t now) {
  switch (shtHeaterState) {
    case SH_IDLE:
      if (now - shtLastCompleteMs >= SHT_HEAT_INTERVAL_MS) {
        if (heaterOn) {
          heaterOn = false;
          digitalWrite(heater, LOW);
        }
        if (humidifierOn) {
          humidifierOn = false;
          digitalWrite(humidifier, LOW);
        }

        sht31.heater(true);
        shtPhaseStartMs     = now;
        shtHeaterState      = SH_HEATING;
        forceDisplayRefresh = true;
      }
      break;

    case SH_HEATING:
      if (now - shtPhaseStartMs >= SHT_HEAT_DURATION_MS) {
        sht31.heater(false);
        shtPhaseStartMs     = now;
        shtHeaterState      = SH_COOLING;
        forceDisplayRefresh = true;
      }
      break;

    case SH_COOLING:
      if (now - shtPhaseStartMs >= SHT_COOL_DURATION_MS) {
        shtLastCompleteMs   = now;
        shtHeaterState      = SH_IDLE;
        forceDisplayRefresh = true;
      }
      break;
  }
}



// ============================================================
// BUZZER — beeps on sensor failure or over-temp
// ============================================================
void handleBuzzer() {
  bool alarm = (!g_sensorOk) || overTempActive;
  digitalWrite(buzzer, alarm ? HIGH : LOW);
}




// ============================================================
// SETUP
// ============================================================
void setup() {
  analogWrite(heater_fan, 85);
  // --- [NEW] Reset-cause analysis — MUST be first ---
  uint8_t mcusr = MCUSR;
  MCUSR = 0;
  bool wasWdtReset = (mcusr & _BV(WDRF)) != 0;

  wdt_disable();
  Serial.begin(115200);

  // --- [NEW] Reset counter logic ---
  uint8_t storedCount = EEPROM.read(EEPROM_ADDR_RESET_COUNT);
  uint8_t healthyFlag = EEPROM.read(EEPROM_ADDR_HEALTHY_FLAG);
  if (storedCount == 255) storedCount = 0;
  if (healthyFlag  > 1)   healthyFlag  = 0;

  if (wasWdtReset) {
    if (healthyFlag == 1) {
      storedCount = 1;
    } else {
      storedCount++;
    }
    healthyFlag = 0;
  } else {
    storedCount = 0;
    healthyFlag = 0;
  }
  resetCountCurrent = storedCount;
  EEPROM.update(EEPROM_ADDR_RESET_COUNT, storedCount);
  EEPROM.update(EEPROM_ADDR_HEALTHY_FLAG, healthyFlag);

  Serial.print(F("Reset cause: "));
  if (wasWdtReset) Serial.print(F("WATCHDOG, "));
  else if (mcusr & _BV(PORF)) Serial.print(F("POWER-ON, "));
  else if (mcusr & _BV(BORF)) Serial.print(F("BROWNOUT, "));
  else if (mcusr & _BV(EXTRF)) Serial.print(F("EXTERNAL, "));
  Serial.print(F("count=")); Serial.println(resetCountCurrent);

  // --- OLED ---
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clearDisplay();
  u8x8.drawString(0, 0, "Booting...");
  delay(800);
  u8x8.clearDisplay();

  // --- Arm watchdog ---
  wdt_enable(WDTO_8S);
  wdt_reset();

  // --- SHT31 ---
  if (!sht31.begin(0x44)) {
    Serial.println(F("SHT31 not found — check wiring / try 0x45"));
  }
  wdt_reset();

  // --- Load setpoint from EEPROM ---
  uint8_t saved = EEPROM.read(EEPROM_ADDR_SETPOINT_RH);
  if (saved <= SETPOINT_RH_MAX) {
    setpointRH = saved;
  } else {
    setpointRH = SETPOINT_RH_DEFAULT;
    EEPROM.update(EEPROM_ADDR_SETPOINT_RH, (uint8_t)setpointRH);
  }
  editRH = setpointRH;

  
    // --- SHT31 SELF-TEST at boot — conditional on EEPROM flag ---
  setupFlagCurrent = EEPROM.read(EEPROM_ADDR_SHT_SETUP_FLAG);
  if (setupFlagCurrent != 0 && setupFlagCurrent != 1) {
    setupFlagCurrent = 0;   // first boot / corrupt → heat
    EEPROM.update(EEPROM_ADDR_SHT_SETUP_FLAG, 0);
  }

  if (setupFlagCurrent == 0) {
    Serial.println(F("SHT: running setup self-heat (flag=0)"));

    u8x8.clearDisplay();
    u8x8.setFont(u8x8_font_7x14B_1x2_r);
    u8x8.drawString(0, 0, "  SELF-TEST     ");
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.drawString(0, 2, "SHT31 heating");

    sht31.heater(true);
    safeDelay(5000);
    sht31.heater(false);

    u8x8.drawString(0, 2, "SHT31 cooling");
    u8x8.drawString(0, 3, "Stabilizing...");
    safeDelay(SHT_SETUP_COOL_MS);

    wdt_reset();

    setupFlagCurrent = 1;
    EEPROM.update(EEPROM_ADDR_SHT_SETUP_FLAG, 1);
  } else {
    Serial.println(F("SHT: skipping setup self-heat (flag=1)"));
  }


  u8x8.clearDisplay();

  // --- First sensor read + screen ---
  readSensors();
  displayReadings();

  // --- Output pins ---
  pinMode(relay,          OUTPUT);
  pinMode(humidifier,     OUTPUT);
  pinMode(heater_fan,     OUTPUT);
  pinMode(heater,         OUTPUT);
  pinMode(humidifier_fan, OUTPUT);
  pinMode(buzzer,         OUTPUT);

  // --- Encoder pins ---
  pinMode(clk, INPUT_PULLUP);
  pinMode(Dt,  INPUT_PULLUP);
  pinMode(sw,  INPUT_PULLUP);

  digitalWrite(relay,          LOW);
  digitalWrite(humidifier,     LOW);
  digitalWrite(heater_fan,     LOW);
  digitalWrite(heater,         LOW);
  digitalWrite(humidifier_fan, LOW);

  digitalWrite(buzzer,    HIGH);
  delay(1000);
  digitalWrite(buzzer,    LOW);


  // --- Sync button state ---
  buttonStable   = digitalRead(sw);
  buttonLastRead = buttonStable;

  humidifierOn = false;
  humidifierFanOn = false;
  humidFanState   = HF_IDLE;

  overHumLatched    = false;
  overHumRecovered  = false;
  overHumTimerStart = 0;

  overTempLatched = false;
  overTempActive  = false;

  // --- ADC reference + calibration ---
  analogReference(INTERNAL);
  delay(10);
  (void)analogRead(ac_sense);
  VOLTAGE_CORRECTION_FACTOR = calculateVoltageCorrectionFactor();

  delay(1000);
  wdt_reset();

  float adc0_corrected = VOLTAGE_CORRECTION_FACTOR * readADC(ac_sense);

  if (adc0_corrected > RAIL_PRESENT_THRESHOLD_V) {
    digitalWrite(relay, HIGH);
    delay(200);
  }

  if (digitalRead(relay) == LOW) {
    g_batteryV = VOLTAGE_CORRECTION_FACTOR * readADC(battery_voltage);
  }

  
  heaterFanPWM = PWM_VALUE;
  delay(1000);
  wdt_reset();

  shtLastCompleteMs = millis();
  shtHeaterState    = SH_IDLE;
  shtPhaseStartMs   = 0;

  // --- [NEW] Mark start of healthy window ---
  healthyStartMs = millis();
  phaseTimerStartMs = millis();
  analogWrite(heater_fan, PWM_VALUE);

}





// ============================================================
// LOOP
// ============================================================
void loop() {
  wdt_reset();
 


  uint32_t now = millis();

  // --- [NEW] 10-min healthy check ---
  if (!healthyMarked && (now - healthyStartMs >= HEALTHY_TIME_MS)) {
    healthyMarked = true;
    resetCountCurrent = 0;
    EEPROM.update(EEPROM_ADDR_RESET_COUNT, 0);
    EEPROM.update(EEPROM_ADDR_HEALTHY_FLAG, 1);
    forceDisplayRefresh = true;
    Serial.println(F("INFO: 10 min healthy — reset count cleared"));
  }

    // --- SHT setup-flag phase timer (50 min @ 1 / 15 min @ 0) ---
  uint32_t phaseElapsed = now - phaseTimerStartMs;
  if (setupFlagCurrent == 1 && phaseElapsed >= SHT_FLAG_PHASE1_MS) {
    setupFlagCurrent = 0;
    EEPROM.update(EEPROM_ADDR_SHT_SETUP_FLAG, 0);
    phaseTimerStartMs = now;
    Serial.println(F("SHT flag → 0 (next boot will heat)"));
  } else if (setupFlagCurrent == 0 && phaseElapsed >= SHT_FLAG_PHASE2_MS) {
    setupFlagCurrent = 1;
    EEPROM.update(EEPROM_ADDR_SHT_SETUP_FLAG, 1);
    phaseTimerStartMs = now;
    Serial.println(F("SHT flag → 1 (next boot will skip)"));
  }

  // --- UI input ---
  handleButton();
  handleEncoder();

  // --- SAVED flash timeout ---
  if (uiState == UI_SAVED_FLASH && (int32_t)(now - savedFlashUntil) >= 0) {
    uiState = UI_IDLE;
    forceDisplayRefresh = true;
  }

  // --- Sensor read every 2 s ---
  if (now - g_lastSensorRead >= SENSOR_PERIOD_MS) {
    g_lastSensorRead = now;

    if (shtHeaterState == SH_IDLE) {
      readSensors();
      handleOverTemp();
      handleHeater();
      handleHumidifier();
      handleOverHumidity(now);
      if (digitalRead(relay) == LOW) {
        g_batteryV = VOLTAGE_CORRECTION_FACTOR * readADC(battery_voltage);
      }
    }
  }

  handleBuzzer();

  // --- Heartbeat 250 ms ---
  handleHeartbeat(now);

  // --- Humidifier fan cycle ---
  handleHumidifierFan(now);

  // --- SHT31 23h periodic self-heat ---
  handleShtPeriodicHeater(now);

  // --- Display refresh ---
  if (forceDisplayRefresh || (now - g_lastDisplayUpd >= DISPLAY_PERIOD_MS)) {
    forceDisplayRefresh = false;
    g_lastDisplayUpd = now;
    displayReadings();
  }

  // --- Relay check every pass ---
  float adc0_corrected = VOLTAGE_CORRECTION_FACTOR * readADC(ac_sense);
  digitalWrite(relay, (adc0_corrected > RAIL_PRESENT_THRESHOLD_V) ? HIGH : LOW);
}