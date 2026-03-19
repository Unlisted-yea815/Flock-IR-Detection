#include <Arduino.h>
/*
  IR Strobe Detector (XIAO ESP32S3 + BPW34NA + MCP6002). Intended to detect Flock LPR 10hz 850nm IR pulses at night.

  Hardware:
  - Three BPW34NA photodiodes feed MCP6002 op-amps configured as transimpedance amplifiers (each channel uses ~4.7 MΩ feedback in parallel with ~10 pF for stability/noise shaping).
  - Each op-amp output is read by the ESP32-S3 ADC (3.3 V system). In darkness the output sits near a low baseline (~0.2 V); an IR strobe produces upward “towers” toward the top rail.

  Signal processing (per channel, no averaging across sensors):
  - Samples each ADC channel at ~1 kHz.
  - Tracks a slowly-updating baseline using an exponential moving average (EMA) to follow ambient changes without following the fast pulse tops.
  - Computes an AC component = (sample - baseline). When AC exceeds a threshold, it marks a pulse edge. The time between pulse edges is measured and accepted only if it falls in the target band (5–15 Hz; typical camera strobes ~10 Hz).
  - Requires multiple consecutive in-band intervals before asserting “detected” (helps reject random flicker/noise). A channel “unlocks” after a long quiet period.

  Outputs / UX:
  - System LED: indicates the on/off switch state (switch is active-HIGH from 3.3 V).
  - Detection LED: latches briefly (hold time) after a confirmed detection event.
  - Active buzzer: beeps rapidly ONLY while valid in-band pulses are currently present (no holdover; stops quickly when strobes stop).

  Pin naming:
  - Uses XIAO “Dx/Ax” pin names (D0..D10) so code matches the board silkscreen/header labels.
*/

/* =========================================================
   PIN ASSIGNMENTS (XIAO HEADER NAMES)
   ========================================================= */
static const uint8_t N_CH = 3;

static const uint8_t PIN_SENSOR[N_CH] = { D0, D1, D4 }; // Rear1, Rear2, Roof
static const char*   CH_NAME[N_CH]    = { "Rear 1", "Rear 2", "Roof" };

static const uint8_t PIN_SWITCH  = D5;   // active‑HIGH (3.3 V)
static const uint8_t PIN_LED_SYS = D8;
static const uint8_t PIN_LED_DET = D9;
static const uint8_t PIN_BUZZER  = D10;  // active buzzer

/* =========================================================
   DETECTION PARAMETERS
   ========================================================= */
static const uint32_t SAMPLE_PERIOD_US = 1000; // ~1 kHz

// Target 5–15 Hz
static const uint32_t MIN_PERIOD_MS = 67;
static const uint32_t MAX_PERIOD_MS = 200;

// Baseline EMA (1/8)
static const int EMA_NUM = 1;
static const int EMA_DEN = 8;

// Thresholds (12‑bit ADC counts)
static const int THR_IDLE   = 360;
static const int THR_LOCKED = 110;
static const int HYST_AC    = 90;

static const uint32_t REFRACTORY_MS = 35;
static const uint8_t  REQUIRED_VALID_INTERVALS = 2;

static const uint32_t LED_HOLD_MS = 300;

/* =========================================================
   BUZZER BEHAVIOR (active buzzer, NO holdover)
   - Beep rapidly ONLY while valid in-band pulses are present.
   ========================================================= */
static const uint16_t BUZZ_ON_MS  = 30;
static const uint16_t BUZZ_OFF_MS = 70;

// "Active" window: if we saw an in-band interval within this window,
// consider strobes actively present. Use > MAX_PERIOD_MS with margin.
static const uint32_t ACTIVE_WINDOW_MS = 260;

bool     buzzerOn      = false;
uint32_t buzzerNextMs  = 0;

/* =========================================================
   STATE
   ========================================================= */
uint32_t lastSampleUs     = 0;
uint32_t ledHoldUntilMs   = 0;

int      baseline[N_CH];
bool     inPulse[N_CH];
bool     locked[N_CH];
bool     detected[N_CH];
uint8_t  validIntervals[N_CH];
uint32_t lastPulseMs[N_CH];

// NEW: last time we saw a VALID in-band interval (used for buzzer only)
uint32_t lastInBandMs[N_CH];

/* ========================================================= */

inline bool systemEnabled() {
  return digitalRead(PIN_SWITCH) == HIGH;
}

void resetChannel(uint8_t ch, int v) {
  baseline[ch] = v;
  inPulse[ch]  = false;
  locked[ch]   = false;
  detected[ch] = false;
  validIntervals[ch] = 0;
  lastPulseMs[ch]    = 0;
  lastInBandMs[ch]   = 0;  // NEW
}

/* =========================================================
   ACTIVE BUZZER STATE MACHINE
   ========================================================= */
void buzzerUpdate(bool activeNow) {
  uint32_t now = millis();

  if (!activeNow) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerOn = false;
    buzzerNextMs = now;
    return;
  }

  if (now < buzzerNextMs) return;

  if (buzzerOn) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerOn = false;
    buzzerNextMs = now + BUZZ_OFF_MS;
  } else {
    digitalWrite(PIN_BUZZER, HIGH);
    buzzerOn = true;
    buzzerNextMs = now + BUZZ_ON_MS;
  }
}

/* =========================================================
   PER‑CHANNEL PROCESSING
   ========================================================= */
void processChannel(uint8_t ch, int v, uint32_t nowMs) {
  baseline[ch] += (v - baseline[ch]) * EMA_NUM / EMA_DEN;

  int ac  = v - baseline[ch];        // upward pulses
  int thr = locked[ch] ? THR_LOCKED : THR_IDLE;

  if (!inPulse[ch]) {
    if (ac >= thr && nowMs - lastPulseMs[ch] >= REFRACTORY_MS) {

      if (lastPulseMs[ch] != 0) {
        uint32_t period = nowMs - lastPulseMs[ch];
        bool inBand = (period >= MIN_PERIOD_MS && period <= MAX_PERIOD_MS);

        if (inBand) {
          lastInBandMs[ch] = nowMs;          // NEW: mark active pulse train
          validIntervals[ch]++;
          if (!locked[ch]) locked[ch] = true;
        } else {
          validIntervals[ch] = 0;
        }

        detected[ch] = locked[ch] && (validIntervals[ch] >= REQUIRED_VALID_INTERVALS);

        if (detected[ch]) {
          digitalWrite(PIN_LED_DET, HIGH);
          ledHoldUntilMs = nowMs + LED_HOLD_MS;
          Serial.printf("DETECT: %s\n", CH_NAME[ch]);
        }
      }

      lastPulseMs[ch] = nowMs;
      inPulse[ch] = true;
    }
  } else {
    if (ac <= thr - HYST_AC) inPulse[ch] = false;
  }

  // Keep your original "unlock" behavior unchanged
  if (lastPulseMs[ch] && (nowMs - lastPulseMs[ch] > 4000)) {
    locked[ch] = false;
    detected[ch] = false;
    validIntervals[ch] = 0;
  }
}

/* =========================================================
   SETUP
   ========================================================= */
void setup() {
  Serial.begin(115200);
  delay(150);

  pinMode(PIN_LED_SYS, OUTPUT);
  pinMode(PIN_LED_DET, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_SWITCH, INPUT_PULLDOWN);

  digitalWrite(PIN_LED_SYS, LOW);
  digitalWrite(PIN_LED_DET, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  analogReadResolution(12);

  for (uint8_t ch = 0; ch < N_CH; ch++) {
    long sum = 0;
    for (int i = 0; i < 32; i++) {
      sum += analogRead(PIN_SENSOR[ch]);
      delay(2);
    }
    resetChannel(ch, sum / 32);
  }

  Serial.println("XIAO ESP32S3 IR Detector started");
}

/* =========================================================
   LOOP
   ========================================================= */
void loop() {
  uint32_t nowMs = millis();

  bool enabled = systemEnabled();
  digitalWrite(PIN_LED_SYS, enabled ? HIGH : LOW);

  if (!enabled) {
    digitalWrite(PIN_LED_DET, LOW);
    digitalWrite(PIN_BUZZER, LOW);
    buzzerOn = false;
    return;
  }

  uint32_t nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_PERIOD_US) return;
  lastSampleUs = nowUs;

  for (uint8_t ch = 0; ch < N_CH; ch++) {
    int v = analogRead(PIN_SENSOR[ch]);
    processChannel(ch, v, nowMs);
  }

  bool anyDetect = false;
  for (uint8_t ch = 0; ch < N_CH; ch++) anyDetect |= detected[ch];

  // Detection LED holdover (unchanged)
  if (!anyDetect && nowMs >= ledHoldUntilMs) {
    digitalWrite(PIN_LED_DET, LOW);
  }

  // NEW: buzzer depends on "active pulse train", not latched detect
  bool anyActive = false;
  for (uint8_t ch = 0; ch < N_CH; ch++) {
    if (lastInBandMs[ch] && (nowMs - lastInBandMs[ch] <= ACTIVE_WINDOW_MS)) {
      anyActive = true;
      break;
    }
  }

  // Active buzzer — NO holdover
  buzzerUpdate(anyActive);
}