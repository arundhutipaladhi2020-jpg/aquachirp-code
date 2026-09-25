
//  Generic STM32F401CCX6
//  PCF8591
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int potTemp      = PA0;
const int potSalinity  = PA1;
const int potDepth     = PA6;
const int potTurbidity = PA7;
const int potBattery   = PB0;
const int btnWaveform  = PA4;
const int btnWindow    = PA5;

const uint8_t PCF8591_ADDR = 0x48;

const float TEMP_MIN  = 3.0,   TEMP_MAX  = 30.0;
const float SAL_MIN   = 0.0,   SAL_MAX   = 40.0;
const float DEPTH_MIN = 0.0,   DEPTH_MAX = 500.0;
const float TURB_MIN  = 0.0,   TURB_MAX  = 100.0;
const float BATT_MIN  = 0.0,   BATT_MAX  = 100.0;

const float TURB_AMP_MIN = 0.4;
const float TURB_AMP_MAX = 1.0;

const float BATT_LOW_THRESHOLD   = 30.0;
const float BATT_FLOOR_FRACTION  = 0.3;   // amplitude floor at 0% battery
const float DUR_BATT_FLOOR_FRACTION = 0.5; // duration floor at 0% battery
const unsigned long IDLE_PAUSE_MAX_MS = 1000;

const float DELAY_US_CLEAR = 100.0;
const float DELAY_US_MUDDY = 400.0;
const float SWEEP_SPAN_FRACTION = 0.4;

// --- Pulse duration now comes from depth ---
const int PULSE_N_MIN = 80;    // clearer/shallower: shorter pulse
const int PULSE_N_MAX = 320;   // deeper: longer pulse, more total energy
const int PULSE_N_FLOOR = 8;   // safety floor so window math never breaks

const int8_t BARKER13[13] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
const int BARKER_LEN = 13;

enum WaveformType { WF_LFM_CHIRP = 0, WF_GEOMETRIC_SWEEP = 1, WF_PHASE_CODED = 2, WF_COUNT = 3 };
enum WindowType   { WIN_NONE = 0, WIN_HANN = 1, WIN_HAMMING = 2, WIN_BLACKMAN = 3, WIN_COUNT = 4 };

WaveformType currentWaveform = WF_LFM_CHIRP;
WindowType   currentWindow   = WIN_NONE;
bool pcfPresent = true;

bool lastWaveBtnState = false;
bool lastWinBtnState  = false;
unsigned long lastWaveDebounce = 0;
unsigned long lastWinDebounce  = 0;
const unsigned long DEBOUNCE_MS = 30;

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
}
float clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

bool debouncedPress(int pin, bool &lastState, unsigned long &lastDebounce) {
  bool reading = digitalRead(pin);
  bool pressed = false;
  if (reading != lastState && (millis() - lastDebounce) > DEBOUNCE_MS) {
    lastDebounce = millis();
    if (reading == HIGH) pressed = true;
    lastState = reading;
  }
  return pressed;
}

void pcf8591Write(uint8_t value) {
  Wire.beginTransmission(PCF8591_ADDR);
  Wire.write(0x40);
  Wire.write(value);
  Wire.endTransmission();
}

float windowValue(WindowType w, int n, int N) {
  float ratio = (float)n / (float)(N - 1);
  switch (w) {
    case WIN_HANN:     return 0.5f - 0.5f * cos(2.0f * PI * ratio);
    case WIN_HAMMING:  return 0.54f - 0.46f * cos(2.0f * PI * ratio);
    case WIN_BLACKMAN: return 0.42f - 0.5f * cos(2.0f * PI * ratio) + 0.08f * cos(4.0f * PI * ratio);
    case WIN_NONE:
    default:           return 1.0f;
  }
}

float lfmChirpSample(int n, int N, float f0, float f1, float Ts) {
  float t = n * Ts;
  float T = N * Ts;
  float k = (f1 - f0) / T;
  float phase = 2.0f * PI * (f0 * t + 0.5f * k * t * t);
  return cos(phase);
}

float geometricSweepSample(int n, int N, float f0, float f1, float Ts) {
  float t = n * Ts;
  float T = N * Ts;
  float k = log(f1 / f0) / T;
  float phase = 2.0f * PI * (f0 / k) * (exp(k * t) - 1.0f);
  return cos(phase);
}

float phaseCodedSample(int n, int N, float fc, float Ts) {
  float t = n * Ts;
  int chip = (int)(((float)n / (float)N) * BARKER_LEN);
  if (chip >= BARKER_LEN) chip = BARKER_LEN - 1;
  float phaseShift = (BARKER13[chip] > 0) ? 0.0f : PI;
  float phase = 2.0f * PI * fc * t + phaseShift;
  return cos(phase);
}

const char* waveformName(WaveformType w) {
  switch (w) {
    case WF_LFM_CHIRP:       return "LFM Chirp";
    case WF_GEOMETRIC_SWEEP: return "GeoSweep";
    case WF_PHASE_CODED:     return "PhaseCode";
  }
  return "?";
}
const char* windowName(WindowType w) {
  switch (w) {
    case WIN_NONE:     return "None";
    case WIN_HANN:      return "Hann";
    case WIN_HAMMING:   return "Hamming";
    case WIN_BLACKMAN:  return "Blackman";
  }
  return "?";
}

HardwareTimer *sampleTimer = new HardwareTimer(TIM2);
volatile bool sampleTick = false;
void onSampleTick() { sampleTick = true; }

bool transmitting = false;
int sampleIndex = 0;
int pulseN = PULSE_N_MIN;
float pulseF0, pulseF1, pulseBaseFreq, pulseTs, pulseAmpScale;
WaveformType pulseWaveform;
WindowType pulseWindow;

void startPulse(float f0, float f1, float baseFreq, float Ts, float ampScale, int nSamples, WaveformType wf, WindowType win) {
  pulseF0 = f0; pulseF1 = f1; pulseBaseFreq = baseFreq; pulseTs = Ts;
  pulseAmpScale = ampScale; pulseN = nSamples; pulseWaveform = wf; pulseWindow = win;
  sampleIndex = 0;
  transmitting = true;
  sampleTimer->setOverflow((uint32_t)(1.0f / Ts), HERTZ_FORMAT);
  sampleTimer->refresh();
  sampleTimer->resume();
}

void stopPulse() {
  sampleTimer->pause();
  transmitting = false;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(btnWaveform, INPUT);
  pinMode(btnWindow, INPUT);
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found at 0x3C");
    while (1);
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  Wire.beginTransmission(PCF8591_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("Warning: PCF8591 not responding at 0x48 - check wiring/address");
    pcfPresent = false;
  }

  sampleTimer->attachInterrupt(onSampleTick);
  sampleTimer->pause();
}

void loop() {
  if (debouncedPress(btnWaveform, lastWaveBtnState, lastWaveDebounce)) {
    currentWaveform = (WaveformType)((currentWaveform + 1) % WF_COUNT);
  }
  if (debouncedPress(btnWindow, lastWinBtnState, lastWinDebounce)) {
    currentWindow = (WindowType)((currentWindow + 1) % WIN_COUNT);
  }

  if (transmitting) {
    if (!sampleTick) {
      __WFI();
      return;
    }
    sampleTick = false;

    float raw;
    switch (pulseWaveform) {
      case WF_LFM_CHIRP:       raw = lfmChirpSample(sampleIndex, pulseN, pulseF0, pulseF1, pulseTs); break;
      case WF_GEOMETRIC_SWEEP: raw = geometricSweepSample(sampleIndex, pulseN, pulseF0, pulseF1, pulseTs); break;
      case WF_PHASE_CODED:
      default:                 raw = phaseCodedSample(sampleIndex, pulseN, pulseBaseFreq, pulseTs); break;
    }
    float win = windowValue(pulseWindow, sampleIndex, pulseN);
    float shaped = raw * win * pulseAmpScale;
    uint8_t dacValue = (uint8_t)(127.5f + 127.5f * shaped);
    if (pcfPresent) pcf8591Write(dacValue);

    sampleIndex++;
    if (sampleIndex >= pulseN) stopPulse();
    return;
  }

  // --- Not transmitting: read all five inputs, compute the next pulse ---
  int rawTemp = analogRead(potTemp);
  int rawSal  = analogRead(potSalinity);
  int rawDep  = analogRead(potDepth);
  int rawTurb = analogRead(potTurbidity);
  int rawBatt = analogRead(potBattery);

  float temperature = mapFloat(rawTemp, 0, 4095, TEMP_MIN,  TEMP_MAX);
  float salinity    = mapFloat(rawSal,  0, 4095, SAL_MIN,   SAL_MAX);
  float depth       = mapFloat(rawDep,  0, 4095, DEPTH_MIN, DEPTH_MAX);
  float turbidity   = mapFloat(rawTurb, 0, 4095, TURB_MIN,  TURB_MAX);
  float battery     = mapFloat(rawBatt, 0, 4095, BATT_MIN,  BATT_MAX);

  float turbFraction  = clamp01((turbidity - TURB_MIN) / (TURB_MAX - TURB_MIN));
  float depthFraction = clamp01((depth - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN));
  float salFraction   = clamp01((salinity - SAL_MIN) / (SAL_MAX - SAL_MIN));
  float tempColdness  = clamp01(1.0f - (temperature - TEMP_MIN) / (TEMP_MAX - TEMP_MIN));

  // (A1) Frequency: turbidity-led, nudged by depth and temperature.
  
  float envDifficulty = clamp01(0.6f * turbFraction + 0.25f * depthFraction + 0.15f * tempColdness);
  float delayUs = DELAY_US_CLEAR + (DELAY_US_MUDDY - DELAY_US_CLEAR) * envDifficulty;
  float Ts = delayUs / 1000000.0f;
  float baseFreqHz = 1.0f / (Ts * 20.0f);
  float f0 = baseFreqHz * (1.0f - SWEEP_SPAN_FRACTION);
  float f1 = baseFreqHz * (1.0f + SWEEP_SPAN_FRACTION);

  // (A2) Amplitude: turbidity-led, nudged by salinity; battery scales it down.
  float ampFromEnv = TURB_AMP_MIN + (TURB_AMP_MAX - TURB_AMP_MIN) * clamp01(0.8f * turbFraction + 0.2f * salFraction);

  bool lowBattery = battery < BATT_LOW_THRESHOLD;
  float battAmpScale = 1.0f;
  float battDurScale = 1.0f;
  unsigned long idlePauseMs = 0;
  if (lowBattery) {
    float batteryFraction = battery / BATT_LOW_THRESHOLD;
    battAmpScale = BATT_FLOOR_FRACTION + (1.0f - BATT_FLOOR_FRACTION) * batteryFraction;
    battDurScale = DUR_BATT_FLOOR_FRACTION + (1.0f - DUR_BATT_FLOOR_FRACTION) * batteryFraction;
    idlePauseMs = (unsigned long)((1.0f - batteryFraction) * IDLE_PAUSE_MAX_MS);
  }
  float ampScale = ampFromEnv * battAmpScale;

  // (B) Duration: depth-led, independent of Ts/frequency, then battery-scaled.
  int basePulseN = (int)(PULSE_N_MIN + (PULSE_N_MAX - PULSE_N_MIN) * depthFraction);
  int pulseNSamplesThis = (int)(basePulseN * battDurScale);
  if (pulseNSamplesThis < PULSE_N_FLOOR) pulseNSamplesThis = PULSE_N_FLOOR;
  float durationMs = pulseNSamplesThis * Ts * 1000.0f;

  Serial.print("Temp:");  Serial.print(temperature, 1);
  Serial.print(" Sal:");  Serial.print(salinity, 1);
  Serial.print(" Depth:");Serial.print(depth, 1);
  Serial.print(" Turb:"); Serial.print(turbidity, 1);
  Serial.print(" Batt:"); Serial.print(battery, 0);
  if (lowBattery) Serial.print("(LOW)");
  Serial.print(" Freq:"); Serial.print(baseFreqHz, 0);
  Serial.print(" Dur:");  Serial.print(durationMs, 1); Serial.print("ms");
  Serial.print(" Amp:");  Serial.print(ampScale, 2);
  Serial.print(" WF:");   Serial.print(waveformName(currentWaveform));
  Serial.print(" Win:");  Serial.println(windowName(currentWindow));

  display.clearDisplay();

  char battStr[8];
  sprintf(battStr, "%d%%%s", (int)battery, lowBattery ? "!" : "");
  int16_t bx1, by1; uint16_t bw, bh;
  display.getTextBounds(battStr, 0, 0, &bx1, &by1, &bw, &bh);
  display.setCursor(SCREEN_WIDTH - bw, 0);
  display.print(battStr);

  display.setCursor(0, 0);
  display.print("Temp: ");  display.print(temperature, 1); display.print("C");
  display.setCursor(0, 16);
  display.print("Depth: "); display.print(depth, 1); display.print("m");
  display.setCursor(0, 32);
  display.print("Sal:");    display.print(salinity, 1);
  display.print(" Turb:");  display.print(turbidity, 1); display.print("%");
  display.setCursor(0, 48);
  display.print(waveformName(currentWaveform));
  display.print("/");
  display.print(windowName(currentWindow));

  display.display();

  if (idlePauseMs > 0) {
    delay(idlePauseMs);
  }

  startPulse(f0, f1, baseFreqHz, Ts, ampScale, pulseNSamplesThis, currentWaveform, currentWindow);
}
