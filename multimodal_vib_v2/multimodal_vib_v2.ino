/* Integrated comparative sketch
   - ADXL via MCP3008, MPU-6500 (I2C), SW-420, Piezo A1/A2
   - Zero-centers sensors at startup (and re-zero on 'r')
   - Optional two-point ADXL calibration on 'c' (in Serial Monitor)
   - Smoothing: median + oversample, IIR + moving average
   - Computes Noise RMS, Response Avg, Sensitivity, scores, and FoM
   - Adjustable FoM weights (W_NOISE, W_RESPONSE, W_SENS)
   - Serial commands: 'c' run two-point ADXL calibration, 'r' re-zero, 'w' print weights
*/

#include <Wire.h>
#include <SPI.h>

// ---------- Hardware pins ----------
const int CS_PIN = 10;
const float ADC_MID_DEFAULT = 512.0;
float ADC_MID = ADC_MID_DEFAULT;
float ADXL_COUNTS_PER_G = 93.0;  // will be updated if two-point calibration used

#define IMU_ADDR 0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A

#define SW420_ANALOG A0
#define SW420_DIGITAL 52
#define PIEZO1_PIN A1
#define PIEZO2_PIN A2

// ---------- Smoothing and sampling params ----------
#define MEDIAN_N 5
#define OS_AVG_N 3
#define IIR_ALPHA 0.12f      // 0..1 (lower = smoother)
#define AVG_N 20             // moving average length
#define METRICS_INTERVAL_MS 7000UL

// ---------- FoM weight defaults (tuneable) ----------
double W_NOISE = 0.45;
double W_RESPONSE = 0.30;
double W_SENS = 0.25;

// ---------- Calibration offsets (zero-centering) ----------
float offAdxlX = 0, offAdxlY = 0, offAdxlZ = 0;
float offMpuX  = 0, offMpuY  = 0, offMpuZ  = 0;

// ---------- Smoothing buffers ----------
float bufAx[AVG_N], bufAy[AVG_N], bufAz[AVG_N];
int avgIndex = 0;
bool avgFilled = false;
float iirX = 0, iirY = 0, iirZ = 0;

// ---------- Metrics accumulators ----------
double adxlNoiseSum = 0.0, mpuNoiseSum = 0.0;
double adxlResponseSum = 0.0, mpuResponseSum = 0.0;
long sampleCount = 0;
float adxlLastMean = 0.0, mpuLastMean = 0.0;

// event sensitivity accumulators
double adxlEventPeakSum = 0.0, mpuEventPeakSum = 0.0;
long eventCount = 0;

// timing
unsigned long lastMetricsTime = 0;

// ---------- Helper inline conversions ----------
inline float adxlToG(int adc)   { return (adc - ADC_MID) / ADXL_COUNTS_PER_G; }
inline float mpuToG(int16_t r)  { return r / 16384.0f; } // ±2g

// ---------- Low-level MCP3008 reads ----------
int readMCP3008_once(int ch) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x01);
  byte mid = SPI.transfer(0b10000000 | (ch << 4));
  byte lsb = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return ((mid & 0x03) << 8) | lsb;
}

int readMCP3008_median(int ch) {
  int s[MEDIAN_N];
  for (int i = 0; i < MEDIAN_N; ++i) {
    s[i] = readMCP3008_once(ch);
    delay(2);
  }
  // insertion sort small array
  for (int i = 1; i < MEDIAN_N; ++i) {
    int v = s[i], j = i - 1;
    while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
    s[j + 1] = v;
  }
  return s[MEDIAN_N / 2];
}

int readMCP3008_os(int ch) {
  long sum = 0;
  for (int g = 0; g < OS_AVG_N; ++g) {
    sum += readMCP3008_median(ch);
  }
  return (int)(sum / OS_AVG_N);
}

// ---------- I2C helpers (MPU) ----------
void i2cWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission(true);
}
void i2cReadN(uint8_t reg, uint8_t n, uint8_t *buf) {
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((int)IMU_ADDR, (int)n);
  for (uint8_t i = 0; i < n && Wire.available(); ++i) buf[i] = Wire.read();
}
void imuInit() {
  i2cWrite8(REG_PWR_MGMT_1, 0x00);
  i2cWrite8(REG_SMPLRT_DIV, 19);
  i2cWrite8(REG_CONFIG, 0x03);
}
void readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t buf[6];
  i2cReadN(REG_ACCEL_XOUT_H, 6, buf);
  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
}

// ---------- Moving average helper ----------
float movingAvg(float *buf, float newVal) {
  buf[avgIndex] = newVal;
  int n = avgFilled ? AVG_N : (avgIndex + 1);
  float s = 0;
  for (int i = 0; i < n; ++i) s += buf[i];
  return s / n;
}

// ---------- Calibration: zero-center (mean) ----------
void calibrateZeroCenter(int N = 400) {
  Serial.println("Calibrating zero-center: keep board flat & still...");
  long double sAx=0, sAy=0, sAz=0, sMx=0, sMy=0, sMz=0;
  for (int i = 0; i < N; ++i) {
    int ax = readMCP3008_os(0);
    int ay = readMCP3008_os(1);
    int az = readMCP3008_os(2);
    sAx += adxlToG(ax);
    sAy += adxlToG(ay);
    sAz += adxlToG(az);
    int16_t mx, my, mz;
    readAccelRaw(mx, my, mz);
    sMx += mpuToG(mx);
    sMy += mpuToG(my);
    sMz += mpuToG(mz);
    delay(6);
  }
  offAdxlX = (float)(sAx / N);
  offAdxlY = (float)(sAy / N);
  offAdxlZ = (float)(sAz / N);
  offMpuX  = (float)(sMx / N);
  offMpuY  = (float)(sMy / N);
  offMpuZ  = (float)(sMz / N);
  Serial.println("Zero-centering complete.");
}

// ---------- Two-point calibration for ADXL Z (optional) ----------
void twoPointCalibrateADXL() {
  Serial.println("=== TWO-POINT ADXL CALIBRATION ===");
  Serial.println("Place board flat with +Z up. Press any key in Serial Monitor to capture +1g.");
  while (!Serial.available()) delay(50);
  Serial.read(); delay(200);
  const int N = 400;
  long sumPos = 0;
  for (int i = 0; i < N; ++i) { sumPos += readMCP3008_os(2); delay(6); }
  float avgPos = (float)sumPos / N;
  Serial.print("Captured +Z avg raw = "); Serial.println(avgPos, 3);

  Serial.println("Now flip board so Z = -1g (upside down). Press any key to capture -1g.");
  while (!Serial.available()) delay(50);
  Serial.read(); delay(200);
  long sumNeg = 0;
  for (int i = 0; i < N; ++i) { sumNeg += readMCP3008_os(2); delay(6); }
  float avgNeg = (float)sumNeg / N;
  Serial.print("Captured -Z avg raw = "); Serial.println(avgNeg, 3);

  float countsPerG = (avgPos - avgNeg) / 2.0f;
  float midCounts = (avgPos + avgNeg) / 2.0f;
  if (countsPerG > 5.0f) {
    ADXL_COUNTS_PER_G = countsPerG;
    ADC_MID = midCounts;
    Serial.print("Updated ADXL_COUNTS_PER_G = "); Serial.println(ADXL_COUNTS_PER_G, 4);
    Serial.print("Updated ADC_MID = "); Serial.println(ADC_MID, 3);
  } else {
    Serial.println("Calibration failed: countsPerG suspiciously small. No change made.");
  }
  while (Serial.available()) Serial.read();
  delay(200);
}

// ---------- Utility: print current FoM weights ----------
void printWeights() {
  Serial.println("FoM weights:");
  Serial.print(" W_NOISE = "); Serial.println(W_NOISE, 3);
  Serial.print(" W_RESPONSE = "); Serial.println(W_RESPONSE, 3);
  Serial.print(" W_SENS = "); Serial.println(W_SENS, 3);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(CS_PIN, OUTPUT); digitalWrite(CS_PIN, HIGH);
  SPI.begin(); SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  Wire.begin(); Wire.setClock(400000); imuInit();
  pinMode(SW420_DIGITAL, INPUT);
  analogReadResolution(10);

  Serial.println("Starting. Keep board flat & still for initial zero-centering (6s)...");
  delay(6000);
  calibrateZeroCenter();
  lastMetricsTime = millis();
  printWeights();
}

// ---------- Main loop ----------
void loop() {
  // Serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') twoPointCalibrateADXL();
    if (c == 'r') { calibrateZeroCenter(); }
    if (c == 'w') printWeights();
  }

  // --- ADXL readings (oversample+median) ---
  int axRaw = readMCP3008_os(0);
  int ayRaw = readMCP3008_os(1);
  int azRaw = readMCP3008_os(2);

  // convert raw -> g, then zero-center
  float gx = adxlToG(axRaw) - offAdxlX;
  float gy = adxlToG(ayRaw) - offAdxlY;
  float gz = adxlToG(azRaw) - offAdxlZ;

  // IIR smoothing
  iirX = (1.0f - IIR_ALPHA) * iirX + IIR_ALPHA * gx;
  iirY = (1.0f - IIR_ALPHA) * iirY + IIR_ALPHA * gy;
  iirZ = (1.0f - IIR_ALPHA) * iirZ + IIR_ALPHA * gz;

  // moving average
  float sAx = movingAvg(bufAx, iirX);
  float sAy = movingAvg(bufAy, iirY);
  float sAz = movingAvg(bufAz, iirZ);
  avgIndex = (avgIndex + 1) % AVG_N;
  if (avgIndex == 0) avgFilled = true;

  // --- MPU readings ---
  int16_t mxr, myr, mzr;
  readAccelRaw(mxr, myr, mzr);
  float mX = mpuToG(mxr) - offMpuX;
  float mY = mpuToG(myr) - offMpuY;
  float mZ = mpuToG(mzr) - offMpuZ;

  // --- SW-420 and piezos ---
  int vibAnalog = analogRead(SW420_ANALOG);
  int vibDigital = digitalRead(SW420_DIGITAL);
  int pz1 = analogRead(PIEZO1_PIN);
  int pz2 = analogRead(PIEZO2_PIN);

  // piezo envelope approx (center ~512)
  float pz1_mag = fabs((float)pz1 - 512.0f) / 512.0f;
  float pz2_mag = fabs((float)pz2 - 512.0f) / 512.0f;
  float piezo_env = max(pz1_mag, pz2_mag);

  // --- Metrics accumulation ---
  float meanAdxl = (sAx + sAy + sAz) / 3.0f;
  float meanMpu  = (mX + mY + mZ) / 3.0f;

  adxlNoiseSum += (double)meanAdxl * (double)meanAdxl;
  mpuNoiseSum  += (double)meanMpu  * (double)meanMpu;

  adxlResponseSum += fabs(meanAdxl - adxlLastMean);
  mpuResponseSum  += fabs(meanMpu  - mpuLastMean);
  adxlLastMean = meanAdxl; mpuLastMean = meanMpu;

  sampleCount++;

  // Event detection
  const float PIEZO_THRESHOLD = 0.12f;
  bool eventNow = (vibDigital == 1) || (piezo_env > PIEZO_THRESHOLD);
  if (eventNow) {
    float peakAdxl = (fabs(sAx) + fabs(sAy) + fabs(sAz)) / 3.0f;
    float peakMpu  = (fabs(mX) + fabs(mY) + fabs(mZ)) / 3.0f;
    adxlEventPeakSum += peakAdxl;
    mpuEventPeakSum  += peakMpu;
    eventCount++;
  }

  // --- Serial Plotter live output ---
  Serial.print("ADXL_X:"); Serial.print(sAx); Serial.print('\t');
  Serial.print("ADXL_Y:"); Serial.print(sAy); Serial.print('\t');
  Serial.print("ADXL_Z:"); Serial.print(sAz); Serial.print('\t');

  Serial.print("MPU_X:");  Serial.print(mX); Serial.print('\t');
  Serial.print("MPU_Y:");  Serial.print(mY); Serial.print('\t');
  Serial.print("MPU_Z:");  Serial.print(mZ); Serial.print('\t');

  Serial.print("VIB_A:");  Serial.print(vibAnalog); Serial.print('\t');
  Serial.print("VIB_D:");  Serial.print(vibDigital); Serial.print('\t');

  Serial.print("PZ1:"); Serial.print(pz1); Serial.print('\t');
  Serial.print("PZ2:"); Serial.print(pz2);
  Serial.println();

  // --- Periodic metrics report ---
  if (millis() - lastMetricsTime >= METRICS_INTERVAL_MS && sampleCount > 5) {
    double adxlNoiseRMS = sqrt(adxlNoiseSum / sampleCount);
    double mpuNoiseRMS  = sqrt(mpuNoiseSum  / sampleCount);
    double adxlRespAvg  = adxlResponseSum / sampleCount;
    double mpuRespAvg   = mpuResponseSum  / sampleCount;

    double avgPeakAdxl = (eventCount > 0) ? (adxlEventPeakSum / eventCount) : 0.0;
    double avgPeakMpu  = (eventCount > 0) ? (mpuEventPeakSum  / eventCount) : 0.0;

    // Noise score (higher better). If MPU noise is extremely small, avoid divide-by-zero.
    double noiseRatio = (mpuNoiseRMS > 1e-12) ? (adxlNoiseRMS / mpuNoiseRMS) : 1.0;
    double noiseScore = 100.0 * (1.0 / noiseRatio);
    if (noiseScore > 100.0) noiseScore = 100.0;
    if (noiseScore < 0.0) noiseScore = 0.0;

    // Response score (100 when equal)
    double responseScore = 100.0;
    if (adxlRespAvg > 1e-12 && mpuRespAvg > 1e-12) {
      double rratio = min(adxlRespAvg, mpuRespAvg) / max(adxlRespAvg, mpuRespAvg);
      responseScore = 100.0 * rratio;
    }

    // Sensitivity score
    double sensitivityScore = 70.0; // neutral default if no events
    if (eventCount > 0 && avgPeakMpu > 1e-6) {
      double sratio = min(avgPeakAdxl, avgPeakMpu) / max(avgPeakAdxl, avgPeakMpu);
      sensitivityScore = 100.0 * sratio;
    }

    // Combine FoM
    double FoM = W_NOISE * noiseScore + W_RESPONSE * responseScore + W_SENS * sensitivityScore;
    if (FoM < 0) FoM = 0; if (FoM > 100) FoM = 100;

    // Print summary
    Serial.println("\n===== METRICS =====");
    Serial.print("Samples: "); Serial.println(sampleCount);
    Serial.print("ADXL Noise RMS (g): "); Serial.println(adxlNoiseRMS, 6);
    Serial.print("MPU  Noise RMS (g): "); Serial.println(mpuNoiseRMS, 6);
    Serial.print("Noise Score (0-100): "); Serial.println(noiseScore, 3);
    Serial.print("ADXL Resp Avg (g/change): "); Serial.println(adxlRespAvg, 6);
    Serial.print("MPU  Resp Avg (g/change): "); Serial.println(mpuRespAvg, 6);
    Serial.print("Response Score (0-100): "); Serial.println(responseScore, 3);
    Serial.print("EventCount: "); Serial.println(eventCount);
    Serial.print("AvgPeak ADXL (g): "); Serial.println(avgPeakAdxl, 6);
    Serial.print("AvgPeak MPU  (g): "); Serial.println(avgPeakMpu, 6);
    Serial.print("Sensitivity Score (0-100): "); Serial.println(sensitivityScore, 3);
    Serial.print("=> Figure of Merit (0-100): "); Serial.println(FoM, 3);
    Serial.println("===================\n");

    // Reset accumulators
    adxlNoiseSum = mpuNoiseSum = 0.0;
    adxlResponseSum = mpuResponseSum = 0.0;
    adxlEventPeakSum = mpuEventPeakSum = 0.0;
    sampleCount = 0; eventCount = 0;
    lastMetricsTime = millis();
  }

  delay(120); // stable update rate (~8 Hz)
}