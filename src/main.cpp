#include "header.h"

#include "MQTTHandle.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

Adafruit_ADS1115 ads;
Adafruit_ADS1115 adsPh;
MD_AD9833 AD(DATA_PIN, SCLK_PIN, FSYNC_PIN);
Adafruit_SSD1306 oledDisplay(128, 32, &Wire, -1);

float V0_rms = 0.0f;
float vexc_rms = 0.0f;
double Kcell = NAN;

namespace timing {
constexpr uint32_t MEASUREMENT_INTERVAL_MS = 5000;
constexpr uint32_t PUBLISH_INTERVAL_MS = 60000;
constexpr uint32_t VEXC_REFRESH_INTERVAL_MS = 30UL * 60UL * 1000UL;
constexpr size_t MEASUREMENT_BUFFER_CAPACITY =
    (PUBLISH_INTERVAL_MS + MEASUREMENT_INTERVAL_MS - 1) / MEASUREMENT_INTERVAL_MS + 2;
}

namespace calibrationStorage {
constexpr char kNamespace[] = "ec_sensor";
constexpr char kCellKey[] = "kcell";
constexpr char kPhMidVoltageKey[] = "ph_mid_v";
constexpr char kPhMidBufferKey[] = "ph_mid_buf";
constexpr char kPhLowSlope25Key[] = "ph_low_s25";
constexpr char kPhLowBufferKey[] = "ph_low_buf";
constexpr char kPhHighSlope25Key[] = "ph_hi_s25";
constexpr char kPhHighBufferKey[] = "ph_hi_buf";
}

namespace oledConfig {
constexpr uint8_t kKnownAddresses[] = {0x3C, 0x3D};
constexpr uint32_t kRefreshIntervalMs = 1000;
}

namespace phDefaults {
constexpr double kNeutralVoltage = 1.50000;
constexpr double kAcidVoltage = 2.03244;
constexpr double kSlope25 = (kAcidVoltage - kNeutralVoltage) / (PH_NEUTRAL_DEFAULT - PH_LOW_DEFAULT);
}

namespace phBufferTables {
constexpr size_t kCount = 11;
constexpr float kTempsC[kCount] = {0.0f, 5.0f, 10.0f, 15.0f, 20.0f, 25.0f, 30.0f, 35.0f, 40.0f, 45.0f, 50.0f};
constexpr float kPh401[kCount] = {4.000f, 3.999f, 3.999f, 4.001f, 4.005f, 4.010f, 4.015f, 4.020f, 4.026f, 4.033f, 4.041f};
constexpr float kPh700[kCount] = {7.111f, 7.082f, 7.056f, 7.033f, 7.013f, 6.997f, 6.984f, 6.974f, 6.966f, 6.959f, 6.953f};
constexpr float kPh1001[kCount] = {10.320f, 10.249f, 10.182f, 10.121f, 10.064f, 10.013f, 9.966f, 9.924f, 9.886f, 9.852f, 9.820f};
}

struct PhCalibrationData {
  double neutralVoltage = NAN;
  double neutralBufferPh = PH_NEUTRAL_DEFAULT;
  double lowSlope25 = NAN;
  double lowBufferPh = PH_LOW_DEFAULT;
  double highSlope25 = NAN;
  double highBufferPh = PH_HIGH_DEFAULT;
};

static unsigned long lastMeasurementAt = 0;
static unsigned long lastPublishAt = 0;
static unsigned long lastExcitationRefreshAt = 0;
static unsigned long lastNetworkAt = 0;
static unsigned long lastDisplayAt = 0;
static ECraw measurementBuffer[timing::MEASUREMENT_BUFFER_CAPACITY];
static size_t measurementBufferCount = 0;
static ECraw lastMeasurement;
static bool hasLastMeasurement = false;
static bool oledReady = false;
static uint8_t oledAddress = 0;
static PhCalibrationData phCalibration;

static void applyDefaultPhCalibration() {
  phCalibration.neutralVoltage = phDefaults::kNeutralVoltage;
  phCalibration.neutralBufferPh = PH_NEUTRAL_DEFAULT;
  phCalibration.lowSlope25 = phDefaults::kSlope25;
  phCalibration.lowBufferPh = PH_LOW_DEFAULT;
  phCalibration.highSlope25 = phDefaults::kSlope25;
  phCalibration.highBufferPh = PH_HIGH_DEFAULT;
}

static float clamp01(float value) {
  return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

static float clampPh(float value) {
  if (!isfinite(value)) {
    return NAN;
  }
  if (value < PH_MIN_VALUE) {
    return PH_MIN_VALUE;
  }
  if (value > PH_MAX_VALUE) {
    return PH_MAX_VALUE;
  }
  return value;
}

static double toKelvin(float temperatureC) {
  return static_cast<double>(temperatureC) + KELVIN_OFFSET;
}

static float sanitizeTemperature(float temperature) {
  if (!isfinite(temperature) || temperature <= -100.0f) {
    return TEMP_REF;
  }
  return temperature;
}

static float interpolateBufferValue(const float* table, float temperatureC) {
  float temp = sanitizeTemperature(temperatureC);
  if (temp <= phBufferTables::kTempsC[0]) {
    return table[0];
  }
  if (temp >= phBufferTables::kTempsC[phBufferTables::kCount - 1]) {
    return table[phBufferTables::kCount - 1];
  }

  for (size_t i = 1; i < phBufferTables::kCount; ++i) {
    if (temp <= phBufferTables::kTempsC[i]) {
      float x0 = phBufferTables::kTempsC[i - 1];
      float x1 = phBufferTables::kTempsC[i];
      float y0 = table[i - 1];
      float y1 = table[i];
      float alpha = (temp - x0) / (x1 - x0);
      return y0 + alpha * (y1 - y0);
    }
  }

  return table[phBufferTables::kCount - 1];
}

static float compensatePhBufferValue(float bufferPhAt25C, float temperatureC) {
  if (fabsf(bufferPhAt25C - 4.01f) <= 0.2f || fabsf(bufferPhAt25C - 4.00f) <= 0.2f) {
    return interpolateBufferValue(phBufferTables::kPh401, temperatureC);
  }
  if (fabsf(bufferPhAt25C - 7.00f) <= 0.2f) {
    return interpolateBufferValue(phBufferTables::kPh700, temperatureC);
  }
  if (fabsf(bufferPhAt25C - 10.01f) <= 0.3f || fabsf(bufferPhAt25C - 10.00f) <= 0.3f) {
    return interpolateBufferValue(phBufferTables::kPh1001, temperatureC);
  }
  return bufferPhAt25C;
}

static void startWifiConnection() {
  WiFi.disconnect();
  WiFi.begin(ssid, pass);
}

static void ensureWifiConnected(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  if (now - lastWifiReconnectAttempt < WifiReconnectIntervalMs) {
    return;
  }

  lastWifiReconnectAttempt = now;
  Serial.println("WLAN getrennt, starte Neuverbindung");
  startWifiConnection();
}

static bool loadStoredKcell() {
  Preferences prefs;
  if (!prefs.begin(calibrationStorage::kNamespace, true)) {
    Serial.println(F("Kcell storage could not be opened for reading"));
    return false;
  }

  double storedKcell = prefs.getDouble(calibrationStorage::kCellKey, NAN);
  prefs.end();

  if (!isfinite(storedKcell) || storedKcell <= 0.0) {
    Serial.println(F("No stored Kcell found"));
    return false;
  }

  Kcell = storedKcell;
  Serial.print(F("Loaded stored Kcell="));
  Serial.println(Kcell, 6);
  return true;
}

static bool saveStoredKcell(double value) {
  if (!isfinite(value) || value <= 0.0) {
    return false;
  }

  Preferences prefs;
  if (!prefs.begin(calibrationStorage::kNamespace, false)) {
    Serial.println(F("Kcell storage could not be opened for writing"));
    return false;
  }

  size_t written = prefs.putDouble(calibrationStorage::kCellKey, value);
  prefs.end();

  if (written != sizeof(double)) {
    Serial.println(F("Saving Kcell failed"));
    return false;
  }

  Serial.println(F("Kcell saved to device"));
  return true;
}

static bool saveStoredPhCalibration() {
  Preferences prefs;
  if (!prefs.begin(calibrationStorage::kNamespace, false)) {
    Serial.println(F("pH storage could not be opened for writing"));
    return false;
  }

  prefs.putDouble(calibrationStorage::kPhMidVoltageKey, phCalibration.neutralVoltage);
  prefs.putDouble(calibrationStorage::kPhMidBufferKey, phCalibration.neutralBufferPh);
  prefs.putDouble(calibrationStorage::kPhLowSlope25Key, phCalibration.lowSlope25);
  prefs.putDouble(calibrationStorage::kPhLowBufferKey, phCalibration.lowBufferPh);
  prefs.putDouble(calibrationStorage::kPhHighSlope25Key, phCalibration.highSlope25);
  prefs.putDouble(calibrationStorage::kPhHighBufferKey, phCalibration.highBufferPh);
  prefs.end();

  Serial.println(F("pH calibration saved to device"));
  return true;
}

static bool loadStoredPhCalibration() {
  Preferences prefs;
  if (!prefs.begin(calibrationStorage::kNamespace, true)) {
    Serial.println(F("pH storage could not be opened for reading"));
    return false;
  }

  phCalibration.neutralVoltage = prefs.getDouble(calibrationStorage::kPhMidVoltageKey, NAN);
  phCalibration.neutralBufferPh = prefs.getDouble(calibrationStorage::kPhMidBufferKey, PH_NEUTRAL_DEFAULT);
  phCalibration.lowSlope25 = prefs.getDouble(calibrationStorage::kPhLowSlope25Key, NAN);
  phCalibration.lowBufferPh = prefs.getDouble(calibrationStorage::kPhLowBufferKey, PH_LOW_DEFAULT);
  phCalibration.highSlope25 = prefs.getDouble(calibrationStorage::kPhHighSlope25Key, NAN);
  phCalibration.highBufferPh = prefs.getDouble(calibrationStorage::kPhHighBufferKey, PH_HIGH_DEFAULT);
  prefs.end();

  if (!isfinite(phCalibration.neutralVoltage)) {
    applyDefaultPhCalibration();
    Serial.println(F("No stored pH calibration found, using DFRobot defaults"));
    return false;
  }

  if (!isfinite(phCalibration.lowSlope25)) {
    phCalibration.lowSlope25 = phDefaults::kSlope25;
    phCalibration.lowBufferPh = PH_LOW_DEFAULT;
  }

  if (!isfinite(phCalibration.highSlope25)) {
    phCalibration.highSlope25 = phDefaults::kSlope25;
    phCalibration.highBufferPh = PH_HIGH_DEFAULT;
  }

  Serial.print(F("Loaded pH neutral voltage="));
  Serial.println(phCalibration.neutralVoltage, 6);
  return true;
}

static void clearStoredPhCalibration() {
  applyDefaultPhCalibration();
  saveStoredPhCalibration();
  Serial.println(F("pH calibration reset to DFRobot defaults"));
}

static bool hasNeutralPhCalibration() {
  return isfinite(phCalibration.neutralVoltage);
}

static uint8_t detectOledAddress() {
  for (uint8_t address : oledConfig::kKnownAddresses) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      return address;
    }
  }
  return 0;
}

static void drawOledStatus(const char* line1, const char* line2 = "", const char* line3 = "", const char* line4 = "") {
  if (!oledReady) {
    return;
  }

  oledDisplay.clearDisplay();
  oledDisplay.setTextSize(1);
  oledDisplay.setTextColor(SSD1306_WHITE);
  oledDisplay.setCursor(0, 0);
  oledDisplay.println(line1);
  oledDisplay.println(line2);
  oledDisplay.println(line3);
  oledDisplay.println(line4);
  oledDisplay.display();
}

static bool initOledDisplay() {
  oledAddress = detectOledAddress();
  if (oledAddress == 0) {
    Serial.println(F("SSD1306 display not found on I2C"));
    return false;
  }

  if (!oledDisplay.begin(SSD1306_SWITCHCAPVCC, oledAddress, false, false)) {
    Serial.println(F("SSD1306 initialization failed"));
    return false;
  }

  oledReady = true;
  oledDisplay.clearDisplay();
  oledDisplay.display();

  char addressLine[20];
  snprintf(addressLine, sizeof(addressLine), "Addr: 0x%02X", oledAddress);
  drawOledStatus("OLED bereit", addressLine, "EC+pH Boot", "");
  Serial.print(F("SSD1306 initialized at 0x"));
  Serial.println(oledAddress, HEX);
  return true;
}

static void updateOledMeasurement(const ECraw& measurement) {
  if (!oledReady) {
    return;
  }

  oledDisplay.clearDisplay();
  oledDisplay.setTextSize(1);
  oledDisplay.setTextColor(SSD1306_WHITE);
  oledDisplay.setCursor(0, 0);
  oledDisplay.print(F("EC: "));
  oledDisplay.print(measurement.EC_comp_mS, 2);
  oledDisplay.println(F(" mS"));
  oledDisplay.print(F("pH : "));
  if (isfinite(measurement.pHValue)) {
    oledDisplay.println(measurement.pHValue, 2);
  } else {
    oledDisplay.println(F("CAL"));
  }
  oledDisplay.print(F("T:"));
  oledDisplay.print(measurement.temperature, 1);
  oledDisplay.print(F(" W:"));
  oledDisplay.println(WiFi.status() == WL_CONNECTED ? F("OK") : F("OFF"));
  oledDisplay.display();
}

static void updateOledNoMeasurement() {
  drawOledStatus("EC+pH Sensor", "Warte auf", "erste Messung", "");
}

static adsGain_t coarserGain(adsGain_t gain) {
  if (gain == GAIN_SIXTEEN) return GAIN_EIGHT;
  if (gain == GAIN_EIGHT) return GAIN_FOUR;
  if (gain == GAIN_FOUR) return GAIN_TWO;
  if (gain == GAIN_TWO) return GAIN_ONE;
  return GAIN_ONE;
}

static adsGain_t finerGain(adsGain_t gain) {
  if (gain == GAIN_ONE) return GAIN_TWO;
  if (gain == GAIN_TWO) return GAIN_FOUR;
  if (gain == GAIN_FOUR) return GAIN_EIGHT;
  if (gain == GAIN_EIGHT) return GAIN_SIXTEEN;
  return GAIN_SIXTEEN;
}

static void selectExcitationPath() {
  digitalWrite(RELAY_PIN, HIGH);
  delay(T_SETTLE_MS);
}

static void selectCellPath() {
  digitalWrite(RELAY_PIN, LOW);
  delay(T_SETTLE_MS);
}

static int16_t readAdcAutoRange(uint8_t channel = EC_ADC_CHANNEL) {
  for (int i = 0; i < 3; ++i) {
    int16_t raw = ads.readADC_SingleEnded(channel);
    adsGain_t currentGain = ads.getGain();
    float fraction = fabsf(static_cast<float>(raw) / 32767.0f);

    if (abs(raw) >= 32760 && currentGain != GAIN_ONE) {
      ads.setGain(coarserGain(currentGain));
      delay(3);
      continue;
    }

    if (fraction > HI_FRAC && currentGain != GAIN_ONE) {
      ads.setGain(coarserGain(currentGain));
      delay(3);
      continue;
    }

    if (fraction < LO_FRAC && currentGain != GAIN_SIXTEEN) {
      ads.setGain(finerGain(currentGain));
      delay(3);
      continue;
    }

    return raw;
  }

  return ads.readADC_SingleEnded(channel);
}

static float trimmedMeanFromSamples(float* values, int count) {
  for (int i = 0; i < count - 1; ++i) {
    for (int j = i + 1; j < count; ++j) {
      if (values[j] < values[i]) {
        float t = values[i];
        values[i] = values[j];
        values[j] = t;
      }
    }
  }

  int cut = count / 10;
  int start = cut;
  int end = count - cut;
  if (end <= start) {
    return values[count / 2];
  }

  float sum = 0.0f;
  for (int i = start; i < end; ++i) {
    sum += values[i];
  }
  return sum / static_cast<float>(end - start);
}

static float readAverageVoltage(uint8_t channel = EC_ADC_CHANNEL, int samples = N_AVG) {
  const int maxSamples = 128;
  int n = samples;
  if (n < 8) n = 8;
  if (n > maxSamples) n = maxSamples;

  float vals[maxSamples];
  for (int i = 0; i < n; ++i) {
    int16_t reading = readAdcAutoRange(channel);
    vals[i] = ads.computeVolts(reading);
  }

  return trimmedMeanFromSamples(vals, n);
}

static float readAverageVoltageFixed(Adafruit_ADS1115& device, uint8_t channel, int samples) {
  const int maxSamples = 128;
  int n = samples;
  if (n < 8) n = 8;
  if (n > maxSamples) n = maxSamples;

  float vals[maxSamples];
  for (int i = 0; i < n; ++i) {
    int16_t reading = device.readADC_SingleEnded(channel);
    vals[i] = device.computeVolts(reading);
  }

  return trimmedMeanFromSamples(vals, n);
}

static const char* gainToString(adsGain_t gain) {
  switch (gain) {
    case GAIN_SIXTEEN:
      return "+/-0.256V";
    case GAIN_EIGHT:
      return "+/-0.512V";
    case GAIN_FOUR:
      return "+/-1.024V";
    case GAIN_TWO:
      return "+/-2.048V";
    case GAIN_ONE:
      return "+/-4.096V";
    case GAIN_TWOTHIRDS:
      return "+/-6.144V";
    default:
      return "unknown";
  }
}

static void clearMeasurementBuffer() {
  measurementBufferCount = 0;
}

static void appendMeasurement(const ECraw& measurement) {
  if (measurementBufferCount < timing::MEASUREMENT_BUFFER_CAPACITY) {
    measurementBuffer[measurementBufferCount++] = measurement;
    return;
  }

  for (size_t i = 1; i < timing::MEASUREMENT_BUFFER_CAPACITY; ++i) {
    measurementBuffer[i - 1] = measurementBuffer[i];
  }
  measurementBuffer[timing::MEASUREMENT_BUFFER_CAPACITY - 1] = measurement;
}

static void sortValues(double* values, size_t count) {
  for (size_t i = 0; i + 1 < count; ++i) {
    for (size_t j = i + 1; j < count; ++j) {
      if (values[j] < values[i]) {
        double t = values[i];
        values[i] = values[j];
        values[j] = t;
      }
    }
  }
}

static double trimmedMean(const double* values, size_t count) {
  if (count == 0) {
    return NAN;
  }

  double sorted[timing::MEASUREMENT_BUFFER_CAPACITY];
  for (size_t i = 0; i < count; ++i) {
    sorted[i] = values[i];
  }

  sortValues(sorted, count);

  size_t cut = count / 10;
  if (count >= 10 && cut == 0) {
    cut = 1;
  }
  if ((cut * 2) >= count) {
    cut = 0;
  }

  double sum = 0.0;
  for (size_t i = cut; i < (count - cut); ++i) {
    sum += sorted[i];
  }

  return sum / static_cast<double>(count - (2 * cut));
}

static bool buildMeasurementSummary(ECraw& summary) {
  if (measurementBufferCount == 0) {
    return false;
  }

  double vCellValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double rhoValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double rCellValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double ecRawValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double temperatureValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double ecCompValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double kCellValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double phVoltageValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double phRawValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double phValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double phNeutralVoltageValues[timing::MEASUREMENT_BUFFER_CAPACITY];
  double phSlope25Values[timing::MEASUREMENT_BUFFER_CAPACITY];

  for (size_t i = 0; i < measurementBufferCount; ++i) {
    vCellValues[i] = measurementBuffer[i].Vcell;
    rhoValues[i] = measurementBuffer[i].rho;
    rCellValues[i] = measurementBuffer[i].Rcell;
    ecRawValues[i] = measurementBuffer[i].EC_raw_mS;
    temperatureValues[i] = measurementBuffer[i].temperature;
    ecCompValues[i] = measurementBuffer[i].EC_comp_mS;
    kCellValues[i] = measurementBuffer[i].Kcell;
    phVoltageValues[i] = measurementBuffer[i].pHVoltage;
    phRawValues[i] = measurementBuffer[i].pHRawValue;
    phValues[i] = measurementBuffer[i].pHValue;
    phNeutralVoltageValues[i] = measurementBuffer[i].pHNeutralVoltage;
    phSlope25Values[i] = measurementBuffer[i].pHSlope25;
  }

  summary.Vcell = static_cast<float>(trimmedMean(vCellValues, measurementBufferCount));
  summary.rho = static_cast<float>(trimmedMean(rhoValues, measurementBufferCount));
  summary.Rcell = trimmedMean(rCellValues, measurementBufferCount);
  summary.EC_raw_mS = trimmedMean(ecRawValues, measurementBufferCount);
  summary.temperature = static_cast<float>(trimmedMean(temperatureValues, measurementBufferCount));
  summary.EC_comp_mS = trimmedMean(ecCompValues, measurementBufferCount);
  summary.Kcell = trimmedMean(kCellValues, measurementBufferCount);
  summary.pHVoltage = static_cast<float>(trimmedMean(phVoltageValues, measurementBufferCount));
  summary.pHRawValue = static_cast<float>(trimmedMean(phRawValues, measurementBufferCount));
  summary.pHValue = static_cast<float>(trimmedMean(phValues, measurementBufferCount));
  summary.pHNeutralVoltage = static_cast<float>(trimmedMean(phNeutralVoltageValues, measurementBufferCount));
  summary.pHSlope25 = static_cast<float>(trimmedMean(phSlope25Values, measurementBufferCount));
  return true;
}

static float calcTemperature() {
  float u2 = readAverageVoltage(NTC_ADC_CHANNEL, N_AVG);
  float resistance2 = u2 * R1_ref / (Uref - u2);
  float temperature = 1.0f / (1.0f / T_0_K + (1.0f / B_CONST) * logf(resistance2 / R_NTC_25C)) - KELVIN_OFFSET;
  return temperature;
}

static void refreshExcitationReference(bool clearBufferedSamples = true) {
  Serial.println(F("Refreshing excitation reference"));
  selectExcitationPath();
  vexc_rms = readAverageVoltage(EC_ADC_CHANNEL);
  selectCellPath();
  lastExcitationRefreshAt = millis();

  if (clearBufferedSamples) {
    clearMeasurementBuffer();
  }

  Serial.print(F("Vexc="));
  Serial.print(vexc_rms, 6);
  Serial.println(F(" V"));
}

static float readPhVoltage() {
  return readAverageVoltageFixed(adsPh, PH_ADC_CHANNEL, PH_N_AVG);
}

static double getDefaultPhSlope25() {
  if (isfinite(phCalibration.lowSlope25) && isfinite(phCalibration.highSlope25)) {
    return 0.5 * (phCalibration.lowSlope25 + phCalibration.highSlope25);
  }
  if (isfinite(phCalibration.lowSlope25)) {
    return phCalibration.lowSlope25;
  }
  if (isfinite(phCalibration.highSlope25)) {
    return phCalibration.highSlope25;
  }
  return phDefaults::kSlope25;
}

static double selectPhSlope25(float deltaVoltage) {
  if (deltaVoltage >= 0.0f && isfinite(phCalibration.lowSlope25)) {
    return phCalibration.lowSlope25;
  }
  if (deltaVoltage < 0.0f && isfinite(phCalibration.highSlope25)) {
    return phCalibration.highSlope25;
  }
  return getDefaultPhSlope25();
}

static float getActivePhSlope25(float phVoltage) {
  return static_cast<float>(selectPhSlope25(phVoltage - phCalibration.neutralVoltage));
}

static float calculatePhFromVoltage(float phVoltage, float temperature, bool temperatureCompensated) {
  if (!hasNeutralPhCalibration()) {
    return NAN;
  }

  double slope25 = selectPhSlope25(phVoltage - phCalibration.neutralVoltage);
  if (!isfinite(slope25) || fabs(slope25) < 1e-6) {
    return NAN;
  }

  double tempFactor = 1.0;
  if (temperatureCompensated) {
    tempFactor = toKelvin(sanitizeTemperature(temperature)) / toKelvin(TEMP_REF);
  }
  double slopeAtMeasurement = slope25 * tempFactor;
  double phValue = phCalibration.neutralBufferPh - ((phVoltage - phCalibration.neutralVoltage) / slopeAtMeasurement);
  return clampPh(static_cast<float>(phValue));
}

static void logPhCalibrationState() {
  Serial.print(F("pH neutral V="));
  Serial.print(phCalibration.neutralVoltage, 6);
  Serial.print(F(" @ pH "));
  Serial.println(phCalibration.neutralBufferPh, 2);

  Serial.print(F("pH low slope25="));
  Serial.print(phCalibration.lowSlope25, 6);
  Serial.print(F(" using buffer "));
  Serial.println(phCalibration.lowBufferPh, 2);

  Serial.print(F("pH high slope25="));
  Serial.print(phCalibration.highSlope25, 6);
  Serial.print(F(" using buffer "));
  Serial.println(phCalibration.highBufferPh, 2);
}

static bool calibratePhNeutral(float bufferPh) {
  float temperature = sanitizeTemperature(calcTemperature());
  float voltage = readPhVoltage();
  if (!isfinite(voltage)) {
    return false;
  }

  float compensatedBufferPh = compensatePhBufferValue(bufferPh, temperature);

  phCalibration.neutralVoltage = voltage;
  phCalibration.neutralBufferPh = compensatedBufferPh;
  phCalibration.lowSlope25 = NAN;
  phCalibration.highSlope25 = NAN;

  Serial.print(F("Stored pH neutral voltage="));
  Serial.print(voltage, 6);
  Serial.print(F(" V at pH "));
  Serial.print(compensatedBufferPh, 3);
  Serial.print(F(" (input "));
  Serial.print(bufferPh, 2);
  Serial.println(F(" @25C)"));
  Serial.print(F("Calibration temperature="));
  Serial.println(temperature, 2);

  saveStoredPhCalibration();
  return true;
}

static bool calibratePhSlope(float bufferPh, bool isLowPoint) {
  if (!hasNeutralPhCalibration()) {
    Serial.println(F("Set pH neutral point first"));
    return false;
  }

  float temperature = sanitizeTemperature(calcTemperature());
  float voltage = readPhVoltage();
  float compensatedBufferPh = compensatePhBufferValue(bufferPh, temperature);
  double deltaPh = phCalibration.neutralBufferPh - compensatedBufferPh;
  if (fabs(deltaPh) < 1e-6) {
    Serial.println(F("pH buffer must differ from neutral point"));
    return false;
  }

  double slopeAtCalibration = (voltage - phCalibration.neutralVoltage) / deltaPh;
  double slope25 = slopeAtCalibration * (toKelvin(TEMP_REF) / toKelvin(temperature));
  if (!isfinite(slope25) || fabs(slope25) < 1e-6) {
    Serial.println(F("Computed pH slope is invalid"));
    return false;
  }

  if (isLowPoint) {
    phCalibration.lowSlope25 = slope25;
    phCalibration.lowBufferPh = compensatedBufferPh;
  } else {
    phCalibration.highSlope25 = slope25;
    phCalibration.highBufferPh = compensatedBufferPh;
  }

  Serial.print(F("Stored pH slope25="));
  Serial.print(slope25, 6);
  Serial.print(F(" V/pH using buffer "));
  Serial.print(compensatedBufferPh, 3);
  Serial.print(F(" (input "));
  Serial.print(bufferPh, 2);
  Serial.println(F(" @25C)"));
  saveStoredPhCalibration();
  return true;
}

static double calibrateKcellFromStd(float kappaStd25C) {
  float temperature = sanitizeTemperature(calcTemperature());
  float vCell = readAverageVoltage(EC_ADC_CHANNEL);
  float numerator = fabsf(vCell - V0_rms);
  float denominator = fmaxf(fabsf(vexc_rms - V0_rms), 1e-6f);
  float rho = clamp01(numerator / denominator);

  float oneMinus = fmaxf(1.0f - rho, 1e-6f);
  double rCell = Rref * (rho / oneMinus);

  double kappaAtTemperature = kappaStd25C * (1.0 + TEMP_COEF * (temperature - TEMP_REF));
  Kcell = kappaAtTemperature * (rCell / 1000.0);
  return Kcell;
}

static ECraw measureEcRaw() {
  ECraw result;

  result.temperature = sanitizeTemperature(calcTemperature());
  result.pHVoltage = readPhVoltage();
  result.pHNeutralVoltage = static_cast<float>(phCalibration.neutralVoltage);
  result.pHSlope25 = getActivePhSlope25(result.pHVoltage);
  result.pHRawValue = calculatePhFromVoltage(result.pHVoltage, TEMP_REF, false);
  result.pHValue = calculatePhFromVoltage(result.pHVoltage, result.temperature, true);

  result.Vcell = readAverageVoltage(EC_ADC_CHANNEL);
  float numerator = fabsf(result.Vcell - V0_rms);
  float denominator = fmaxf(fabsf(vexc_rms - V0_rms), 1e-6f);
  result.rho = clamp01(numerator / denominator);

  float oneMinus = fmaxf(1.0f - result.rho, 1e-6f);
  result.Rcell = Rref * (result.rho / oneMinus);
  result.EC_raw_mS = 1000.0 / result.Rcell;

  float tempFactor = 1.0f + TEMP_COEF * (result.temperature - TEMP_REF);
  double ecMeasured = result.EC_raw_mS * (isfinite(Kcell) ? Kcell : 1.0);
  result.EC_comp_mS = ecMeasured / tempFactor;
  result.Kcell = Kcell;
  return result;
}

static void logMeasurement(const ECraw& ec) {
  Serial.println(F("--------------------------------------------------"));
  Serial.print(F("EC gain: "));
  Serial.println(gainToString(ads.getGain()));

  Serial.print(F("Vexc="));
  Serial.print(vexc_rms, 6);
  Serial.print(F(" V  Vcell="));
  Serial.print(ec.Vcell, 6);
  Serial.print(F(" V  rho="));
  Serial.println(ec.rho, 6);

  Serial.print(F("Rcell="));
  Serial.print(ec.Rcell, 2);
  Serial.print(F(" ohm  EC_raw="));
  Serial.print(ec.EC_raw_mS, 5);
  Serial.println(F(" mS"));

  Serial.print(F("T="));
  Serial.print(ec.temperature, 2);
  Serial.print(F(" C  EC_comp="));
  Serial.print(ec.EC_comp_mS, 5);
  Serial.println(F(" mS"));

  Serial.print(F("Kcell="));
  Serial.print(isfinite(Kcell) ? Kcell : NAN, 6);
  Serial.println(F(" cm^-1"));

  Serial.print(F("pH_V="));
  Serial.print(ec.pHVoltage, 6);
  Serial.print(F(" V  pH_raw="));
  Serial.print(ec.pHRawValue, 3);
  Serial.print(F("  pH_comp="));
  Serial.println(ec.pHValue, 3);
  Serial.print(F("pH cal neutral="));
  Serial.print(ec.pHNeutralVoltage, 6);
  Serial.print(F(" V  slope25="));
  Serial.print(ec.pHSlope25, 6);
  Serial.println(F(" V/pH"));
  Serial.println(F("--------------------------------------------------"));
}

static void performEcCalibration(float kappaStd25C) {
  Serial.println(F("Starting EC calibration with standard solution"));
  refreshExcitationReference(true);

  double newK = calibrateKcellFromStd(kappaStd25C);
  Serial.print(F("New Kcell="));
  Serial.println(newK, 6);

  saveStoredKcell(newK);
}

static bool processCalibrationCommand(const CalibrationCommand& command) {
  switch (command.type) {
    case CalibrationCommandType::Ec:
      performEcCalibration(command.value);
      return true;

    case CalibrationCommandType::PhMid:
      return calibratePhNeutral(command.value);

    case CalibrationCommandType::PhLow:
      return calibratePhSlope(command.value, true);

    case CalibrationCommandType::PhHigh:
      return calibratePhSlope(command.value, false);

    case CalibrationCommandType::PhClear:
      clearStoredPhCalibration();
      return true;

    case CalibrationCommandType::None:
    default:
      return false;
  }
}

static void handleSerialCommand(int incoming) {
  char c = static_cast<char>(incoming);
  if (c == '\n' || c == '\r') {
    return;
  }

  switch (toupper(c)) {
    case 'C':
      performEcCalibration(KAPPA_STD_25C);
      break;
    case 'M':
      calibratePhNeutral(PH_NEUTRAL_DEFAULT);
      break;
    case 'L':
      calibratePhSlope(PH_LOW_DEFAULT, true);
      break;
    case 'H':
      calibratePhSlope(PH_HIGH_DEFAULT, false);
      break;
    case 'X':
      clearStoredPhCalibration();
      break;
    case 'P':
      logPhCalibrationState();
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(460800);
  delay(200);

  loadStoredKcell();
  loadStoredPhCalibration();

  startWifiConnection();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  initOledDisplay();
  drawOledStatus("WLAN Start", "Verbinde...", "", "");

  Serial.print("Warte auf WLAN");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  mqttInit();
  drawOledStatus("WLAN verbunden", WiFi.localIP().toString().c_str(), "MQTT startet", "");

  ArduinoOTA.setHostname("esp32-hydro_ec_sensor");
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA.onStart([]() { Serial.println("OTA: Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA: Ende"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: %u%%\r", (progress * 100) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("OTA bereit");
  drawOledStatus("OTA bereit", "Initialisiere", "Messhardware", "");

  delay(1000);
  Serial.println(F("Booting conductivity and pH monitor"));

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  delay(1000);

  AD.begin();
  AD.setFrequency(MD_AD9833::CHAN_0, 1000);
  AD.setMode(MD_AD9833::MODE_SINE);

  delay(300);

  if (!ads.begin(EC_ADS_ADDRESS)) {
    Serial.println(F("Failed to initialize EC ADS1115"));
    while (true) {
      delay(100);
    }
  }
  ads.setGain(GAIN_EIGHT);
  ads.setDataRate(RATE_ADS1115_250SPS);

  if (!adsPh.begin(PH_ADS_ADDRESS)) {
    Serial.println(F("Failed to initialize pH ADS1115"));
    while (true) {
      delay(100);
    }
  }
  adsPh.setGain(GAIN_ONE);
  adsPh.setDataRate(RATE_ADS1115_128SPS);
  delay(300);

  refreshExcitationReference(false);

  Serial.println(F("Hardware setup complete"));
  updateOledNoMeasurement();
}

void loop() {
  unsigned long now = millis();

  if (now - lastExcitationRefreshAt >= timing::VEXC_REFRESH_INTERVAL_MS) {
    refreshExcitationReference(true);
    now = millis();
  }

  if (now - lastMeasurementAt >= timing::MEASUREMENT_INTERVAL_MS) {
    lastMeasurementAt = now;

    ECraw measurement = measureEcRaw();
    appendMeasurement(measurement);
    lastMeasurement = measurement;
    hasLastMeasurement = true;
  }

  if (now - lastPublishAt >= timing::PUBLISH_INTERVAL_MS) {
    lastPublishAt = now;

    ECraw summary;
    if (buildMeasurementSummary(summary)) {
      logMeasurement(summary);
      network_publish_measurement(summary);
      clearMeasurementBuffer();
    }
  }

  if (Serial.available() > 0) {
    handleSerialCommand(Serial.read());
  }

  CalibrationCommand command;
  if (TryConsumeCalibrationCommand(command)) {
    if (processCalibrationCommand(command)) {
      clearMeasurementBuffer();
      now = millis();
      lastMeasurementAt = now;
      lastPublishAt = now;
    }
  }

  if (now - lastNetworkAt >= 1000) {
    lastNetworkAt = now;
    ensureWifiConnected(now);

    if (WiFi.status() == WL_CONNECTED) {
      ArduinoOTA.handle();
    }

    mqttLoop();
  }

  if (oledReady && (now - lastDisplayAt >= oledConfig::kRefreshIntervalMs)) {
    lastDisplayAt = now;
    if (hasLastMeasurement) {
      updateOledMeasurement(lastMeasurement);
    } else {
      updateOledNoMeasurement();
    }
  }
}
