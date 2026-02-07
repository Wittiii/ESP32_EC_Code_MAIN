#include "header.h"

#include "MQTTHandle.h"


Adafruit_ADS1115 ads;
MD_AD9833 AD(DATA_PIN, SCLK_PIN, FSYNC_PIN);




float V0_rms = 0.0f;
float vexc_rms = 0.0f;
double Kcell = NAN;

namespace timing {
constexpr uint32_t MEASUREMENT_INTERVAL_MS = 5000;
}  // namespace timing

static unsigned long lastMeasurementAt = 0;
static unsigned long last_At = 0;

static adsGain_t coarserGain(adsGain_t gain) {
  if (gain == GAIN_SIXTEEN) return GAIN_EIGHT;
  if (gain == GAIN_EIGHT)   return GAIN_FOUR;
  if (gain == GAIN_FOUR)    return GAIN_TWO;
  if (gain == GAIN_TWO)     return GAIN_ONE;
  return GAIN_ONE;
}

static adsGain_t finerGain(adsGain_t gain) {
  if (gain == GAIN_ONE)     return GAIN_TWO;
  if (gain == GAIN_TWO)     return GAIN_FOUR;
  if (gain == GAIN_FOUR)    return GAIN_EIGHT;
  if (gain == GAIN_EIGHT)   return GAIN_SIXTEEN;
  return GAIN_SIXTEEN;
}

static float clamp01(float value) {
  return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

static void selectExcitationPath() {
  digitalWrite(RELAY_PIN, HIGH);
  delay(T_SETTLE_MS);
}

static void selectCellPath() {
  digitalWrite(RELAY_PIN, LOW);
  delay(T_SETTLE_MS);
}

static int16_t readAdcAutoRange(uint8_t channel = 0) {
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

static float readAverageVoltage(uint8_t channel = 0, int samples = N_AVG) {
  const int maxSamples = 128;
  int n = samples;
  if (n < 8) n = 8;
  if (n > maxSamples) n = maxSamples;

  float vals[maxSamples];
  for (int i = 0; i < n; ++i) {
    int16_t reading = readAdcAutoRange(channel);
    vals[i] = ads.computeVolts(reading);
  }

  // einfache Sortierung (n ist klein)
  for (int i = 0; i < n - 1; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (vals[j] < vals[i]) {
        float t = vals[i];
        vals[i] = vals[j];
        vals[j] = t;
      }
    }
  }

  // unten/oben je 10% verwerfen
  int cut = n / 10;
  int start = cut;
  int end = n - cut;
  if (end <= start) return vals[n / 2];

  float sum = 0.0f;
  for (int i = start; i < end; ++i) sum += vals[i];
  return sum / static_cast<float>(end - start);
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

static float calcTemperature(){
  
  float U2 = readAverageVoltage(1, N_AVG);
  float Resitance2 = U2*R1_ref/(Uref-U2);
  //NTC10K 
  float temperature = 1.0f / (1.0f/T_0_K + (1.0f / B_CONST) * logf(Resitance2 / R_NTC_25C)) - 273.15f;
  return temperature;

  
}

static double calibrateKcellFromStd(float kappaStd25C) {
  
  float temperature = calcTemperature();
  if (temperature <= -100.0f || isnan(temperature)) {
    temperature = TEMP_REF;
  }

  float vCell = readAverageVoltage();
  float numerator = fabsf(vCell - V0_rms);
  float denominator = fmaxf(fabsf(vexc_rms - V0_rms), 1e-6f);
  float rho = clamp01(numerator / denominator);

  float oneMinus = fmaxf(1.0f - rho, 1e-6f);
  double rCell = Rref * (rho / oneMinus);
  double ecRawMilliSiemens = 1000.0 / rCell;

  double kappaAtTemperature = kappaStd25C * (1.0 + TEMP_COEF * (temperature - TEMP_REF));
  Kcell = kappaAtTemperature * (rCell / 1000.0);

  (void)ecRawMilliSiemens;
  return Kcell;
}

static ECraw measureEcRaw() {
  ECraw result;

  
  result.temperature = calcTemperature();
  if (result.temperature <= -100.0f || isnan(result.temperature)) {
    result.temperature = TEMP_REF;
  }

  result.Vcell = readAverageVoltage();
  float numerator = fabsf(result.Vcell - V0_rms);
  float denominator = fmaxf(fabsf(vexc_rms - V0_rms), 1e-6f);
  result.rho = clamp01(numerator / denominator);

  float oneMinus = fmaxf(1.0f - result.rho, 1e-6f);
  result.Rcell = Rref * (result.rho / oneMinus);
  result.EC_raw_mS = 1000.0 / result.Rcell;

  float tempFactor = 1.0f + TEMP_COEF * (result.temperature - TEMP_REF);
  double ecMeasured = result.EC_raw_mS * (isfinite(Kcell) ? Kcell : 1.0);
  result.EC_comp_mS = ecMeasured / tempFactor;

  return result;
}

static void logMeasurement(const ECraw& ec) {
  Serial.println(F("--------------------------------------------------"));
  Serial.print(F("Gain: "));
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
  Serial.println(F("--------------------------------------------------"));
}

static void performCalibration() {
  Serial.println(F("Starting calibration with standard solution"));
  selectExcitationPath();
  vexc_rms = readAverageVoltage();
  selectCellPath();
  delay(1000);

  double newK = calibrateKcellFromStd(KAPPA_STD_25C);
  Serial.print(F("New Kcell="));
  Serial.println(newK, 6);
 
}

static void handleSerialCommand(int incoming) {
  char c = static_cast<char>(incoming);
  if (c == '\n' || c == '\r') {
    return;
  }

  if (toupper(c) == 'C') {
    performCalibration();
  }
}

void setup() {


  Serial.begin(460800);
  WiFi.begin(ssid, pass);
  Serial.print("Warte auf WLAN");
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  mqttInit();

  
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

  delay(1000);
  Serial.println(F("Booting conductivity monitor"));

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  
  delay(1000);

  Wire.begin(8, 9);

  delay(100);
  AD.begin();
  AD.setFrequency(MD_AD9833::CHAN_0, 1000);
  AD.setMode(MD_AD9833::MODE_SINE);

  delay(300);

  if (!ads.begin()) {
    Serial.println(F("Failed to initialize ADS1115"));
    while (true) {
      delay(100);
    }
  }
  ads.setGain(GAIN_EIGHT);
  ads.setDataRate(RATE_ADS1115_250SPS);
  delay(300);

  selectExcitationPath();

  vexc_rms = readAverageVoltage();
  selectCellPath();

  Serial.println(F("Hardware setup complete"));


}

void loop() {

  unsigned long now = millis();
  if (now - lastMeasurementAt >= timing::MEASUREMENT_INTERVAL_MS) {
    lastMeasurementAt = now;

    ECraw measurement = measureEcRaw();
    logMeasurement(measurement);
    network_publish_measurement(measurement);
  }

  if (Serial.available() > 0) {
    handleSerialCommand(Serial.read());
  }

  if (now - last_At >= 1000) {
    last_At = now;
    ArduinoOTA.handle(); // wichtig
    delay(10);
    mqttLoop();//Blockt den Code bis die Verbindung steht, also kein OTA in der Zeit möglich
    Serial.println("Loop Done");
  }

}