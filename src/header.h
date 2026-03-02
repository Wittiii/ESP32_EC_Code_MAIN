#pragma once


#include "Arduino.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "Wire.h"

#include <math.h>
#include <MD_AD9833.h>
#include <Adafruit_ADS1X15.h>



const char* ssid = "FRITZ!Box 7590 BK";
const char* pass = "35428536880518248119";
const char* ota_password = "1234";

inline constexpr uint8_t FSYNC_PIN = 1;
inline constexpr uint8_t DATA_PIN = 7;
inline constexpr uint8_t SCLK_PIN = 4;
inline constexpr uint8_t RELAY_PIN = 2;
inline constexpr uint8_t ONE_WIRE_BUS = 3;

inline constexpr float Rref = 600.0f;
inline constexpr int N_AVG = 64;
inline constexpr int T_SETTLE_MS = 4000;
inline constexpr float KAPPA_STD_25C = 1.413f;
inline constexpr float TEMP_COEF = 0.02f;
inline constexpr float TEMP_REF = 25.0f;
inline constexpr float HI_FRAC = 0.85f;
inline constexpr float LO_FRAC = 0.18f;

extern Adafruit_ADS1115 ads;
extern MD_AD9833 AD;

inline constexpr float Uref = 3.3f;
inline constexpr float R1_ref=13000.0f;
inline constexpr float T_0_K = 298.15f; // 25C in Kelvin 
inline constexpr float  B_CONST = 3950.0f;
inline constexpr float R_NTC_25C = 10000.0f;


extern float V0_rms;
extern float vexc_rms;
extern double Kcell;

struct ECraw {
  float Vcell = 0.0f;
  float rho = 0.0f;
  double Rcell = 0.0;
  double EC_raw_mS = 0.0;
  float temperature = NAN;
  double EC_comp_mS = 0.0;
  double Kcell = NAN;
};