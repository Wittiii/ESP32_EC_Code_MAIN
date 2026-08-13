#include "MQTTHandle.h"

#include "WiFi.h"
#include "PubSubClient.h"

WiFiClient ESP32Client_Hydroponik;
PubSubClient client(ESP32Client_Hydroponik);

namespace mqttConfig {
constexpr char kServer[] = "192.168.178.56";
constexpr int kPort = 1883;
constexpr char kUser[] = "pwi";
constexpr char kPassword[] = "1234";
constexpr char kPublishTopic[] = "esp32_Hydroponic/ec_ph";
constexpr char kSubscribeTopic[] = "esp32_Hydroponic/ec_ph_cmd";
constexpr unsigned long kReconnectIntervalMs = 5000;
}

static unsigned long lastReconnectAttempt = 0;
static char msg[384];
static CalibrationCommand pendingCommand;
static bool hasPendingCommand = false;

static bool tryParseFloatCommand(const String& message, const char* prefix, CalibrationCommandType type) {
  if (!message.startsWith(prefix)) {
    return false;
  }

  String valueText = message.substring(strlen(prefix));
  valueText.trim();
  if (valueText.isEmpty()) {
    return false;
  }

  pendingCommand.type = type;
  pendingCommand.value = valueText.toFloat();
  hasPendingCommand = true;
  return true;
}

static void handleCommandMessage(const String& message) {
  if (tryParseFloatCommand(message, "Calibrate:", CalibrationCommandType::Ec) ||
      tryParseFloatCommand(message, "ECCalibrate:", CalibrationCommandType::Ec) ||
      tryParseFloatCommand(message, "PHMid:", CalibrationCommandType::PhMid) ||
      tryParseFloatCommand(message, "PHCalMid:", CalibrationCommandType::PhMid) ||
      tryParseFloatCommand(message, "PHLow:", CalibrationCommandType::PhLow) ||
      tryParseFloatCommand(message, "PHCalLow:", CalibrationCommandType::PhLow) ||
      tryParseFloatCommand(message, "PHHigh:", CalibrationCommandType::PhHigh) ||
      tryParseFloatCommand(message, "PHCalHigh:", CalibrationCommandType::PhHigh)) {
    return;
  }

  if (message.equalsIgnoreCase("PHClear") || message.equalsIgnoreCase("PHCalClear")) {
    pendingCommand.type = CalibrationCommandType::PhClear;
    pendingCommand.value = NAN;
    hasPendingCommand = true;
  }
}

static void reconnect() {
  unsigned long now = millis();
  if (now - lastReconnectAttempt < mqttConfig::kReconnectIntervalMs) {
    return;
  }
  lastReconnectAttempt = now;

  if (client.connect("ESP32Client_Hydroponik_EC_PH-Sensor", mqttConfig::kUser, mqttConfig::kPassword)) {
    client.subscribe(mqttConfig::kSubscribeTopic);
    lastReconnectAttempt = 0;
  }
}

static void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, mqttConfig::kSubscribeTopic) != 0) {
    return;
  }

  String message;
  for (unsigned int i = 0; i < length; ++i) {
    message += static_cast<char>(payload[i]);
  }
  message.trim();
  handleCommandMessage(message);
}

bool TryConsumeCalibrationCommand(CalibrationCommand& command) {
  if (!hasPendingCommand) {
    return false;
  }

  command = pendingCommand;
  pendingCommand = CalibrationCommand{};
  hasPendingCommand = false;
  return true;
}

void mqttInit() {
  client.setServer(mqttConfig::kServer, mqttConfig::kPort);
  client.setCallback(callback);
}

void network_publish_measurement(const ECraw& measurement) {
  if (!client.connected()) {
    reconnect();
    return;
  }

  client.loop();

  String payload = "Vcell=" + String(measurement.Vcell, 3) + "V, " +
                   "rho=" + String(measurement.rho, 3) + ", " +
                   "Rcell=" + String(measurement.Rcell, 2) + "Ohm, " +
                   "EC_raw=" + String(measurement.EC_raw_mS, 5) + "mS, " +
                   "Temp=" + String(measurement.temperature, 2) + "C, " +
                   "EC_comp=" + String(measurement.EC_comp_mS, 5) + "mS, " +
                   "Kcell=" + String(isfinite(measurement.Kcell) ? measurement.Kcell : NAN, 6) + "cm^-1, " +
                   "PH_V=" + String(measurement.pHVoltage, 4) + "V, " +
                   "PH_raw=" + String(measurement.pHRawValue, 3) + ", " +
                   "PH_comp=" + String(measurement.pHValue, 3) + ", " +
                   "PH=" + String(measurement.pHValue, 3) + ", " +
                   "PH_neutral_V=" + String(measurement.pHNeutralVoltage, 5) + "V, " +
                   "PH_slope25=" + String(measurement.pHSlope25, 6) + "V/pH";
  payload.toCharArray(msg, sizeof(msg));
  client.publish(mqttConfig::kPublishTopic, msg);
}

void mqttLoop() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!client.connected()) {
    reconnect();
    return;
  }

  client.loop();
}
