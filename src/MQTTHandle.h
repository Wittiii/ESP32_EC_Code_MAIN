

#pragma once

#include "header.h"

enum class CalibrationCommandType {
  None,
  Ec,
  PhMid,
  PhLow,
  PhHigh,
  PhClear,
};

struct CalibrationCommand {
  CalibrationCommandType type = CalibrationCommandType::None;
  float value = NAN;
};

void mqttInit();
void mqttLoop();
bool TryConsumeCalibrationCommand(CalibrationCommand& command);
void network_publish_measurement(const ECraw& measurement);

