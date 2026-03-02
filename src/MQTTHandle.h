

#pragma once
void mqttInit();
void mqttLoop();
bool DoCalibration();
float GetCalibrationValue();
void network_publish_measurement(ECraw measurement);

