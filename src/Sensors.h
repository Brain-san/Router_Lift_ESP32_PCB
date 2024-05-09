#ifndef SENSORS_H
#define SENSORS_H

extern bool  preference_sensor_tool_length_normally_closed;
extern bool  preference_sensor_tool_length_enabled_normally_closed;
extern bool  preference_sensor_end_stop_normally_closed;

// SENSOR VALUES START
bool read_sensor_end_stop_trigger() {
  return (!digitalRead(PIN_SENSOR_END_STOP_TRIGGER)) ^ preference_sensor_end_stop_normally_closed;
}

bool read_sensor_tool_length_enabled() {
  return (!digitalRead(PIN_SENSOR_TOOL_LENGTH_ENABLED)) ^ preference_sensor_tool_length_enabled_normally_closed;
}

bool read_sensor_tool_length_trigger() {
  return (!digitalRead(PIN_SENSOR_TOOL_LENGTH_TRIGGER)) ^ preference_sensor_tool_length_normally_closed;
}
// SENSOR VALUES END

#endif // SENSORS_H