void reset_settings_to_default() {
  preferences.clear();
  read_settings();
}

void read_settings() {
  preference_motor_steps_per_revolution = preferences.getLong64("steps_per_rev", default_motor_steps_per_revolution);
  preference_motor_thread_pitch         = preferences.getFloat("thread_pitch", default_motor_thread_pitch);
  preference_motor_steps_slow           = preferences.getLong64("steps_slow", default_motor_steps_slow);
  preference_motor_steps_fast           = preferences.getLong64("steps_fast", default_motor_steps_fast);
  preference_motor_direction            = preferences.getLong64("motor_dir", default_motor_direction);
  preference_motor_speed_maximal        = preferences.getLong64("motor_speed_max", default_motor_speed_maximal);
  preference_motor_speed_toolchange     = preferences.getLong64("speed_toolch", default_motor_speed_toolchange);
  preference_motor_acceleration         = preferences.getLong64("motor_acc", default_motor_acceleration);

  preference_sensor_end_stop_normally_closed = preferences.getBool("end_stop_n_c", default_sensor_end_stop_normally_closed);

  preference_sensor_tool_length_normally_closed         = preferences.getBool("tlsensor_n_c", default_sensor_tool_length_normally_closed);
  preference_sensor_tool_length_enabled_normally_closed = preferences.getBool("tlsensor_en_n_c", default_sensor_tool_length_enabled_normally_closed);
  preference_sensor_tool_length_height                  = preferences.getFloat("tlsensor_height", default_sensor_tool_length_height);

  preference_workspace_height = preferences.getFloat("ws_height", default_workspace_height);

  preference_power_on_toolchange = preferences.getBool("pwr_on_toolch", default_power_on_toolchange);

  preference_auto_zero_speed = preferences.getLong64("auto_zero_speed", default_auto_zero_speed);
}
