#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// STATE MACHINE STUFF START
enum states {
  default_start,
  goto_tool_length_sensor,
  finish_tool_length_sensor,
  goto_toolchange,
  finish_toolchange,
  settings_menu,
  reset,
  error
};

// STATE MACHINE STUFF END
enum states current_state = default_start;

#endif // STATE_MACHINE_H