#ifndef USER_INPUT_H
#define USER_INPUT_H
#define DURATION_BUTTON_HOLD   750 // [ms]
#define ENCODER_SLOW_DISTANCE 0.05 // [mm per encoder step]
#define ENCODER_FAST_DISTANCE 1.0 // [mm per encoder step]

#include <ESP32Encoder.h>
#include "PinDefinitions.h"

extern ESP32Encoder encoder;
extern float mm_per_step();
extern long input_encoder_steps;
extern bool input_toolchange_press;
extern bool input_toolchange_release;
extern bool input_goto_bottom_press;
extern bool input_goto_bottom_hold;
extern bool input_goto_bottom_release;
extern long  preference_motor_speed_maximal;
extern long  preference_motor_acceleration;
extern long  preference_motor_steps_per_revolution; 
extern long  preference_motor_direction;
extern float preference_motor_thread_pitch;
extern long  preference_motor_steps_slow;
extern long  preference_motor_steps_fast;
extern long  preference_motor_speed_toolchange;
extern long  preference_auto_zero_speed;
extern float preference_workspace_height;
extern bool  preference_power_on_toolchange;
extern float preference_sensor_tool_length_height;
extern bool input_set_zero_press;
extern bool input_set_zero_hold;
extern bool input_set_zero_release;
extern bool input_set_speed_press;
extern bool input_set_speed_hold;
extern bool input_set_speed_release;
extern long  default_motor_steps_slow;
extern long  default_motor_steps_fast;
extern long status_settings_menu_active_page;
extern long status_settings_menu_pages_count;
extern int i;

void collect_goto_bottom_input();
void collect_set_zero_input();
void collect_set_speed_input();

void collect_inputs() {
  // encoder steps since last check
  input_encoder_steps = encoder.getCount();
  encoder.clearCount();

  // toolchange button pressed
  input_toolchange_press = input_toolchange_release && !digitalRead(PIN_BUTTON_TOOLCHANGE);
  // collect release signal to avoid double press
  input_toolchange_release = digitalRead(PIN_BUTTON_TOOLCHANGE);


  collect_goto_bottom_input();
  collect_set_zero_input();
  collect_set_speed_input();
}

void collect_goto_bottom_input() {
  // goto_bottom button pressed or hold
  input_goto_bottom_press = false;
  input_goto_bottom_hold  = false;
  // count how long button is pressed
  for (i = 0; input_goto_bottom_release && !digitalRead(PIN_BUTTON_GOTO_BOTTOM); i++) {
    delay(1);
    // break when over threshold
    if (i > DURATION_BUTTON_HOLD) {
      break;
    }
  }
  // check if released
  input_goto_bottom_release = digitalRead(PIN_BUTTON_GOTO_BOTTOM);
  // if button was touched
  if (i > 0) {
    // was button released
    if (input_goto_bottom_release) {
      input_goto_bottom_press = true;
      // or was over threshold
    } else {
      input_goto_bottom_hold  = true;
    }
  }
}

void collect_set_zero_input() {
  // set_zero can be pressed or hold
  input_set_zero_press = false;
  input_set_zero_hold  = false;
  // count how long button is pressed
  for (i = 0; input_set_zero_release && !digitalRead(PIN_BUTTON_SET_ZERO); i++) {
    delay(1);
    // break when over threshold
    if (i > DURATION_BUTTON_HOLD) {
      break;
    }
  }
  // check if released
  input_set_zero_release = digitalRead(PIN_BUTTON_SET_ZERO);
  // if button was touched
  if (i > 0) {
    // was button released
    if (input_set_zero_release) {
      input_set_zero_press = true;
      // or was over threshold
    } else {
      input_set_zero_hold  = true;
    }
  }
}

void collect_set_speed_input() {
  // set_speed can be pressed or hold
  input_set_speed_press = false;
  input_set_speed_hold  = false;
  // count how long button is pressed
  for (i = 0; input_set_speed_release && !digitalRead(PIN_BUTTON_SET_SPEED); i++) {
    delay(1);
    // break when over threshold
    if (i > DURATION_BUTTON_HOLD) {
      break;
    }
  }
  // check if released
  input_set_speed_release = digitalRead(PIN_BUTTON_SET_SPEED);
  // if button was touched
  if (i > 0) {
    // was button released
    if (input_set_speed_release) {
      input_set_speed_press = true;
      // or was over threshold
    } else {
      input_set_speed_hold  = true;
    }
  }
}

bool consume_input_toolchange_press() {
  if (input_toolchange_press) {
    Serial.println("input_toolchange_press");
    input_toolchange_press = false;
    return true;
  }
  return false;
}

bool consume_input_goto_bottom_press() {
  if (input_goto_bottom_press) {
    Serial.println("input_goto_bottom_press");
    input_goto_bottom_press = false;
    return true;
  }
  return false;
}

bool consume_input_goto_bottom_hold() {
  if (input_goto_bottom_hold) {
    Serial.println("input_goto_bottom_hold");
    input_goto_bottom_hold = false;
    return true;
  }
  return false;
}

bool consume_input_set_zero_press() {
  if (input_set_zero_press) {
    Serial.println("input_set_zero_press");
    input_set_zero_press = false;
    return true;
  }
  return false;
}

bool consume_input_set_zero_hold() {
  if (input_set_zero_hold) {
    Serial.println("input_set_zero_hold");
    input_set_zero_hold = false;
    return true;
  }
  return false;
}
  
bool consume_input_set_speed_press() {
  if (input_set_speed_press) {
    Serial.println("input_set_speed_press");
    input_set_speed_press = false;
    return true;
  }
  return false;
}
  
bool consume_input_set_speed_hold() {
  if (input_set_speed_hold) {
    Serial.println("input_set_speed_hold");
    input_set_speed_hold = false;
    return true;
  }
  return false;
}
void reset_settings_to_default(Preferences& preferences);

void handle_settings_menu_change(Preferences& preferences) {
  switch (status_settings_menu_active_page) {
    case 0:
      preference_motor_speed_maximal += input_encoder_steps * 10;
      if (preference_motor_speed_maximal < 0) {
        preference_motor_speed_maximal = 0;
      }
      preferences.putLong64("motor_speed_max", preference_motor_speed_maximal);
      break;
    case 1:
      preference_motor_acceleration += input_encoder_steps * 10;
      if (preference_motor_acceleration < 0) {
        preference_motor_acceleration = 0;
      }
      preferences.putLong64("motor_acc", preference_motor_acceleration);
      break;
    case 2:
      preference_motor_steps_per_revolution += input_encoder_steps * 100;
      if (preference_motor_steps_per_revolution < 0) {
        preference_motor_steps_per_revolution = 0;
      }
      preferences.putLong64("steps_per_rev", preference_motor_steps_per_revolution);
      break;
    case 3:
      if (preference_motor_direction == -1) {
        preference_motor_direction = 1;
      } else {
        preference_motor_direction = -1;
      }
      preferences.putLong64("motor_dir", preference_motor_direction);
      break;
    case 4:
      preference_sensor_tool_length_height += (float)input_encoder_steps / 10;
      preferences.putFloat("tlsensor_height", preference_sensor_tool_length_height);
      break;
    case 5:
      preference_motor_thread_pitch += (float)input_encoder_steps / 10;
      if (preference_motor_thread_pitch < 0) {
        preference_motor_thread_pitch = 0;
      }
      preferences.putFloat("thread_pitch", preference_motor_thread_pitch);
      break;
    case 6:
      preference_motor_steps_slow += input_encoder_steps * round(ENCODER_SLOW_DISTANCE / mm_per_step());
      if (preference_motor_steps_slow < default_motor_steps_slow) {
        preference_motor_steps_slow = default_motor_steps_slow;
      }
      preferences.putLong64("steps_slow", preference_motor_steps_slow);
      break;
    case 7:
      preference_motor_steps_fast += input_encoder_steps * round(ENCODER_FAST_DISTANCE / mm_per_step());
      if (preference_motor_steps_fast < default_motor_steps_fast) {
        preference_motor_steps_fast = default_motor_steps_fast;
      }
      preferences.putLong64("steps_fast", preference_motor_steps_fast);
      break;
    case 8:
      preference_motor_speed_toolchange += input_encoder_steps * 10;
      if (preference_motor_speed_toolchange < 0) {
        preference_motor_speed_toolchange = 0;
      }
      preferences.putLong64("speed_toolch", preference_motor_speed_toolchange);
      break;
    case 9:
      preference_auto_zero_speed += input_encoder_steps * 10;
      if (preference_auto_zero_speed < 0) {
        preference_auto_zero_speed = 0;
      }
      preferences.putLong64("auto_zero_speed", preference_auto_zero_speed);
      break;
    case 10:
      preference_workspace_height += (float)input_encoder_steps / 10;
      if (preference_workspace_height < 0) {
        preference_workspace_height = 0;
      }
      preferences.putFloat("ws_height", preference_workspace_height);
      break;
    case 11:
      preference_power_on_toolchange = !preference_power_on_toolchange;
      preferences.putBool("pwr_on_toolch", preference_power_on_toolchange);
      break;
    case 12:
      preference_sensor_tool_length_enabled_normally_closed = !preference_sensor_tool_length_enabled_normally_closed;
      preferences.putBool("tlsensor_en_n_c", preference_sensor_tool_length_enabled_normally_closed);
      break;
    case 13:
      preference_sensor_tool_length_normally_closed = !preference_sensor_tool_length_normally_closed;
      preferences.putBool("tlsensor_n_c", preference_sensor_tool_length_normally_closed);
      break;
    case 14:
      preference_sensor_end_stop_normally_closed  = !preference_sensor_end_stop_normally_closed ;
      preferences.putBool("end_stop_n_c ", preference_sensor_end_stop_normally_closed );
      break;
    default:
      Serial.println("invalid menu page number: " + String(status_settings_menu_active_page));
  }
}

#endif // USER_INPUT_H