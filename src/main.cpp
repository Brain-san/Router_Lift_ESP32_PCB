/*
    Router Lift Control with ESP32
    Version 1.0
    Copyright (C) 2021  https://github.com/SilvanCodes

    The suggestion for this project is based on the work of:
    Frohnix Bastelbude -> https://www.youtube.com/user/Frohnix
    The display part (OLED I2C) was imported from this code:
    Frohnix Bastelbude  ->  https://drive.google.com/file/d/1x_Z-x_cdAlwg_KYfx1PN03zCVg7Hu3KM/view


    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Preferences.h>
#include <AccelStepper.h>
#include <ESP32Encoder.h>
#include <U8g2lib.h>
#include <Display.h>
#include <Motor.h>
#include <Sensors.h>
#include <Settings.h>
#include <UserInput.h>

#define PIN_STEP      19
#define PIN_DIRECTION 15

#define PIN_BUTTON_UP          23
#define PIN_BUTTON_DOWN        18
#define PIN_BUTTON_TOOLCHANGE  32
#define PIN_BUTTON_SET_ZERO    13
#define PIN_BUTTON_SET_SPEED    4
#define PIN_BUTTON_GOTO_BOTTOM 25

#define PIN_ENCODER_A_MINUS 14
#define PIN_ENCODER_B_MINUS 27

#define PIN_ICC_DATA 21
#define PIN_ICC_CLOCK 22

#define PIN_SENSOR_END_STOP_TRIGGER    16
#define PIN_SENSOR_TOOL_LENGTH_TRIGGER 17
#define PIN_SENSOR_TOOL_LENGTH_ENABLED  5

#define DURATION_BUTTON_HOLD   750 // [ms]

#define DURATION_SHOW_MESSAGE 1000 // [ms]

#define MM_TO_FREE_ERROR 3.0 // [mm]

#define ERROR_END_STOP      "ENDSTOP ERR"
#define ERROR_AUTO_ZERO     "AUTOZERO ERR"
#define ERROR_INVALID_STATE "INVALID STATE"

#define ENCODER_SLOW_DISTANCE 0.05 // [mm per encoder step]
#define ENCODER_FAST_DISTANCE 1.0 // [mm per encoder step]

#define FREE_SENSOR_TOLERANCE 30 // [steps]

// PERIPHERY START
Preferences preferences;

AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIRECTION);

ESP32Encoder encoder;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display_I2C(U8G2_R0, U8X8_PIN_NONE, PIN_ICC_CLOCK, PIN_ICC_DATA);
// PERIPHERY END

// MISC. VARIABLES START
int i  =  0; // general counter
int ux = 57; // helper to draw target circle
int uy = 48; // helper to draw target circle

float pos_in_mm = 0.0; // holds computed value

long  start_pos = 0  ; // for error detection
long  max_pos   = 0  ; // for error detection
// MISC. VARIABLES END

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
enum states current_state = default_start;
// STATE MACHINE STUFF END

// INPUT VALUES START
bool input_up_press   = false;
bool input_down_press = false;

bool input_toolchange_press   = false;
bool input_toolchange_release = true;

bool input_set_zero_press = false;
bool input_set_zero_hold  = false;
bool input_set_zero_release = true;

bool input_set_speed_press = false;
bool input_set_speed_hold  = false;
bool input_set_speed_release = true;

bool input_goto_bottom_press = false;
bool input_goto_bottom_hold  = false;
bool input_goto_bottom_release = true;

long input_encoder_steps = 0; // is non-zero when encoder steps occurred since last input processing
// INPUT VALUES END

// PREFERENCE VALUES START
long  default_motor_steps_per_revolution = 1600  ; // [steps per revolution]
float default_motor_thread_pitch         =    8.0; // [mm per revolution]
long  default_motor_steps_slow           = round(0.05 / (default_motor_thread_pitch / default_motor_steps_per_revolution))  ; // [steps per encoder step]
long  default_motor_steps_fast           = round(1.0 / (default_motor_thread_pitch / default_motor_steps_per_revolution))  ; // [steps per encoder step]
long  default_motor_direction            = -1  ; // [-1 or 1]
long  default_motor_speed_maximal        = default_motor_steps_per_revolution  ; // [steps per second]
long  default_motor_speed_toolchange     = default_motor_steps_per_revolution  ; // [steps per second]
long  default_motor_acceleration         = default_motor_steps_per_revolution >> 2  ; // [steps per second per second]

bool  default_sensor_end_stop_normally_closed = false;

bool  default_sensor_tool_length_enabled_normally_closed = false;
bool  default_sensor_tool_length_normally_closed = false;
float default_sensor_tool_length_height = 0.0; // mm

float default_workspace_height = 75.0; // mm

bool  default_power_on_toolchange = false;

long  default_auto_zero_speed = 1600; // [steps per second]

long  preference_motor_steps_per_revolution; // [steps per revolution]
float preference_motor_thread_pitch;         // [mm per revolution]
long  preference_motor_steps_slow;           // [steps per encoder step]
long  preference_motor_steps_fast;           // [steps per encoder step]
long  preference_motor_direction;            // [-1 or 1]
long  preference_motor_speed_maximal;        // [steps per second]
long  preference_motor_speed_toolchange;     // [steps per second]
long  preference_motor_acceleration;         // [steps per second per second]

bool  preference_sensor_end_stop_normally_closed;
bool  preference_sensor_tool_length_enabled_normally_closed;
bool  preference_sensor_tool_length_normally_closed;
float preference_sensor_tool_length_height; // mm

float preference_workspace_height; // mm

bool  preference_power_on_toolchange;

long  preference_auto_zero_speed; // [steps per second]
// PREFERENCE VALUES END

// STATUS VALUES START
bool  status_motor_mode_constant = false;

bool  status_slow_speed  = false;
long  status_slow_offset = 0;

bool  status_target_active = false;
float status_target_height = 0.0; // mm
float status_target_lower_limit = 0.0; // steps

bool  status_workspace_active = false;
long  status_workspace_upper_limit = 0; // steps
long  status_workspace_lower_limit = 0; // steps

long status_settings_menu_active_page =  0;
long status_settings_menu_pages_count = 15;

char *status_error_message = "";
// STATUS VALUES END

// COMPUTED VALUES START
float mm_per_step() {
  return preference_motor_thread_pitch / preference_motor_steps_per_revolution; // [mm per step]
}

float position_in_mm() {
  return round(stepper.currentPosition() * mm_per_step() * 100) / 100 * preference_motor_direction; // [mm]
}
// COMPUTED VALUES END

void setup() {
  Serial.begin(115200);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIRECTION,  OUTPUT);

  pinMode(PIN_BUTTON_UP,          INPUT_PULLUP);
  pinMode(PIN_BUTTON_DOWN,        INPUT_PULLUP);
  pinMode(PIN_BUTTON_TOOLCHANGE,  INPUT_PULLUP);
  pinMode(PIN_BUTTON_SET_ZERO,    INPUT_PULLUP);
  pinMode(PIN_BUTTON_SET_SPEED,   INPUT_PULLUP);
  pinMode(PIN_BUTTON_GOTO_BOTTOM, INPUT_PULLUP);

  pinMode(PIN_SENSOR_END_STOP_TRIGGER,    INPUT_PULLUP);
  pinMode(PIN_SENSOR_TOOL_LENGTH_TRIGGER, INPUT_PULLUP);
  pinMode(PIN_SENSOR_TOOL_LENGTH_ENABLED, INPUT_PULLUP);

  // PREFERENCES SETUP START
  preferences.begin("settings", false);
  read_settings();
  // PREFERENCES SETUP END

  // STEPPER MOTOR SETUP START
  // speed unit is [steps per second]
  stepper.setMaxSpeed(preference_motor_speed_maximal);
  stepper.setAcceleration(preference_motor_acceleration);
  stepper.setCurrentPosition(0);
  // STEPPER MOTOR SETUP END

  // ENCODER SETUP START
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(PIN_ENCODER_A_MINUS, PIN_ENCODER_B_MINUS);
  encoder.clearCount();
  // ENCODER SETUP END

  // DISPLAY SETUP START
  xTaskCreatePinnedToCore(
    draw_loop, /* Function to implement the task */
    "DisplayTask", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    2,  /* Priority of the task */
    NULL,  /* Task handle. */
    0); /* Core where the task should run */
  // DISPLAY SETUP END

  if (preference_power_on_toolchange) {
    change_state_to(goto_toolchange);
  }
}

void loop() {
  switch (current_state) {
    case default_start:
      // FREE END STOP SENSOR
      if (read_sensor_end_stop_trigger()) {
        if (!free_sensor_end_stop()) {
          error_with(ERROR_END_STOP);
          break;
        }
      }

      // MOVE WITH ENCODER
      if (input_encoder_steps) {
        if (status_slow_speed) {
          // move according steps and make sure it i always a multiple of preference_motor_steps_slow
          stepper.move(input_encoder_steps * preference_motor_steps_slow * preference_motor_direction - (stepper.currentPosition() % preference_motor_steps_slow));
        } else {
          // move according steps and make sure it i always a multiple of preference_motor_steps_fast plus finer position from moving with preference_motor_steps_slow
          stepper.move(input_encoder_steps * preference_motor_steps_fast * preference_motor_direction - (stepper.currentPosition() % preference_motor_steps_fast) + status_slow_offset);
        }
        Serial.println("targetPosition(): " + String(stepper.targetPosition()));
        input_encoder_steps = 0;
      }

      // GO TO WORKSPACE MINIMUM
      if (consume_input_goto_bottom_press()) {
        if (status_workspace_active) {
          stepper.moveTo(status_workspace_upper_limit);
        }
      }

      // TOGGLE TARGET
      if (consume_input_set_speed_hold()) {
        toggle_target();
      }

      // MOVE UP
      if (!digitalRead(PIN_BUTTON_UP)) {
        stepper.setSpeed(preference_motor_speed_maximal * preference_motor_direction);
        while (!digitalRead(PIN_BUTTON_UP)) {
          move_motor_constant();
        }
        halt_motor();
      }

      // MOVE DOWN
      if (!digitalRead(PIN_BUTTON_DOWN)) {
        stepper.setSpeed(-preference_motor_speed_maximal * preference_motor_direction);
        while (!digitalRead(PIN_BUTTON_DOWN)) {
          move_motor_constant();
        }
        halt_motor();
      }

      // TOGGLE SPEED
      if (consume_input_set_speed_press()) {
        status_slow_speed = !status_slow_speed;
        status_slow_offset = stepper.targetPosition() % preference_motor_steps_fast;
        stepper.setAcceleration(preference_motor_acceleration >> status_slow_speed);
        stepper.setMaxSpeed(preference_motor_speed_maximal >> status_slow_speed);
      }

      // MOVE ACCORDING TO ENCODER
      move_motor_accelerate();

      // STATE CHANGES
      if (consume_input_toolchange_press()) {
        change_state_to(goto_toolchange);
      } else if (consume_input_set_zero_press()) {
        if (read_sensor_tool_length_enabled()) {
          change_state_to(goto_tool_length_sensor);
        } else {
          set_zero();
        }
      } else if (consume_input_set_zero_hold()) {
        change_state_to(settings_menu);
      }

      break;
    case goto_tool_length_sensor:
      // keep moving
      if (!move_motor_constant()) {
        error_with(ERROR_AUTO_ZERO);
      } else if (read_sensor_tool_length_trigger()) {
        change_state_to(finish_tool_length_sensor);
      }

      break;
    case finish_tool_length_sensor:
      // move until at target or not allowed
      if (is_motor_at_target()) {
        set_zero();
        change_state_to(default_start);
      } else if (!move_motor_accelerate()) {
        error_with(ERROR_AUTO_ZERO);
      }
      break;
    case goto_toolchange:
      // keep moving
      move_motor_constant();

      // STATE CHANGES
      if (read_sensor_end_stop_trigger()) {
        change_state_to(finish_toolchange);
      } else if (consume_input_toolchange_press()) {
        change_state_to(default_start);
      }
      break;
    case finish_toolchange:
      change_state_to(default_start);
      break;
    case settings_menu:
      if (consume_input_toolchange_press()) {
        status_settings_menu_active_page -= 1;
        status_settings_menu_active_page = (status_settings_menu_active_page + status_settings_menu_pages_count) % status_settings_menu_pages_count;
      } else if (consume_input_set_zero_press()) {
        status_settings_menu_active_page += 1;
        status_settings_menu_active_page = (status_settings_menu_active_page + status_settings_menu_pages_count) % status_settings_menu_pages_count;
      }

      if (input_encoder_steps) {
        handle_settings_menu_change();
        input_encoder_steps = 0;
      }

      if (consume_input_set_zero_hold()) {
        change_state_to(default_start);
      } else if (consume_input_goto_bottom_hold()) {
        change_state_to(reset);
      }
      break;
    case reset:
      change_state_to(default_start);
      break;
    case error:
      halt_motor();
      if (consume_input_set_zero_hold()) {
        change_state_to(settings_menu);
      }
      break;
    default:
      error_with(ERROR_INVALID_STATE);
  }
}

void change_state_to(enum states new_state) {
  if (run_state_exit(current_state) && run_state_entry(new_state)) {
    current_state = new_state;
  }
}

bool run_state_entry(enum states state) {
  switch (state) {
    case goto_toolchange:
      Serial.println("entry goto_toolchange");
      deactivate_target();
      deactivate_workspace();
      stepper.setSpeed(preference_motor_speed_maximal * preference_motor_direction);
      return true;
    case finish_toolchange:
      Serial.println("entry finish_toolchange");
      if (!free_sensor_end_stop()) {
        error_with(ERROR_END_STOP);
        return false;
      };
      activate_workspace();
      return true;
    case goto_tool_length_sensor:
      Serial.println("entry goto_tool_length_sensor");
      deactivate_target();
      stepper.setSpeed(preference_auto_zero_speed * preference_motor_direction);
      return true;
    case finish_tool_length_sensor:
      Serial.println("entry finish_tool_length_sensor");
      if (!free_sensor_tool_length()) {
        error_with(ERROR_AUTO_ZERO);
        return false;
      };
      // set target position at tool_length_height
      stepper.move(-preference_sensor_tool_length_height / mm_per_step() * preference_motor_direction);
      return true;
    case settings_menu:
      //      status_settings_menu_active_page =  0;
      return true;
    case reset:
      reset_settings_to_default();
      return true;
    default:
      Serial.println("no entry code found for state: " + String(state));
      return true;
  }
}

bool run_state_exit(enum states state) {
  switch (state) {
    case goto_toolchange:
      Serial.println("exit goto_toolchange");
      halt_motor();
      return true;
    case goto_tool_length_sensor:
      Serial.println("exit goto_tool_length_sensor");
      halt_motor();
      return true;
    case finish_tool_length_sensor:
      Serial.println("exit finish_tool_length_sensor");
      halt_motor();
      return true;
    case reset:
      delay(DURATION_SHOW_MESSAGE); // show message for this time
      return true;
    default:
      Serial.println("no exit code found for state: " + String(state));
      return true;
  }
}

void activate_workspace() {
  status_workspace_lower_limit = stepper.currentPosition();
  status_workspace_upper_limit = status_workspace_lower_limit + (preference_workspace_height / mm_per_step());
  status_workspace_active = true;
  Serial.println("status_workspace_lower_limit: " + String(status_workspace_lower_limit));
  Serial.println("status_workspace_upper_limit: " + String(status_workspace_upper_limit));
}

void deactivate_workspace() {
  status_workspace_active = false;
}

void activate_target() {
  status_target_active = true;
  status_target_height = position_in_mm();
  status_target_lower_limit = stepper.currentPosition();
}

void deactivate_target() {
  status_target_active = false;
}

bool free_sensor_end_stop() {
  Serial.println("free_sensor_end_stop");
  start_pos = stepper.currentPosition();
  max_pos   = (long)(MM_TO_FREE_ERROR / mm_per_step());
  // move back with a quarter of base speed
  stepper.setSpeed(-preference_motor_speed_maximal * 0.25 * preference_motor_direction);
  while (read_sensor_end_stop_trigger()) {
    if (abs(stepper.currentPosition() - start_pos) > max_pos) {
      return false;
    }
    stepper.runSpeed();
  }
  // move some extra steps to avoid triggering sensor again
  stepper.runToNewPosition(stepper.currentPosition() - FREE_SENSOR_TOLERANCE * preference_motor_direction);
  Serial.println("free_sensor_end_stop done");
  halt_motor();
  return true;
}

bool free_sensor_tool_length() {
  Serial.println("free_sensor_tool_length");
  start_pos = stepper.currentPosition();
  max_pos   = (long)(MM_TO_FREE_ERROR / mm_per_step());
  // move back with a quarter of base speed
  stepper.setSpeed(-preference_motor_speed_maximal * 0.25 * preference_motor_direction);
  while (read_sensor_tool_length_trigger()) {
    if (abs(stepper.currentPosition() - start_pos) > max_pos) {
      return false;
    }
    stepper.runSpeed();
  }
  // move some extra steps to avoid triggering sensor again
  stepper.runToNewPosition(stepper.currentPosition() - FREE_SENSOR_TOLERANCE * preference_motor_direction);
  Serial.println("free_sensor_tool_length done");
  halt_motor();
  return true;
}

void toggle_target() {
  if (status_target_active) {
    deactivate_target();
  } else {
    activate_target();
  }
}

void set_zero() {
  deactivate_target();
  if (status_workspace_active) {
    status_workspace_upper_limit -= stepper.currentPosition();
    status_workspace_lower_limit -= stepper.currentPosition();
  }
  stepper.setCurrentPosition(0);
}

void error_with(char *error_message) {
  Serial.println("Error: " + String(error_message));
  status_error_message = error_message;
  current_state = error;
}