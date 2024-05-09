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

#define EN_PIN    36 //enable (CFG6)
#define PIN_STEP      12
#define PIN_DIRECTION 13

#define PIN_BUTTON_UP          21
#define PIN_BUTTON_DOWN        19
#define PIN_BUTTON_TOOLCHANGE  18
#define PIN_BUTTON_SET_ZERO    5
#define PIN_BUTTON_SET_SPEED    17
#define PIN_BUTTON_GOTO_BOTTOM 16

#define PIN_ENCODER_A_MINUS 14
#define PIN_ENCODER_B_MINUS 27

#define PIN_ICC_DATA 22
#define PIN_ICC_CLOCK 23

#define PIN_SENSOR_END_STOP_TRIGGER    33
#define PIN_SENSOR_TOOL_LENGTH_TRIGGER 25
#define PIN_SENSOR_TOOL_LENGTH_ENABLED  26

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
//ESP32Encoder handRad;

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

// *********************************** Handrad **********************************
float         handRadMultiplier = 1;// 0.25; // HandradSteps -> MotorSteps & Einstellungssteps (Anzeige *4)
bool          handRadMode = true; // Position über Handrad
int64_t       handRadValue = 0; // hier wird die vorherige Position von Handrad geladen, bevor Settings
int           handWheelFactor = 0; // hier werden die Slow/Fast Variablen reingeladen

// PREFERENCE VALUES START
long  default_motor_steps_per_revolution = 400  ; // [steps per revolution]
float default_motor_thread_pitch         =    8.0; // [mm per revolution]
long  default_motor_steps_slow           = round(0.05 / (default_motor_thread_pitch / default_motor_steps_per_revolution))  ; // [steps per encoder step]
long  default_motor_steps_fast           = round(1.0 / (default_motor_thread_pitch / default_motor_steps_per_revolution))  ; // [steps per encoder step]
long  default_motor_direction            = -1  ; // [-1 or 1]
long  default_motor_speed_maximal        = default_motor_steps_per_revolution  ; // [steps per second]
long  default_motor_speed_toolchange     = default_motor_steps_per_revolution  ; // [steps per second]
long  default_motor_acceleration         = default_motor_steps_per_revolution >> 2  ; // [steps per second per second]

bool  default_sensor_end_stop_normally_closed = true;

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

const char *status_error_message = "";
// STATUS VALUES END

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

// COMPUTED VALUES START
float mm_per_step() {
  return preference_motor_thread_pitch / preference_motor_steps_per_revolution; // [mm per step]
}

float position_in_mm() {
  return round(stepper.currentPosition() * mm_per_step() * 100) / 100 * preference_motor_direction; // [mm]
}
void halt_motor() {
  stepper.setSpeed(0);
  stepper.move(0);
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

void deactivate_target() {
  status_target_active = false;
}
void deactivate_workspace() {
  status_workspace_active = false;
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

void error_with(const char *error_message) {
  Serial.println("Error: " + String(error_message));
  status_error_message = error_message;
  current_state = error;
}
void activate_workspace() {
  status_workspace_lower_limit = stepper.currentPosition();
  status_workspace_upper_limit = status_workspace_lower_limit + (preference_workspace_height / mm_per_step());
  status_workspace_active = true;
  Serial.println("status_workspace_lower_limit: " + String(status_workspace_lower_limit));
  Serial.println("status_workspace_upper_limit: " + String(status_workspace_upper_limit));
}

void activate_target() {
  status_target_active = true;
  status_target_height = position_in_mm();
  status_target_lower_limit = stepper.currentPosition();
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

void reset_settings_to_default() {
  preferences.clear();
  read_settings();
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

// COMPUTED VALUES END
void change_state_to(enum states new_state) {
  if (run_state_exit(current_state) && run_state_entry(new_state)) {
    current_state = new_state;
  }
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


void show_position_in_mm() {
  display_I2C.setFont(u8g2_font_helvB18_tf); // choose a suitable font

  pos_in_mm = position_in_mm();

  if (pos_in_mm >= 0) {
    if (pos_in_mm < 10) {
      display_I2C.setCursor(30, 25);
    } else {
      display_I2C.setCursor(17, 25);
    }
  } else {
    if (pos_in_mm > -10) {
      display_I2C.setCursor(22, 25);
    } else {
      display_I2C.setCursor(9, 25);
    }
  }
  display_I2C.print(pos_in_mm);
  display_I2C.print(" mm");
}

void show_fast_slow_and_target() {
  display_I2C.setCursor(45, 48);

  if (status_target_active) {
    display_I2C.drawCircle(ux, uy - 6, 7, U8G2_DRAW_ALL);
    display_I2C.drawCircle(ux, uy - 6, 4, U8G2_DRAW_ALL);
    display_I2C.drawLine(ux - 8, uy - 6, ux + 8, uy - 6);
    display_I2C.drawLine(ux, uy - 14, ux, uy + 2);
    display_I2C.setCursor(69, 48);
    display_I2C.setFont(u8g2_font_helvB10_tf);
    display_I2C.print(status_target_height);
    display_I2C.print("mm");
    display_I2C.setFont(u8g2_font_helvB12_tf);
    display_I2C.setCursor(0, 48);
  }

  if (status_slow_speed) {
    display_I2C.print("SLOW");
  } else {
    display_I2C.print("FAST");
  }
}

void show_workspace() {
  if (status_workspace_active) {
    display_I2C.setCursor(5, 64);
    display_I2C.setFont(u8g2_font_helvB08_tf);
    display_I2C.print("WS");
    if (stepper.currentPosition() == status_workspace_lower_limit) {
      display_I2C.print(" MAX");
    }
    if (stepper.currentPosition() == status_workspace_upper_limit) {
      display_I2C.print(" MIN");;
    }
  }
}

void show_menu_title(const char *title) {
  display_I2C.setFont(u8g2_font_helvB10_tf);
  display_I2C.setCursor(0, 15);
  display_I2C.print(title);
  display_I2C.setFont(u8g2_font_helvB12_tf);
  display_I2C.setCursor(0, 50);
}

void show_sensor_tool_length_enabled() {
  if (read_sensor_tool_length_enabled()) {
    display_I2C.drawDisc(125, 61, 2, U8G2_DRAW_ALL);
  }
}

void show_settings_menu() {
  switch (status_settings_menu_active_page) {
    case 0:
      show_menu_title("Maximal Speed");
      display_I2C.print(preference_motor_speed_maximal);
      display_I2C.print(" steps/sec");
      break;
    case 1:
      show_menu_title("Acceleration");
      display_I2C.print(preference_motor_acceleration);
      display_I2C.print(" steps/sec^2");
      break;
    case 2:
      show_menu_title("Steps per Rev.");
      display_I2C.print(preference_motor_steps_per_revolution);
      break;
    case 3:
      show_menu_title("Motor Direction");
      if (preference_motor_direction == -1) {
        display_I2C.print("CCW");
      } else {
        display_I2C.print("CW");
      }
      break;
    case 4:
      show_menu_title("TL-Sensor Height");
      display_I2C.print(preference_sensor_tool_length_height);
      display_I2C.print(" mm");
      break;
    case 5:
      show_menu_title("Thread Pitch");
      display_I2C.print(preference_motor_thread_pitch);
      display_I2C.print(" mm");
      break;
    case 6:
      show_menu_title("Encoder Slow");
      display_I2C.print(preference_motor_steps_slow * mm_per_step());
      display_I2C.print(" mm/step");
      break;
    case 7:
      show_menu_title("Encoder Fast");
      display_I2C.print(preference_motor_steps_fast * mm_per_step());
      display_I2C.print(" mm/step");
      break;
    case 8:
      show_menu_title("Toolchange Speed");
      display_I2C.print(preference_motor_speed_toolchange);
      display_I2C.print(" steps/sec");
      break;
    case 9:
      show_menu_title("Autozero Speed");
      display_I2C.print(preference_auto_zero_speed);
      display_I2C.print(" steps/sec");
      break;
    case 10:
      show_menu_title("Workspace Height");
      display_I2C.print(preference_workspace_height);
      display_I2C.print(" mm");
      break;
    case 11:
      show_menu_title("PowerUp-Toolch.");
      if (preference_power_on_toolchange) {
        display_I2C.print("OK");
      } else {
        display_I2C.print("--");
      }
      break;
    case 12:
      show_menu_title("TL-Sensor Enable");
      if (preference_sensor_tool_length_enabled_normally_closed) {
        display_I2C.print("NC");
      } else {
        display_I2C.print("NO");
      }
      break;
    case 13:
      show_menu_title("TL-Sensor");
      if (preference_sensor_tool_length_normally_closed) {
        display_I2C.print("NC");
      } else {
        display_I2C.print("NO");
      }
      break;
    case 14:
      show_menu_title("Endstop-Sensor");
      if (preference_sensor_end_stop_normally_closed) {
        display_I2C.print("NC");
      } else {
        display_I2C.print("NO");
      }
      break;
    default:
      Serial.println("invalid menu page number: " + String(status_settings_menu_active_page));
  }
}

void show_error() {
  display_I2C.setFont(u8g2_font_helvB14_tf);
  display_I2C.setCursor(30, 30);
  display_I2C.print("ERROR:");
  display_I2C.setFont(u8g2_font_helvB10_tf);
  display_I2C.setCursor(0, 50);
  display_I2C.print(status_error_message);
}

void show_reset() {
  display_I2C.setFont(u8g2_font_helvB14_tf);
  display_I2C.setCursor(30, 30);
  display_I2C.print("RESET");
  display_I2C.setCursor(30, 50);
  display_I2C.print("VALUES");
}

void draw() {
  display_I2C.clearBuffer();
  switch (current_state) {
    case settings_menu:
      show_settings_menu();
      break;
    case reset:
      show_reset();
      break;
    case error:
      show_error();
      break;
    default:
      show_position_in_mm();
      show_fast_slow_and_target();
      show_workspace();
      show_sensor_tool_length_enabled();
  }
  display_I2C.sendBuffer();
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

void draw_loop(void * parameter) {
  display_I2C.begin();

  while (true) {
    collect_inputs();
    draw();
  }
}

bool moves_motor_within_workspace() {
  if (status_motor_mode_constant) {
    // moving down with space below
    return (stepper.speed() < 0 && stepper.currentPosition() > status_workspace_lower_limit)
           // or moving up with space above
           || (stepper.speed() > 0 && stepper.currentPosition() < status_workspace_upper_limit);
  } else  {
    // moving down with space below
    return (stepper.targetPosition() < stepper.currentPosition() && stepper.currentPosition() > status_workspace_lower_limit)
           // or moving up with space above
           || (stepper.targetPosition() > stepper.currentPosition() && stepper.currentPosition() < status_workspace_upper_limit);
  }
}

bool moves_motor_within_target() {
  if (status_motor_mode_constant) {
    // moving down with space to target
    return (stepper.speed() < 0 && stepper.currentPosition() > status_target_lower_limit)
           // or moving up with no limit
           || (stepper.speed() > 0);
  } else  {
    // moving down with space to target
    return (stepper.targetPosition() < stepper.currentPosition() && stepper.currentPosition() > status_target_lower_limit)
           // or moving up with no limit
           || (stepper.targetPosition() > stepper.currentPosition());
  }
}


bool is_motor_move_possible() {
  // end stop not activated
  return !read_sensor_end_stop_trigger()
         && (
           // workspace is off
           !status_workspace_active
           // or inside workspace
           || moves_motor_within_workspace()
         )
         && (
           // target is off
           !status_target_active
           // or inside target
           || moves_motor_within_target()
         );
}

bool is_motor_at_target() {
  return stepper.distanceToGo() == 0;
}

bool move_motor_constant() {
  status_motor_mode_constant = true;
  if (is_motor_move_possible()) {
    stepper.runSpeed();
    return true;
  } else {
    halt_motor();
    return false;
  }
}

bool move_motor_accelerate() {
  status_motor_mode_constant = false;
  if (is_motor_move_possible()) {
    stepper.run();
    return true;
  } else {
    halt_motor();
    return false;
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

void handle_settings_menu_change() {
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

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); //deactivate driver (LOW active)  
  digitalWrite(EN_PIN, LOW); //activate driver
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
  //encoder.attachHalfQuad(PIN_ENCODER_A_MINUS, PIN_ENCODER_B_MINUS);
  encoder.attachSingleEdge(PIN_ENCODER_A_MINUS, PIN_ENCODER_B_MINUS);
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