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

void draw_loop(void * parameter) {
  display_I2C.begin();

  while (true) {
    collect_inputs();
    draw();
  }
}
