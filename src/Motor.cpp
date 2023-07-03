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

void halt_motor() {
  stepper.setSpeed(0);
  stepper.move(0);
}
