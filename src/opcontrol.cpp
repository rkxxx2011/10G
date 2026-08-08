#include "claw.hpp"
#include "dr4b.hpp"
#include "intake.hpp"
#include "main.h"
#include "ports.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "tasks.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

class OpControl {
private:
  inline static size_t level_ = 1;
  inline static float init_height_ = init_heights::ALLIANCE;
private:
  static void lift_incr() {
    const auto prev_target = lift.get_target();

    lift.set_target(get_height_at_level(level_ += 1, init_height_));

    if (lift.get_target() == prev_target) {
      level_--;
    }

    if (level_ > 1) {
      claw.set_target_angle(claw_angle::CLIMB);
    }
  }

  static void lift_decr() {
    if (level_ == 1) {
      return;
    }

    lift.set_target(get_height_at_level(--level_, init_height_));

    if (level_ == 1) {
      claw.set_target_angle(claw_angle::LOADING);
    }
  }

  static void score(pros::controller_digital_e_t cancel_btn) {
    lift.set_target(get_height_at_level(level_ - 0.25, init_height_));
    claw.set_target_angle(130);
    for (size_t time = 0; time < 500; time += 10) {
      lift.update();
      claw.update();
      pros::delay(10);
    }
    claw.set_target_angle(82);
    for (size_t time = 0; time < 150; time += 10) {
      lift.update();
      claw.update();
      pros::delay(10);
    }
    claw.set_state(ClawState::OUTAKING);
    for (size_t time = 0; time < 350; time += 5) {
      lift.update();
      claw.update();
      pros::delay(5);
    }
    lift.set_target(get_height_at_level(level_ + 0.75, init_height_));
    claw.set_target_angle(claw_angle::CLIMB);
  }

  static void toggle_init_height() {
    if (init_height_ == init_heights::ALLIANCE) {
      ctrler.rumble("--");
      init_height_ = init_heights::NEUTRAL;
      return;
    }

    if (init_height_ == init_heights::NEUTRAL) {
      ctrler.rumble("-.");
      init_height_ = init_heights::MIDDLE;
      return;
    }

    if (init_height_ == init_heights::MIDDLE) {
      ctrler.rumble("..");
      init_height_ = init_heights::ALLIANCE;
      return;
    }
  }
public:
 static void chassis_update() {
    const auto linear = ctrler.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const auto angular = ctrler.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    chassis.arcade(linear, angular);
  }

  static void lift_update() {
    const auto prev_level = level_;

    if (ctrler.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) lift_incr();
    else if (ctrler.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) lift_decr();

    if (level_ != prev_level) {
      ctrler.rumble(".");
    }
  }

  static void intake_update() {
    claw.set_state(ClawState::INTAKING);

    if (ctrler.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.set_mode(IntakeMode::INTAKE);
      return;
    }

    if (ctrler.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.set_mode(IntakeMode::OUTAKE);
      claw.set_state(ClawState::OUTAKING);
      return;
    }

    intake.set_mode(IntakeMode::STOPPED);
  }

  static void score_update() {
    if (ctrler.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      score(pros::E_CONTROLLER_DIGITAL_Y);
    }
  }

  static void update() {
    chassis_update();
    lift_update();
    intake_update();
    score_update();
    lift.update();
    claw.update();

    if (ctrler.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        toggle_init_height();
    }

    if (ctrler.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      lift.set_target(0);
      claw.set_target_angle(70);
      for (size_t time = 0; time < 400; time += 10) {
      lift.update();
      claw.update();
      pros::delay(10);
    }
      claw.set_state(ClawState::OUTAKING);
      for (size_t time = 0; time < 500; time += 10) {
      lift.update();
      claw.update();
      pros::delay(10);
    }
      lift.set_target(get_height_at_level(1, init_height_));
      claw.set_state(ClawState::INTAKING);
    }
  }

  static void loop() {
    while (true) {
      update();

      pros::delay(10);
    }
  }
};

void opcontrol() {
  OpControl::loop();
}