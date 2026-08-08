#include "grape.hpp"
#include "claw.hpp"
#include "dr4b.hpp"
#include "intake.hpp"
#include "ports.hpp"

#include "ports.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/device.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "tasks.hpp"
#include <cstdint>

constexpr int32_t IGB = INT32_MAX;

constexpr float RESET_ALLIANCE_GOAL = 5.75 / 2.f;
constexpr float RESET_NEUTRAL_GOAL = 6.5 / 2.f;
#define UNDECLRAED_GOAL(RADIUS) (static_cast<float>(RADIUS))

void circle_reset(float goal, const float goal_x = 0, const float goal_y = 0) {
    const float theta = chassis.getPose().theta;

    const float theta_radians = lemlib::degToRad(theta);

    goal = std::abs(goal);

    //Basic Trigonemtry to get cartesian coordinates along a circle of radius(AKA r): (r * sin(theta), r * cos(theta))
    //NOTE: because in lemlib 0 degrees is north and not east (which is different from standard mathimatics) we swap around cosine and sine to account for that
    chassis.setPose(
        goal_x + goal * std::sin(theta_radians),
        goal_y + goal * std::cos(theta_radians),
        theta
    );
}

void set_lift(const float target) {
  lift_mutex.take();
  lift.set_target(target);
  lift_mutex.give();
}

void set_level(const float level, const float init_height, const float height_increment = 14) {
  lift_mutex.take();
  lift.set_target(get_height_at_level(level, init_height, height_increment));
  lift_mutex.give();
}

void manualRun(int v, uint32_t t) {
    lift_task.suspend();

    lift_motors.move_voltage(v);
    pros::delay(t);
    lift_motors.move(0);

    lift_task.resume();
}

void set_claw_tilt(const float angle) {
  claw_mutex.take();
  claw.set_target_angle(angle);
  claw_mutex.give();
}

void set_claw_state(const ClawState state) {
  claw_mutex.take();
  claw.set_state(state);
  claw_mutex.give();
}

void c() {
  set_claw_state(ClawState::INTAKING);

  chassis.setPose(0, 0, 0);
  chassis.moveToPose(10, 34, 0, 2000, {.minSpeed = 127});
  chassis.moveToPoint(12, 26, 900, {.forwards = false, .maxSpeed = 110, .minSpeed = 80});
  chassis.waitUntilDone();
  chassis.arcade(-60, 0);
  pros::delay(500);
  chassis.arcade(0, 0);

  pros::delay(1000);

  chassis.turnToHeading(-60, 1500, {.maxSpeed = 127, .minSpeed = 20});
  chassis.waitUntilDone();

  circle_reset(RESET_ALLIANCE_GOAL);

  chassis.moveToPoint(-8, 8, 1000);
  chassis.waitUntilDone();
  chassis.turnToHeading(125, 1000, {.maxSpeed = 80});
  set_claw_tilt(120);
  chassis.waitUntilDone();
  chassis.moveToPoint(-11, 10, INT32_MAX, {.forwards = false, .maxSpeed = 30});
  chassis.waitUntilDone();
  pros::delay(1000);
  set_claw_tilt(22.5);
  pros::delay(1000);
  set_claw_tilt(claw_angle::SCORING);
  chassis.moveToPoint(-37, -10, 1500, {.forwards = false, .maxSpeed = 127, .minSpeed = 100});
  set_level(3.5, init_heights::NEUTRAL);
  chassis.waitUntilDone();
  pros::delay(400);
  set_level(2, init_heights::NEUTRAL);
  set_claw_state(ClawState::OUTAKING);
  pros::delay(300);
  set_level(3.5, init_heights::NEUTRAL);
}

void fourpin() {
  set_claw_state(ClawState::INTAKING);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.setPose(0.0, 0.0, 0.0);
  intake.disable_jam_detection();

  chassis.moveToPoint(0, 5, 1000, {.minSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(100);
  chassis.moveToPoint(0, -2, 1000, {.forwards = false, .minSpeed = 127});
  chassis.moveToPoint(0, 5, 1000, {.minSpeed = 127});
  chassis.waitUntilDone();
  chassis.moveToPoint(0, -2, 1000, {.forwards = false, .minSpeed = 127});
  chassis.waitUntilDone();

  pros::delay(200);

  chassis.arcade(100, 0);
  pros::delay(300);

  set_level(1, init_heights::NEUTRAL);
  set_claw_tilt(claw_angle::LOADING);
  intake.move(127);

  // Load rings
  chassis.moveToPoint(0, 45, 3000, {.maxSpeed = 127, .minSpeed = 120});
  chassis.waitUntilDone();

  pros::delay(1000);

  // Mirror of fivepin
  chassis.moveToPose(7, 10, -70, 1000, {.forwards = false, .minSpeed = 90});

  chassis.moveToPoint(-28, 26, 1000, {.forwards = false, .minSpeed = 60});

  set_level(3.5, init_heights::NEUTRAL);


  pros::delay(425);

  set_claw_tilt(claw_angle::SCORING - 3.4);
  pros::delay(100);
  set_level(3.5, init_heights::NEUTRAL);

  chassis.waitUntilDone();

  chassis.arcade(-50, 0);
  pros::delay(100);

  set_level(2, init_heights::NEUTRAL);

  pros::delay(900);

  set_claw_state(ClawState::OUTAKING);

  pros::delay(400);

  set_level(3.5, init_heights::NEUTRAL);

  pros::delay(400);

  set_claw_tilt(claw_angle::SCORING - 5);

  pros::delay(100);

  set_level(3.5, init_heights::NEUTRAL);

  chassis.arcade(0, 0);
  pros::delay(100);

  // Mirrored odom reset
  circle_reset(RESET_NEUTRAL_GOAL);

  pros::delay(100);

  chassis.moveToPoint(14, 21, 3000, {.maxSpeed = 100});
  chassis.waitUntilDone();
  set_level(1, init_heights::INTAKE);
  set_claw_tilt(claw_angle::LOADING);
  set_claw_state(ClawState::INTAKING);
  intake.set_mode(IntakeMode::INTAKE);

  intake.set_mode(IntakeMode::INTAKE);
  chassis.moveToPose(1, 27, -90, 5000, {.maxSpeed = 70});
  chassis.waitUntilDone();
  chassis.moveToPoint(36, 7, 2000, {.forwards = false, .minSpeed = 80});
  pros::delay(300);
  set_claw_tilt(76);
  chassis.waitUntilDone();
  chassis.arcade(-50, 0);
  pros::delay(500);
  set_claw_state(static_cast<ClawState>(-40));
}

void fourpin_mirrored() {
  set_claw_state(ClawState::INTAKING);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.setPose(0.0, 0.0, 0.0);
  intake.disable_jam_detection();

  chassis.moveToPoint(0, 5, 1000, {.minSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(100);
  chassis.moveToPoint(0, 0, 1000, {.forwards = false, .minSpeed = 127});
  chassis.moveToPoint(0, 5, 1000, {.minSpeed = 127});
  chassis.waitUntilDone();
  chassis.moveToPoint(0, 0, 1000, {.forwards = false, .minSpeed = 127});
  chassis.waitUntilDone();

  pros::delay(200);

  chassis.arcade(100, 0);
  pros::delay(300);

  set_level(1, init_heights::NEUTRAL);
  set_claw_tilt(claw_angle::LOADING);
  intake.move(127);

  // Load rings
  chassis.moveToPoint(0, 37, 3000, {.maxSpeed = 127, .minSpeed = 120});
  chassis.waitUntilDone();

  pros::delay(1000);

  // Mirrored approach
  chassis.moveToPose(-7, 10, 70, 1000, {.forwards = false, .minSpeed = 90});

  chassis.moveToPoint(30, 27, 1000, {.forwards = false, .minSpeed = 60});

  set_level(3.5, init_heights::NEUTRAL);

  pros::delay(425);

  set_claw_tilt(claw_angle::SCORING - 6.7);
  pros::delay(100);
  set_level(3.5, init_heights::NEUTRAL);

  chassis.waitUntilDone();

  chassis.arcade(-50, 0);
  pros::delay(100);

  set_level(2, init_heights::NEUTRAL);

  pros::delay(1200);

  set_claw_state(ClawState::OUTAKING);

  pros::delay(400);

  set_level(3.5, init_heights::NEUTRAL);

  pros::delay(400);

  set_claw_tilt(claw_angle::SCORING - 5);

  pros::delay(100);

  set_level(3.5, init_heights::NEUTRAL);

  chassis.arcade(0, 0);
  pros::delay(100);

  // Mirror this reset constant if your project has one.
  circle_reset(RESET_NEUTRAL_GOAL);

  pros::delay(100);

  chassis.moveToPoint(-14, 21, 3000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  set_level(1, init_heights::INTAKE);
  set_claw_tilt(claw_angle::LOADING);
  set_claw_state(ClawState::INTAKING);
  intake.set_mode(IntakeMode::INTAKE);

  intake.set_mode(IntakeMode::INTAKE);

  chassis.moveToPose(2, 27, 90, 5000, {.maxSpeed = 70});
  chassis.waitUntilDone();

  chassis.moveToPoint(-36, 0, 1200, {.forwards = false, .minSpeed = 127});
  pros::delay(300);
  set_claw_tilt(70);

  chassis.waitUntilDone();

  chassis.arcade(-50, 0);

  pros::delay(200);

  set_claw_state(ClawState::OUTAKING);
}

void fourpin_mirrored_mirrored() {
  set_claw_state(ClawState::INTAKING);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.setPose(0.0, 0.0, 0.0);
  intake.disable_jam_detection();

  chassis.moveToPoint(0, 5, 1000, {.minSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(100);

  chassis.moveToPoint(0, -2, 1000, {.forwards = false, .minSpeed = 127});
  chassis.moveToPoint(0, 5, 1000, {.minSpeed = 127});
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -2, 1000, {.forwards = false, .minSpeed = 127});
  chassis.waitUntilDone();

  pros::delay(200);

  chassis.arcade(100, 0);
  pros::delay(300);

  set_level(1, init_heights::NEUTRAL);
  set_claw_tilt(claw_angle::LOADING);
  intake.move(127);

  // Load rings
  chassis.moveToPoint(0, 47, 3000, {.maxSpeed = 127, .minSpeed = 120});
  chassis.waitUntilDone();

  pros::delay(1000);

  // Mirrored across Y-axis
  chassis.moveToPose(-17, 10, 110, 1000,
                     {.forwards = false, .minSpeed = 90});

  chassis.moveToPoint(-30, 23, 1000,
                      {.forwards = false, .minSpeed = 60});

  set_level(3.5, init_heights::NEUTRAL);

  pros::delay(425);

  set_claw_tilt(claw_angle::SCORING - 3.4);
  pros::delay(100);
  set_level(3.5, init_heights::NEUTRAL);

  chassis.waitUntilDone();

  chassis.arcade(-50, 0);
  pros::delay(100);

  set_level(2, init_heights::NEUTRAL);

  pros::delay(900);

  set_claw_state(ClawState::OUTAKING);

  pros::delay(400);

  set_level(3.5, init_heights::NEUTRAL);

  pros::delay(400);

  set_claw_tilt(claw_angle::SCORING - 5);

  pros::delay(100);

  set_level(3.5, init_heights::NEUTRAL);

  chassis.arcade(0, 0);
  pros::delay(100);

  // Use mirrored reset constant if your project has one.
  circle_reset(RESET_NEUTRAL_GOAL);

  pros::delay(100);

  chassis.moveToPoint(14, 21, 3000, {.maxSpeed = 100});
  chassis.waitUntilDone();

  set_level(1, init_heights::INTAKE);
  set_claw_tilt(claw_angle::LOADING);
  set_claw_state(ClawState::INTAKING);
  intake.set_mode(IntakeMode::INTAKE);

  intake.set_mode(IntakeMode::INTAKE);

  chassis.moveToPose(-2, 27, 90, 3000, {.maxSpeed = 70});
  chassis.waitUntilDone();

  chassis.moveToPoint(36, 3, 2000, {.forwards = false, .minSpeed = 127});

  pros::delay(300);
  set_claw_tilt(70);

  chassis.waitUntilDone();

  chassis.arcade(-50, 0);

  pros::delay(100);

  set_claw_state(ClawState::OUTAKING);
}

void skills(){
  chassis.setBrakeMode(
  pros::E_MOTOR_BRAKE_HOLD
  );
  // original: clawRotate.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); (claw rotation brake mode set to HOLD)
  chassis.setPose(
  64,
  0,
  270
  );


  intake.set_mode(IntakeMode::INTAKE);
  set_claw_state(ClawState::INTAKING);
  chassis.moveToPoint(47,270, 1000, {.minSpeed=127}); // go back
  chassis.waitUntilDone();

  chassis.turnToHeading(180, 500, {.minSpeed=127}); //turn to alliance
  chassis.waitUntilDone();

  chassis.moveToPoint(47, 18, 1000, {.forwards=false, .minSpeed=127}); //go to alliance
  chassis.waitUntilDone();
  
  set_level(1, init_heights::ALLIANCE);
  set_claw_tilt(70);//score preload
  set_claw_state(ClawState::OUTAKING);
  pros::delay(400);

  chassis.turnToHeading(228, 500, {.minSpeed=127}); // turn to stack (center)
  chassis.waitUntilDone();

  chassis.moveToPoint(27, 2, 500, {.forwards=false, .minSpeed=127}); //go to stack
  chassis.waitUntilDone();

  set_claw_tilt(180); //prep clamp
  set_claw_state(ClawState::INTAKING);

  chassis.moveToPose(27, 2, 50, 750, {.forwards=false, .minSpeed=127});//finih stack motion
  chassis.waitUntilDone();

  set_claw_tilt(90); //grab stack

  chassis.moveToPose(44, -20, 320, 1000, {.forwards=false, .minSpeed=127}); //go to alliance
  chassis.waitUntilDone();

  //score on neutral goal #1
  set_level(1, init_heights::NEUTRAL);
  set_claw_tilt(90);
  set_claw_state(ClawState::OUTAKING);
  pros::delay(400);
  set_claw_tilt(180);
  set_claw_state(ClawState::INTAKING);
  

  chassis.moveToPoint(25, 20, 500, {.minSpeed=127}); //go to stack
  chassis.waitUntilDone();
  set_level(1, init_heights::INTAKE); //GO DOWN

  chassis.moveToPose(25, 20, 157, 1000, {.forwards=false, .minSpeed=127}); //go to stack
  chassis.waitUntilDone();

  set_claw_tilt(90); //grab stack
  
  chassis.moveToPose(43, 23, 270, 1000, {.forwards=false, .minSpeed=127}); //go to alliance
  chassis.waitUntilDone();

  set_claw_tilt (70); //score on alliance
  set_claw_state(ClawState::OUTAKING);
  set_level(1, init_heights::ALLIANCE);
  pros::delay(400);
  set_claw_tilt(180);
  set_claw_state(ClawState::INTAKING);
  

  chassis.moveToPoint(0, 24, 500, {.minSpeed=127});
  chassis.waitUntilDone();
  set_level(1, init_heights::INTAKE); //GO DOWN

  chassis.moveToPose(0, 24, 90, 1000, {.forwards=false, .minSpeed=127}); //go to stack
  chassis.waitUntilDone();

  set_claw_tilt(90); //grab stack

  chassis.moveToPose(43, 23, 270, 2500, {.forwards=false, .minSpeed=127}); //go to alliance
  chassis.waitUntilDone();

  set_claw_tilt (70); //score on alliance
  set_claw_state(ClawState::OUTAKING);
  set_level(1, init_heights::ALLIANCE);
  pros::delay(400);
  set_claw_tilt(180);
  set_claw_state(ClawState::INTAKING);

  chassis.moveToPoint(-18, 24, 2000, {.minSpeed=127}); //move to flower to intake
  chassis.waitUntilDone();
  set_claw_tilt(0);

  chassis.moveToPose(19, 43, 240, 2000, {.forwards=false, .minSpeed=127}); //go to flower
  chassis.waitUntilDone();
  
  //score the flower on alliance goal
  set_claw_tilt(70);
  set_claw_state(ClawState::OUTAKING);
  set_level(1, init_heights::ALLIANCE);
  pros::delay(400);
  set_claw_tilt(180);
  set_claw_state(ClawState::INTAKING);

  chassis.moveToPose(23, 39, 240, 500, {.minSpeed=127}); //go to stack
  chassis.waitUntilDone();

  chassis.moveToPose(44, 43, 230, 750, {.forwards=false, .minSpeed=127}); //go to stack
  chassis.waitUntilDone();

  chassis.moveToPose(27, 47, 90, 750, {.forwards=false, .minSpeed=127}); //score on alliance goal
  chassis.waitUntilDone();

  set_level(2, init_heights::ALLIANCE); //idk how to update this to two but its the first pin+cup
  set_claw_tilt(90);
  set_claw_state(ClawState::OUTAKING);

  chassis.moveToPose(66, 58, 90, 2000, {.minSpeed = 127});// go to matchload
  chassis.waitUntilDone();
  pros::delay(2000);

  chassis.moveToPose(27, 47, 90, 750, {.forwards=false, .minSpeed=127}); //score on alliance goal
  chassis.waitUntilDone();

  set_level(3, init_heights::ALLIANCE); //idk how to update this to three but its the second pin+cup
  set_claw_tilt(90);
  set_claw_state(ClawState::OUTAKING);

  chassis.moveToPose(66, 58, 90, 2000, {.minSpeed = 127});// go to matchload
  chassis.waitUntilDone();
  pros::delay(2000);

  chassis.moveToPose(27, 47, 90, 750, {.forwards=false, .minSpeed=127}); //score on alliance goal
  chassis.waitUntilDone();

  set_level(4, init_heights::ALLIANCE); //idk how to update this to four but its the third pin+cup
  set_claw_tilt(90);
  set_claw_state(ClawState::OUTAKING);chassis.moveToPose(66, 58, 90, 2000, {.minSpeed = 127});// go to matchload
  chassis.waitUntilDone();
  pros::delay(2000);

  chassis.moveToPose(27, 47, 90, 750, {.forwards=false, .minSpeed=127}); //score on alliance goal
  chassis.waitUntilDone();

  set_level(5, init_heights::ALLIANCE); //idk how to update this to five but its the fourth pin+cup
  set_claw_tilt(90);
  set_claw_state(ClawState::OUTAKING);

  chassis.moveToPose(66, 58, 90, 2000, {.minSpeed = 127});// go to matchload
  chassis.waitUntilDone();
  pros::delay(2000);

  chassis.moveToPose(27, 47, 90, 750, {.forwards=false, .minSpeed=127}); //score on alliance goal
  chassis.waitUntilDone();

  set_level(6, init_heights::ALLIANCE); //idk how to update this to six but its the fifth pin+cup
  set_claw_tilt(90);
  set_claw_state(ClawState::OUTAKING);

}

void no_skill() {
  intake.set_mode(IntakeMode::INTAKE);
  set_claw_state(ClawState::INTAKING);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 5, 1000, {.minSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(100);
  chassis.moveToPoint(0, 0, 1000, {.forwards = false, .minSpeed = 127});
  chassis.moveToPoint(0, 5, 1000, {.minSpeed = 127});
  chassis.waitUntilDone();
  chassis.moveToPoint(0, 0, 1000, {.forwards = false, .minSpeed = 127});
  chassis.waitUntilDone();
  chassis.moveToPose(0, 20, -90, 2000, {.minSpeed = 50});
  chassis.moveToPoint(20, 20, 1000, {.forwards = false, .minSpeed = 90});
  pros::delay(300);
  set_claw_tilt(70);
  chassis.waitUntilDone();
  chassis.arcade(-50, 0);
  pros::delay(150);
  set_claw_state(ClawState::OUTAKING);
  circle_reset(RESET_ALLIANCE_GOAL);
  pros::delay(400);
  set_claw_tilt(130);
  intake.set_mode(IntakeMode::STOPPED);
  chassis.turnToHeading(-45, 4000, {.minSpeed = 120});
  chassis.moveToPoint(-7, 10, 1000, {.minSpeed = 127});
  chassis.moveToPoint(40, 10, 1000, {.minSpeed = 100});
  chassis.moveToPose(35, -35, 180, 2000, {.maxSpeed = 127, .minSpeed = 40});
  chassis.waitUntilDone();
  set_claw_tilt(0);
  intake.set_mode(IntakeMode::INTAKE);
  chassis.moveToPoint(35,-40,2000, {.minSpeed = 100});
  chassis.waitUntilDone();
  pros::delay(3000);
  chassis.moveToPose(5,-5, 90, 2000, {.forwards = false, .minSpeed = 100});
  chassis.waitUntilDone();

}

void sawp_v2() {

  chassis.setPose(-69.104, 0.291, 90);

  set_claw_state(ClawState::INTAKING);
  chassis.moveToPoint(-63,0.291,700,{.maxSpeed=110});
  chassis.moveToPoint(-72, 0.291,800,{.forwards=false,.minSpeed=120});
  chassis.moveToPoint(-63,0.291,700,{.maxSpeed=110});
  chassis.moveToPoint(-72, 0.291,800,{.forwards=false, .minSpeed=120});
  chassis.waitUntilDone();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.swingToHeading(-13,DriveSide::LEFT,800,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90});
  intake.move(127);
  set_level(1, init_heights::NEUTRAL);

  chassis.waitUntilDone();
  chassis.moveToPoint(-58,-14,1000,{.forwards=false,.maxSpeed=90});
  pros::delay(150);
  set_claw_tilt(claw_angle::LOADING);
  chassis.waitUntilDone();
  chassis.arcade(-127, 0);
  pros::delay(200);
  set_claw_state(ClawState::OUTAKING);
  pros::delay(200);
  set_level(1, init_heights::NEUTRAL);
  chassis.waitUntilDone();

  pros::delay(100);
  set_claw_tilt(claw_angle::LOADING);
  set_claw_state(ClawState::INTAKING);

  chassis.moveToPoint(-47.067,-5.354,800,{.maxSpeed=80});
  chassis.turnToHeading(-40,700,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90});
  chassis.waitUntilDone();
  chassis.moveToPoint(-30.059,-17.832,800,{.forwards=false,.maxSpeed=70});
  set_level(1, init_heights::NEUTRAL);
  chassis.waitUntilDone();

  set_level(1, init_heights::NEUTRAL);
  chassis.waitUntilDone();
  pros::delay(150);

  set_claw_state(ClawState::INTAKING);

  chassis.turnToHeading(-287,800,{.maxSpeed=80});
  set_claw_tilt(claw_angle::SCORING);
  set_level(2, init_heights::NEUTRAL);
  chassis.moveToPoint(-46.859,-24.032,700,{.forwards=false,.maxSpeed=90});
  chassis.waitUntilDone();
  chassis.arcade(-127, 0);

  pros::delay(100);
  set_claw_state(ClawState::OUTAKING);

  set_claw_tilt(claw_angle::LOADING);
  chassis.moveToPoint(-30.332,-23.513,800,{.maxSpeed=110});
  chassis.waitUntilDone();
  chassis.moveToPoint(-29.604,-30.373,900,{.maxSpeed=100});
  chassis.waitUntilDone();
  set_level(1, init_heights::NEUTRAL);
  chassis.turnToHeading(42,900,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=85});
  set_claw_tilt(claw_angle::LOADING);

  chassis.waitUntilDone();
  chassis.moveToPoint(-42.375,-43.909,800,{.forwards=false,.maxSpeed=85});
  set_claw_state(ClawState::INTAKING);
  chassis.waitUntilDone();

  set_level(1, init_heights::NEUTRAL);
  chassis.waitUntilDone();

  set_claw_state(ClawState::INTAKING);

  chassis.moveToPoint(-51.375,-51.509,800,{.forwards=false,.maxSpeed=95});

  chassis.turnToHeading(153,700,{.maxSpeed=100});
  set_claw_tilt(claw_angle::SCORING);
  set_level(2.75, init_heights::NEUTRAL);
  chassis.waitUntilDone();
  chassis.moveToPoint(-53.547,-30.176,900,{.forwards=false,.maxSpeed=90});
  chassis.waitUntilDone();
  chassis.arcade(-70, 0);

  pros::delay(100);
  set_claw_state(ClawState::OUTAKING);

  set_claw_tilt(claw_angle::CLIMB);

  intake.move(127);

  chassis.moveToPoint(-49.77,-59.854,900,{.maxSpeed=100});
  pros::delay(400);
  set_level(1, init_heights::NEUTRAL);
  set_claw_tilt(claw_angle::LOADING);
  chassis.waitUntilDone();
  chassis.turnToHeading(270,750,{.maxSpeed=100});
  chassis.waitUntilDone();

  chassis.moveToPoint(-72.272,-59.466,700,{.maxSpeed=110,.minSpeed=20});
  chassis.waitUntilDone();
  chassis.arcade(100,0);
  pros::delay(800);

  set_claw_state(ClawState::INTAKING);
  chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});
  pros::delay(200);
  set_claw_tilt(claw_angle::LOADING);
  chassis.waitUntilDone();
  chassis.arcade(-80, 0);
  pros::delay(300);
  chassis.arcade(0,0);
  set_claw_state(ClawState::OUTAKING);
  pros::delay(200);
  set_level(1, init_heights::NEUTRAL);

  chassis.moveToPoint(-63.917,-59.5,800,{.maxSpeed=90});
  chassis.waitUntilDone();
  chassis.turnToHeading(270,500,{.maxSpeed=90});
  set_claw_state(ClawState::INTAKING);
  set_claw_tilt(claw_angle::LOADING);

  chassis.moveToPoint(-72.272,-58.3,900,{.maxSpeed=100});
  chassis.waitUntilDone();
  chassis.arcade(90,0);
  pros::delay(1000);

  chassis.moveToPoint(-38.543,-49.345,800,{.forwards=false,.maxSpeed=80});
  pros::delay(200);
  set_claw_tilt(claw_angle::LOADING);
  set_level(2, init_heights::NEUTRAL);
  chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});

  chassis.waitUntilDone();
  chassis.arcade(-100, 0);

  pros::delay(150);
  chassis.arcade(0,0);
  set_claw_state(ClawState::OUTAKING);
  pros::delay(200);
  set_claw_tilt(claw_angle::CLIMB);

  chassis.moveToPoint(-63.917,-59.4,1000,{.maxSpeed=90});
  set_claw_state(ClawState::INTAKING);
  chassis.waitUntilDone();
  chassis.turnToHeading(270,500,{.maxSpeed=90});
  set_claw_tilt(claw_angle::LOADING);
  chassis.moveToPoint(-72.272,-58.9,900,{.maxSpeed=80});
  set_claw_state(ClawState::INTAKING);
  set_level(1, init_heights::NEUTRAL);
  set_claw_tilt(claw_angle::LOADING);
  chassis.waitUntilDone();
  chassis.arcade(90,0);
  pros::delay(1000);

  chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
  set_claw_tilt(claw_angle::SCORING);
  set_level(2.75, init_heights::NEUTRAL);
  chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});
  chassis.waitUntilDone();
  chassis.arcade(-80, 0);

  pros::delay(300);
  chassis.arcade(0,0);

  set_claw_state(ClawState::OUTAKING);
  pros::delay(100);

  set_claw_tilt(claw_angle::CLIMB);

  chassis.moveToPoint(-63.917,-58.9,800,{.maxSpeed=90});
  set_claw_state(ClawState::INTAKING);
  chassis.turnToHeading(270,500,{.maxSpeed=100});
  set_claw_tilt(claw_angle::LOADING);
  set_level(1, init_heights::NEUTRAL);
  chassis.moveToPoint(-72.272,-57.9,900,{.maxSpeed=100});

  set_claw_state(ClawState::INTAKING);
  chassis.waitUntilDone();
  chassis.arcade(100,0);
  pros::delay(1000);

  chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
  set_claw_tilt(claw_angle::SCORING);
  set_level(3.5, init_heights::NEUTRAL);
  chassis.moveToPoint(-25.03,-47.232,800,{.forwards=false,.maxSpeed=70});
  chassis.waitUntilDone();
  chassis.arcade(-70, 0);

  pros::delay(300);
  chassis.arcade(0,0);

  pros::delay(400);
  set_claw_state(ClawState::OUTAKING);
  pros::delay(100);

  set_claw_tilt(claw_angle::CLIMB);

  chassis.moveToPoint(-63.917,-58,900,{.maxSpeed=100});
  chassis.turnToHeading(270,400,{.maxSpeed=100});
  set_claw_state(ClawState::INTAKING);
  set_claw_tilt(claw_angle::LOADING);
  set_level(1, init_heights::NEUTRAL);
  chassis.moveToPoint(-72.272,-58,600,{.maxSpeed=100});

  set_claw_state(ClawState::INTAKING);

  chassis.waitUntilDone();
  chassis.arcade(100,0);
  pros::delay(1000);

  chassis.moveToPoint(-56.314,-48.148,800,{.forwards=false,.maxSpeed=80});
  chassis.waitUntilDone();
  chassis.turnToHeading(180,800,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=80});
  set_claw_tilt(claw_angle::SCORING);
  set_level(3.5, init_heights::NEUTRAL);
  chassis.waitUntilDone();
  chassis.moveToPoint(-55.483,-20.448,1200,{.forwards=false,.maxSpeed=70});
  chassis.waitUntilDone();
  chassis.arcade(-60, 0);

  pros::delay(300);
  chassis.arcade(0,0);
  pros::delay(400);

  set_claw_state(ClawState::OUTAKING);
  pros::delay(100);

  chassis.moveToPoint(-53.154, -22.353, 500, {.maxSpeed=80});
  chassis.turnToHeading(60,900,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90});
  pros::delay(400);
  set_level(1, init_heights::NEUTRAL);
  set_claw_tilt(claw_angle::LOADING);
  chassis.moveToPoint(6, -15, 1000, {.maxSpeed=100, .earlyExitRange = 5});
  chassis.turnToHeading(0, 800, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=80});
  chassis.waitUntilDone();
  chassis.moveToPoint(chassis.getPose().x, -75, 1000, {.forwards = false, .minSpeed = 120});
  chassis.waitUntilDone();
  chassis.moveToPoint(chassis.getPose().x, -53, 1000);
  chassis.waitUntilDone();
  chassis.moveToPoint(chassis.getPose().x, -75, 1000, {.forwards = false, .minSpeed = 120});
  chassis.waitUntilDone();
  chassis.moveToPoint(-5,-8,3000,{.minSpeed=130});
}

void sawp() {
  chassis.setBrakeMode(
  pros::E_MOTOR_BRAKE_HOLD
  );
  // original: clawRotate.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); (claw rotation brake mode set to HOLD)
  chassis.setPose(
  0.0,
  0.0,
  0.0
  );
  // original: manualRun(-12000, 400); (lift manual voltage run: -12000mV for 400ms)
  // original: clawToStage(3); (claw to stage 3)
  // original: manualRun(-3000, 100); (lift manual voltage run: -3000mV for 100ms)
  // original: manualRun(5000, 200); (lift manual voltage run: 5000mV for 200ms)
  // original: liftToStage(3); (lift to stage 3)
  pros::delay(200);
  // original: heightMode=1; (lift height mode set to 1)
  chassis.arcade(100,0);
  pros::delay(300);
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(2); (claw to stage 2)
  // original: intake.move_velocity(600); (intake velocity 600)
  //start running claw and intake
  //A / loading position
  chassis.moveToPoint(0,50,1100,{.forwards=true, .minSpeed=127});// grab cup
  // original: liftToStage(2); (lift to stage 2)
  pros::delay(600);
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  chassis.waitUntilDone();
  chassis.arcade(60,0);
  pros::delay(100);
  chassis.arcade(0,0);
  pros::delay(100);
  chassis.arcade(-70,0);
  pros::delay(200);
  chassis.turnToHeading(42, 400, {.minSpeed=80});// turn to goal
  //raise lift to state 3 and position claw
  // original: heightMode=1; // mid mode (lift height mode set to 1)
  chassis.moveToPoint(-20,19,950, {.forwards=false, .minSpeed=120}); //go to goal
  pros::delay(375);
  // original: liftToStage(3); (lift to stage 3)
  // original: clawToStage(3); (claw to stage 3)
  chassis.waitUntilDone();
  chassis.arcade(-50,0);
  pros::delay(100);
  // original: claw.move_velocity(-600); (claw flex wheel velocity -600)
  pros::delay(400);
  // original: clawToStage(4); (claw to stage 4)
  pros::delay(100);
  // original: liftToStage(4); (lift to stage 4)
  chassis.arcade(0,0);
  pros::delay(100);
  // circle_reset(-24,15,8);
  pros::delay(5);
  chassis.arcade(60,0);
  pros::delay(300);
  // original: liftToStage(2); (lift to stage 2)
  // original: clawToStage(2); (claw to stage 2)
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  chassis.turnToHeading(-75,400, {.minSpeed=110});
  //score the thing and raise after
  chassis.moveToPoint(-16,34,700, {.forwards=true, .minSpeed=120});
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.moveToPoint(15,28,900, {.forwards=false, .minSpeed=120});
  // original: liftToStage(2); (lift to stage 2)
  // original: clawToStage(2); (claw to stage 2)
  pros::delay(800);
  // original: clawToStage(3); (claw to stage 3)
  // original: liftToStage(1); (lift to stage 1)
  // original: heightMode=0; (lift height mode set to 0)
  // go to positon to score alliance goal pin
  chassis.moveToPoint(18,17,700, {.forwards=false, .minSpeed=120});
  chassis.waitUntilDone();
  chassis.arcade(-80,0);
  // original: claw.move_velocity(-400); (claw flex wheel velocity -400)
  pros::delay(300);
  chassis.arcade(0,0);
  pros::delay(100);
  // original: clawToStage(3); (claw to stage 3)
  // original: liftToStage(2); (lift to stage 2)
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  // circle_reset(24,15,8);
  pros::delay(50);
  // original: clawToStage(4); (claw to stage 4)
  // original: intake.move_velocity(0); (intake velocity 0)
  chassis.arcade(0,0);
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  chassis.moveToPoint(11, 30, 500, {.minSpeed=90});
  chassis.turnToHeading(-120,400, {.minSpeed=120});
  chassis.waitUntilDone();
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  // original: heightMode=0; (lift height mode set to 0)
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(4); (claw to stage 4)
  //X mode ready to grab the thing
  chassis.moveToPose(20, 37,-147, 800, {.forwards=false, .minSpeed=80});//GRAB FIRST
  chassis.waitUntilDone();
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(3); (claw to stage 3)
  // grab //////////
  pros::delay(450);
  // original: heightMode=0; (lift height mode set to 0)
  // original: liftToStage(3); (lift to stage 3)
  // lift to score alliance stake with pin
  chassis.moveToPoint(52,36,900, {.forwards=false, .minSpeed=120});
  chassis.waitUntilDone();
  chassis.arcade(-100,0);
  pros::delay(50);
  chassis.arcade(-50,0);
  // original: claw.move_velocity(-600); (claw flex wheel velocity -600)
  // original: clawToStage(3); (claw to stage 3)
  pros::delay(300);
  chassis.arcade(0,0);
  pros::delay(50);
  // original: clawToStage(4); (claw to stage 4)
  // circle_reset(48,39,8);
  chassis.arcade(0,0);
  //SCORE pls
  chassis.moveToPoint(30,40, 700, {.forwards=true, .minSpeed=120}); // GRAB SECOND
  chassis.waitUntilDone();
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(4); (claw to stage 4)
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  chassis.turnToHeading(-32,400, {.minSpeed=100});
  chassis.waitUntilDone();
  // original: heightMode=0; (lift height mode set to 0)
  // original: clawToStage(4); (claw to stage 4)
  // ready to grab
  chassis.moveToPoint(52, 19, 1000, {.forwards=false, .minSpeed=100});
  pros::delay(850);
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(3); (claw to stage 3)
  // grab
  pros::delay(300);
  chassis.turnToHeading(75,500, {.minSpeed=100});
  // original: liftToStage(3); (lift to stage 3)
  chassis.moveToPoint(24, 17, 700, {.forwards=false, .minSpeed=127});
  chassis.waitUntilDone();
  // original: claw.move_velocity(-600); (claw flex wheel velocity -600)
  chassis.arcade(-80,0);
  pros::delay(100);
  chassis.arcade(0,0);
  //SCORE
  //score the pin
  return;
}

void Right3_3() {
  chassis.setBrakeMode(
  pros::E_MOTOR_BRAKE_HOLD
  );
  // original: clawRotate.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); (claw rotation brake mode set to HOLD)
  chassis.setPose(
  0.0,
  0.0,
  0.0
  );
  // original: manualRun(-12000, 400); (lift manual voltage run: -12000mV for 400ms)
  // original: clawToStage(3); (claw to stage 3)
  // original: manualRun(-3000, 100); (lift manual voltage run: -3000mV for 100ms)
  // original: manualRun(5000, 200); (lift manual voltage run: 5000mV for 200ms)
  // original: liftToStage(3); (lift to stage 3)
  pros::delay(200);
  // original: heightMode=1; (lift height mode set to 1)
  chassis.arcade(100,0);
  pros::delay(300);
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(2); (claw to stage 2)
  // original: intake.move_velocity(600); (intake velocity 600)
  //start running claw and intake
  //A / loading position
  chassis.moveToPoint(0,41,1100,{.forwards=true, .minSpeed=127});// grab cup
  // original: liftToStage(2); (lift to stage 2)
  pros::delay(600);
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  chassis.waitUntilDone();
  chassis.arcade(60,0);
  pros::delay(100);
  chassis.arcade(0,0);
  pros::delay(100);
  chassis.arcade(-70,0);
  pros::delay(200);
  chassis.turnToHeading(-42, 400, {.minSpeed=80});// turn to goal
  //raise lift to state 3 and position claw
  // original: heightMode=1; // mid mode (lift height mode set to 1)
  chassis.moveToPoint(20,19,950, {.forwards=false, .minSpeed=120}); //go to goal
  pros::delay(375);
  // original: liftToStage(3); (lift to stage 3)
  // original: clawToStage(3); (claw to stage 3)
  chassis.waitUntilDone();
  chassis.arcade(-50,0);
  pros::delay(100);
  // original: claw.move_velocity(-600); (claw flex wheel velocity -600)
  pros::delay(400);
  // original: clawToStage(4); (claw to stage 4)
  pros::delay(100);
  // original: liftToStage(4); (lift to stage 4)
  chassis.arcade(0,0);
  pros::delay(100);
  // circle_reset(24,15,8);
  pros::delay(5);
  chassis.arcade(60,0);
  pros::delay(300);
  // original: liftToStage(2); (lift to stage 2)
  // original: clawToStage(2); (claw to stage 2)
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  chassis.turnToHeading(75,400, {.minSpeed=110});
  //score the thing and raise after
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.moveToPoint(-15,28,900, {.forwards=false, .minSpeed=120});
  // original: liftToStage(2); (lift to stage 2)
  // original: clawToStage(2); (claw to stage 2)
  pros::delay(800);
  // original: clawToStage(3); (claw to stage 3)
  // original: liftToStage(1); (lift to stage 1)
  // original: heightMode=0; (lift height mode set to 0)
  // go to positon to score alliance goal pin
  chassis.moveToPoint(-18,17,700, {.forwards=false, .minSpeed=120});
  chassis.waitUntilDone();
  chassis.arcade(-80,0);
  // original: claw.move_velocity(-400); (claw flex wheel velocity -400)
  pros::delay(300);
  chassis.arcade(0,0);
  pros::delay(100);
  // original: clawToStage(3); (claw to stage 3)
  // original: liftToStage(2); (lift to stage 2)
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  // circle_reset(-24,15,8);
  pros::delay(50);
  // original: clawToStage(4); (claw to stage 4)
  // original: intake.move_velocity(0); (intake velocity 0)
  chassis.arcade(0,0);
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  chassis.moveToPoint(-11, 30, 500, {.minSpeed=90});
  chassis.turnToHeading(120,400, {.minSpeed=120});
  chassis.waitUntilDone();
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  // original: heightMode=0; (lift height mode set to 0)
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(4); (claw to stage 4)
  //X mode ready to grab the thing
  chassis.moveToPose(-20, 37,147, 800, {.forwards=false, .minSpeed=80});//GRAB FIRST
  chassis.waitUntilDone();
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(3); (claw to stage 3)
  // grab //////////
  pros::delay(450);
  // original: heightMode=0; (lift height mode set to 0)
  // original: liftToStage(3); (lift to stage 3)
  // lift to score alliance stake with pin
  chassis.moveToPoint(-52,36,900, {.forwards=false, .minSpeed=120});
  chassis.waitUntilDone();
  chassis.arcade(-100,0);
  pros::delay(50);
  chassis.arcade(-50,0);
  // original: claw.move_velocity(-600); (claw flex wheel velocity -600)
  // original: clawToStage(3); (claw to stage 3)
  pros::delay(300);
  chassis.arcade(0,0);
  pros::delay(50);
  // original: clawToStage(4); (claw to stage 4)
  // circle_reset(-48,39,8);
  chassis.arcade(0,0);
  //SCORE pls
  chassis.moveToPoint(-30,40, 700, {.forwards=true, .minSpeed=120}); // GRAB SECOND
  chassis.waitUntilDone();
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(4); (claw to stage 4)
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  chassis.turnToHeading(32,400, {.minSpeed=100});
  chassis.waitUntilDone();
  // original: heightMode=0; (lift height mode set to 0)
  // original: clawToStage(4); (claw to stage 4)
  // ready to grab
  chassis.moveToPoint(-52, 19, 1000, {.forwards=false, .minSpeed=100});
  pros::delay(850);
  // original: liftToStage(1); (lift to stage 1)
  // original: clawToStage(3); (claw to stage 3)
  // grab
  pros::delay(300);
  chassis.turnToHeading(-75,500, {.minSpeed=100});
  // original: liftToStage(3); (lift to stage 3)
  chassis.moveToPoint(-24, 17, 700, {.forwards=false, .minSpeed=127});
  chassis.waitUntilDone();
  // original: claw.move_velocity(-600); (claw flex wheel velocity -600)
  chassis.arcade(-80,0);
  pros::delay(100);
  chassis.arcade(0,0);
  //SCORE
  //score the pin
  return;
}

void fivepin() {
  set_claw_state(ClawState::INTAKING);
  chassis.setBrakeMode(
  pros::E_MOTOR_BRAKE_HOLD
  );
  // original: clawRotate.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); (claw rotation brake mode set to HOLD)
  chassis.setPose(
  0.0,
  0.0,
  0.0
  );
  // original: manualRun(-12000, 400); (lift manual voltage run: -12000mV for 400ms)
  // original: clawToStage(3); (claw to stage 3)
  // original: manualRun(-3000, 100); (lift manual voltage run: -3000mV for 100ms)
  // original: manualRun(5000, 200); (lift manual voltage run: 5000mV for 200ms)
  // original: liftToStage(3); (lift to stage 3)
  pros::delay(200);
  // original: heightMode=1; (lift height mode set to 1)
  chassis.arcade(100,0);
  pros::delay(300);
  // original: liftToStage(1); (lift to stage 1)
  set_level(1, init_heights::NEUTRAL);
  // original: clawToStage(2); (claw to stage 2)
  set_claw_tilt(claw_angle::LOADING);
  // original: intake.move_velocity(600); (intake velocity 600)
  intake.move(127);
  //start running claw and intake
  //A / loading position
  intake.disable_jam_detection();
  chassis.moveToPoint(0, 50, 3000, {.maxSpeed = 50, .minSpeed = 35});
  chassis.waitUntilDone();

  pros::delay(600);
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  chassis.turnToHeading(-42, 400, {.minSpeed=80});// turn to goal
  //raise lift to state 3 and position claw
  // original: heightMode=1; // mid mode (lift height mode set to 1)
  chassis.moveToPoint(20,20,950, {.forwards=false, .minSpeed=120}); //go to goal
  pros::delay(425);
  intake.set_mode(IntakeMode::STOPPED);
  // original: liftToStage(3); (lift to stage 3)
  set_level(3, init_heights::NEUTRAL);
  // original: clawToStage(3); (claw to stage 3)
  set_claw_tilt(claw_angle::SCORING - 2.5);
  chassis.waitUntilDone();
  chassis.arcade(-50,0);
  pros::delay(100);
  // original: claw.move_velocity(-600); (claw flex wheel velocity -600)
  set_level(2, init_heights::NEUTRAL);
  pros::delay(900);
  set_claw_state(ClawState::OUTAKING);
  pros::delay(400);
  set_level(3.25, init_heights::NEUTRAL);
  pros::delay(400);
  intake.enable_jam_detection();
  // original: clawToStage(4); (claw to stage 4)
  set_claw_tilt(claw_angle::SCORING - 5);
  pros::delay(100);
  // original: liftToStage(4); (lift to stage 4)
  set_level(3, init_heights::NEUTRAL);
  chassis.arcade(0,0);
  pros::delay(100);
  // circle_reset(24,15,8);
  circle_reset(UNDECLRAED_GOAL(8), 24, 15);

  pros::delay(5);
  chassis.arcade(60,0);
  pros::delay(300);
  // original: liftToStage(2); (lift to stage 2)
  set_level(1, init_heights::NEUTRAL);
  // original: clawToStage(2); (claw to stage 2)
  set_claw_tilt(claw_angle::LOADING);
  // original: claw.move_velocity(600); (claw flex wheel velocity 600)
  set_claw_state(ClawState::INTAKING);
  intake.set_mode(IntakeMode::INTAKE);
  chassis.turnToHeading(75,400, {.minSpeed=110});
  //score the thing and raise after
  pros::delay(200);
  chassis.moveToPoint(-15,16,900, {.forwards=false, .minSpeed=120});
  // original: liftToStage(2); (lift to stage 2)
  set_level(1, init_heights::NEUTRAL);
  // original: clawToStage(2); (claw to stage 2)
  set_claw_tilt(claw_angle::LOADING);
  pros::delay(800);
  // original: clawToStage(3); (claw to stage 3)
  set_claw_tilt(claw_angle::CLIMB);
  // original: liftToStage(1); (lift to stage 1)
  set_level(1, init_heights::NEUTRAL);
  // original: heightMode=0; (lift height mode set to 0)
  // go to positon to score alliance goal pin
  chassis.waitUntilDone();
  chassis.arcade(-80,0);
  // original: claw.move_velocity(-400); (claw flex wheel velocity -400)
  set_claw_state(ClawState::INTAKING);
  pros::delay(300);
  chassis.arcade(0,0);
  pros::delay(100);
  // original: clawToStage(3); (claw to stage 3)
  set_claw_tilt(claw_angle::SCORING - 9);
  // original: liftToStage(3); (lift to stage 3)
  set_level(2, init_heights::ALLIANCE);
  pros::delay(500);
  set_level(1, init_heights::ALLIANCE);
  pros::delay(300);
  set_claw_state(ClawState::OUTAKING);
  pros::delay(500);
  chassis.arcade(60,0);
  pros::delay(500);
  chassis.arcade(0,0);
}