#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include "claw.hpp"
#include "dr4b.hpp"
#include "orchestrator.hpp"
#include "anti_tip.hpp"
#include "intake.hpp"
#include "ez_chassis.hpp"

// ==================== CHASSIS CONFIGURATION ====================
// `chassis` is the native LemLib chassis object — all real motion math,
// PID, and odometry live here.
//
// `ez_chassis` is an EZ-Template-style wrapper around the SAME chassis (see
// ez_chassis.hpp) — it just gives autonomous code the option to call
// pid_drive_set/pid_turn_set/set_swing/pid_wait/pid_wait_quick_chain instead
// of LemLib's native names. Both objects drive the same physical robot, so
// use whichever naming reads better for a given routine — just don't
// command both at the same time on top of each other.

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::IMU imu;
extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;
extern EzChassis ez_chassis;

extern pros::Controller ctrler;

// ==================== SUBSYSTEM MOTORS ====================
extern pros::MotorGroup lift_motors;

// ==================== SENSORS & EXTRA ====================
extern pros::Imu& imu_sensor; // same physical IMU as `imu` on port 1 — just an alternate name
extern pros::Rotation lift_rotation_sensor;
extern pros::Rotation odom_rotation_sensor;

// External declaration for tracking wheels if used in multiple files

// ==================== UTILITY CLASSES ====================

extern pros::Motor claw_flex_wheels;
extern pros::Motor claw_rotation_motor;
extern pros::Rotation claw_rotation_sensor;
 
extern Claw claw;

extern Dr4b lift;

extern Orchestrator orchestrator;

extern AntiTip anti_tip;

extern Intake intake;

extern pros::Motor intake_motor;