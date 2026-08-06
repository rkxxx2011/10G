#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include "claw.hpp"
#include "dr4b.hpp"
#include "orchestrator.hpp"
#include "anti_tip.hpp"
#include "intake.hpp"

// ==================== CHASSIS CONFIGURATION ====================
// External declaration of the EZ-Template chassis object

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::IMU imu;
extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

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