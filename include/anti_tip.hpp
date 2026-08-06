#pragma once
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

// Monitors chassis pitch via the IMU and, if the robot tips past a threshold
// (e.g. hard acceleration/deceleration, or lift extended forward shifting the
// center of gravity), drives the drivetrain back toward level. Call update()
// once per opcontrol tick, AFTER your normal joystick drive code — when it's
// not actively correcting, it does nothing and leaves your joystick commands
// in effect; when it detects a tip, it overrides them for that tick.
class AntiTip {
public:
    AntiTip(
        pros::Imu* imu,
        pros::MotorGroup* left_motors,
        pros::MotorGroup* right_motors,
        double pitch_thresh,
        double roll_thresh,
        int max_voltage
    );

    // Returns true if it took over the drivetrain this tick (driver joystick
    // input was overridden to correct a tip); false if the chassis is level
    // enough that driver control was left alone.
    bool update();

private:
    pros::Imu* imu_;
    pros::MotorGroup* left_motors_;
    pros::MotorGroup* right_motors_;
    double pitch_thresh_;
    double roll_thresh_;
    int max_voltage_;
};
