#include "anti_tip.hpp"
#include <algorithm>
#include <cmath>

AntiTip::AntiTip(
    pros::Imu* imu,
    pros::MotorGroup* left_motors,
    pros::MotorGroup* right_motors,
    double pitch_thresh,
    double roll_thresh,
    int max_voltage
) : imu_ {imu},
    left_motors_ {left_motors},
    right_motors_ {right_motors},
    pitch_thresh_ {pitch_thresh},
    roll_thresh_ {roll_thresh},
    max_voltage_ {max_voltage} {}

bool AntiTip::update() {
    if (!imu_ || !left_motors_ || !right_motors_) return false;

    // pros::Imu::get_pitch()/get_roll() return degrees, signed relative to
    // level (0). Which sign corresponds to "tipping forward" vs "tipping
    // backward" depends on how the IMU is mounted on your chassis — this has
    // NOT been verified on real hardware. Test at low speed first: if the
    // correction makes a tip worse instead of catching it, flip the sign on
    // the line marked below.
    // Never command the drivetrain from invalid or calibrating IMU data.
    if (imu_->is_calibrating()) return false;

    const double pitch = imu_->get_pitch();
    const double roll = imu_->get_roll();
    if (!std::isfinite(pitch) || !std::isfinite(roll)) return false;

    if (std::abs(pitch) >= pitch_thresh_) {
        // Drive both sides the same direction (straight back toward level,
        // no turning) at max_voltage, opposing the tip direction.
        // FLIP THIS SIGN if testing shows it drives the wrong way:
        int correction = (pitch > 0) ? -max_voltage_ : max_voltage_;

        left_motors_->move_voltage(correction);
        right_motors_->move_voltage(correction);
        return true;
    }

    if (std::abs(roll) >= roll_thresh_) {
        // A two-side tank drivetrain can't actively correct a sideways
        // (roll) tip by driving forward/backward — both sides move together.
        // Rather than pretend to "fix" it, this just cuts drive power to
        // reduce the chance of making a sideways tip worse; it does not
        // actively right the robot. If you need real roll correction,
        // that generally requires a different chassis config (e.g. a swerve
        // or holonomic drive) or a mechanical solution (wider base, outriggers).
        left_motors_->move_voltage(0);
        right_motors_->move_voltage(0);
        return true;
    }

    return false;
}
