#include "ez_chassis.hpp"
#include <algorithm>
#include <cmath>

namespace {
int resolve_timeout(const int timeout_ms, const int default_timeout_ms) {
    return timeout_ms > 0 ? timeout_ms : default_timeout_ms;
}
} // namespace

float EzChassis::heading_to_point(const float x, const float y) const {
    const lemlib::Pose pose = chassis_.getPose();
    // LemLib's 0deg = north/+y, matching the sin/cos convention already used
    // elsewhere in this codebase (e.g. grape.cpp's circle_reset()).
    return lemlib::radToDeg(std::atan2(x - pose.x, y - pose.y));
}

float EzChassis::angular_distance_to(const float target_heading_deg) const {
    const lemlib::Pose pose = chassis_.getPose();
    // Normalize to the shortest signed distance in [-180, 180].
    float diff = std::fmod(target_heading_deg - pose.theta + 540.0f, 360.0f) - 180.0f;
    return diff;
}

void EzChassis::pid_drive_set(const float distance_in, const int speed, const int timeout_ms) {
    const lemlib::Pose pose = chassis_.getPose();
    const float theta_rad = lemlib::degToRad(pose.theta);

    // Project a target point `distance_in` inches out along the robot's
    // current heading.
    const float target_x = pose.x + distance_in * std::sin(theta_rad);
    const float target_y = pose.y + distance_in * std::cos(theta_rad);

    last_motion_ = MotionType::DRIVE;
    last_motion_extent_ = std::abs(distance_in);

    chassis_.moveToPoint(
        target_x, target_y,
        resolve_timeout(timeout_ms, default_timeout_ms_),
        {.forwards = distance_in >= 0, .maxSpeed = static_cast<float>(std::abs(speed))},
        true // async — mirrors EZ-Template's non-blocking pid_drive_set; call pid_wait() to block
    );
}

void EzChassis::pid_turn_set(const float heading_deg, const int speed, const int timeout_ms) {
    last_motion_ = MotionType::TURN;
    last_motion_extent_ = std::abs(angular_distance_to(heading_deg));

    chassis_.turnToHeading(
        heading_deg,
        resolve_timeout(timeout_ms, default_timeout_ms_),
        {.maxSpeed = static_cast<float>(std::abs(speed))},
        true
    );
}

void EzChassis::pid_turn_set(const float x, const float y, const int speed, const int timeout_ms) {
    last_motion_ = MotionType::TURN;
    last_motion_extent_ = std::abs(angular_distance_to(heading_to_point(x, y)));

    chassis_.turnToPoint(
        x, y,
        resolve_timeout(timeout_ms, default_timeout_ms_),
        {.maxSpeed = static_cast<float>(std::abs(speed))},
        true
    );
}

void EzChassis::set_swing(const ez::e_swing side, const float heading_deg, const int speed, const int timeout_ms) {
    const auto locked_side = (side == ez::LEFT_SWING) ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT;

    last_motion_ = MotionType::SWING;
    last_motion_extent_ = std::abs(angular_distance_to(heading_deg));

    chassis_.swingToHeading(
        heading_deg, locked_side,
        resolve_timeout(timeout_ms, default_timeout_ms_),
        {.maxSpeed = static_cast<float>(std::abs(speed))},
        true
    );
}

void EzChassis::set_swing(const ez::e_swing side, const float x, const float y, const int speed, const int timeout_ms) {
    const auto locked_side = (side == ez::LEFT_SWING) ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT;

    last_motion_ = MotionType::SWING;
    last_motion_extent_ = std::abs(angular_distance_to(heading_to_point(x, y)));

    chassis_.swingToPoint(
        x, y, locked_side,
        resolve_timeout(timeout_ms, default_timeout_ms_),
        {.maxSpeed = static_cast<float>(std::abs(speed))},
        true
    );
}

namespace {
lemlib::AngularDirection resolve_direction(const ez::e_angular_dir dir) {
    switch (dir) {
        case ez::cw: return lemlib::AngularDirection::CW_CLOCKWISE;
        case ez::ccw: return lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
        default: return lemlib::AngularDirection::AUTO;
    }
}
} // namespace

void EzChassis::pid_turn_set(const float heading_deg, const int speed, const ez::e_angular_dir dir, const int timeout_ms) {
    last_motion_ = MotionType::TURN;
    last_motion_extent_ = std::abs(angular_distance_to(heading_deg));

    chassis_.turnToHeading(
        heading_deg,
        resolve_timeout(timeout_ms, default_timeout_ms_),
        {.direction = resolve_direction(dir), .maxSpeed = static_cast<float>(std::abs(speed))},
        true
    );
}

void EzChassis::pid_turn_set(const float x, const float y, const int speed, const ez::e_angular_dir dir, const int timeout_ms) {
    last_motion_ = MotionType::TURN;
    last_motion_extent_ = std::abs(angular_distance_to(heading_to_point(x, y)));

    chassis_.turnToPoint(
        x, y,
        resolve_timeout(timeout_ms, default_timeout_ms_),
        {.direction = resolve_direction(dir), .maxSpeed = static_cast<float>(std::abs(speed))},
        true
    );
}

void EzChassis::set_swing(const ez::e_swing side, const float heading_deg, const int speed, const ez::e_angular_dir dir, const int timeout_ms) {
    const auto locked_side = (side == ez::LEFT_SWING) ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT;

    last_motion_ = MotionType::SWING;
    last_motion_extent_ = std::abs(angular_distance_to(heading_deg));

    chassis_.swingToHeading(
        heading_deg, locked_side,
        resolve_timeout(timeout_ms, default_timeout_ms_),
        {.direction = resolve_direction(dir), .maxSpeed = static_cast<float>(std::abs(speed))},
        true
    );
}

void EzChassis::set_swing(const ez::e_swing side, const float x, const float y, const int speed, const ez::e_angular_dir dir, const int timeout_ms) {
    const auto locked_side = (side == ez::LEFT_SWING) ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT;

    last_motion_ = MotionType::SWING;
    last_motion_extent_ = std::abs(angular_distance_to(heading_to_point(x, y)));

    chassis_.swingToPoint(
        x, y, locked_side,
        resolve_timeout(timeout_ms, default_timeout_ms_),
        {.direction = resolve_direction(dir), .maxSpeed = static_cast<float>(std::abs(speed))},
        true
    );
}

void EzChassis::pid_wait() {
    chassis_.waitUntilDone();
}

void EzChassis::pid_wait_quick_chain() {
    if (last_motion_ == MotionType::NONE) return; // nothing commanded yet, nothing to wait for

    const float chain_range = (last_motion_ == MotionType::DRIVE) ? drive_chain_range_ : turn_chain_range_;
    const float wait_point = std::max(last_motion_extent_ - chain_range, 0.0f);
    chassis_.waitUntil(wait_point);
}
