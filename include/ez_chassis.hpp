// ez_chassis.hpp
//
// Thin EZ-Template-style wrapper around a lemlib::Chassis. Lets autonomous
// code read with EZ-Template's naming/motion-chaining conventions —
// pid_drive_set / pid_turn_set / set_swing / pid_wait / pid_wait_quick_chain
// — while all the actual motion math, PID, and odometry underneath stay
// 100% LemLib. This is a SEPARATE object from `chassis` — it just holds a
// reference to the same lemlib::Chassis, so grape.cpp can freely mix
// EZ-style calls (via `ez_chassis`) and native LemLib calls (via `chassis`)
// in the same routine without anything fighting.

#pragma once
#include "lemlib/api.hpp"

namespace ez {
// Mirrors EZ-Template's swing direction enum.
enum e_swing { LEFT_SWING, RIGHT_SWING };

// Mirrors EZ-Template's turn direction constants. Maps directly onto
// LemLib's own lemlib::AngularDirection — auto lets LemLib pick the
// shortest way to turn (the default when no direction is given).
enum e_angular_dir { cw, ccw, ez_auto };

// EZ-Template-style unit literals so autonomous code can write `-11_in` /
// `90_deg` instead of bare numbers. These are just float pass-throughs —
// pid_drive_set/pid_turn_set already treat their arguments as inches/degrees,
// so the literals exist purely for readability, matching EZ-Template's feel.
constexpr float operator""_in(const long double value) { return static_cast<float>(value); }
constexpr float operator""_in(const unsigned long long value) { return static_cast<float>(value); }
constexpr float operator""_deg(const long double value) { return static_cast<float>(value); }
constexpr float operator""_deg(const unsigned long long value) { return static_cast<float>(value); }
} // namespace ez

class EzChassis {
private:
    lemlib::Chassis& chassis_;

    // What kind of motion was last commanded — pid_wait_quick_chain() needs
    // this to know whether to measure in inches (drive) or degrees (turn/swing).
    enum class MotionType { NONE, DRIVE, TURN, SWING };
    MotionType last_motion_ {MotionType::NONE};

    // Total distance/angle of the last commanded motion, from wherever the
    // robot started that motion. LemLib's chassis.waitUntil(x) waits until
    // the robot has traveled x inches/degrees INTO the current motion (not
    // "x remaining"), so pid_wait_quick_chain() has to work out
    // (total extent - chain range) itself to return once the robot is
    // close to done rather than only just getting started.
    float last_motion_extent_ {0.0f};

    // How close (inches for drive, degrees for turn/swing) counts as "close
    // enough" for pid_wait_quick_chain() to return and let the next motion
    // get queued, instead of waiting for a full precise settle. Tune to
    // taste — smaller = waits longer/tighter, larger = chains sooner/looser.
    float drive_chain_range_ {3.0f};
    float turn_chain_range_ {8.0f};

    int default_timeout_ms_ {4000};

    // Absolute heading (degrees, LemLib convention) from the current pose to
    // point (x, y).
    float heading_to_point(float x, float y) const;
    // Shortest signed angular distance (degrees) from the current heading to
    // a target heading, normalized to [-180, 180].
    float angular_distance_to(float target_heading_deg) const;

public:
    explicit EzChassis(lemlib::Chassis& chassis) : chassis_(chassis) {}

    // ==================== STRAIGHT DRIVE ====================
    // Drives `distance_in` inches forward (negative = backward) along the
    // robot's CURRENT heading — relative motion, matching EZ-Template's
    // pid_drive_set. Internally projects a target point from the current
    // pose and hands it to LemLib's moveToPoint().
    void pid_drive_set(float distance_in, int speed = 127, int timeout_ms = -1);

    // ==================== TURNING ====================
    // Turn to an absolute field heading, in degrees.
    void pid_turn_set(float heading_deg, int speed = 127, int timeout_ms = -1);
    // Turn to face an absolute field point (x, y), in inches.
    void pid_turn_set(float x, float y, int speed = 127, int timeout_ms = -1);
    // Same as above, but forces a turn direction instead of letting LemLib
    // pick the shortest way — matches EZ-Template's cw/ccw argument.
    void pid_turn_set(float heading_deg, int speed, ez::e_angular_dir dir, int timeout_ms = -1);
    void pid_turn_set(float x, float y, int speed, ez::e_angular_dir dir, int timeout_ms = -1);

    // ==================== SWING TURNS ====================
    // Turn to an absolute heading while only one side of the drivetrain
    // drives and the other stays locked — uses LemLib's native
    // swingToHeading() under the hood.
    void set_swing(ez::e_swing side, float heading_deg, int speed = 127, int timeout_ms = -1);
    // Swing to face an absolute point instead of a fixed heading.
    void set_swing(ez::e_swing side, float x, float y, int speed = 127, int timeout_ms = -1);
    // Same as above, but forces a turn direction instead of the shortest way.
    void set_swing(ez::e_swing side, float heading_deg, int speed, ez::e_angular_dir dir, int timeout_ms = -1);
    void set_swing(ez::e_swing side, float x, float y, int speed, ez::e_angular_dir dir, int timeout_ms = -1);

    // ==================== WAITING / CHAINING ====================
    // Block until the most recently commanded motion has fully settled.
    void pid_wait();

    // Returns once the robot is "close enough" to the target of the most
    // recently commanded motion (see drive_chain_range_ / turn_chain_range_)
    // instead of waiting for a full settle. Call this instead of pid_wait()
    // right before your NEXT pid_drive_set/pid_turn_set/set_swing call, so
    // that next motion gets queued behind this one finishing up rather than
    // your autonomous code sitting idle waiting the full settle time first.
    // Built on LemLib's own chassis.waitUntil().
    void pid_wait_quick_chain();

    // Tune how close (inches / degrees) counts as "close enough" for
    // pid_wait_quick_chain().
    void set_chain_range(float drive_in, float turn_deg) {
        drive_chain_range_ = drive_in;
        turn_chain_range_ = turn_deg;
    }

    void set_default_timeout(int timeout_ms) { default_timeout_ms_ = timeout_ms; }
};
