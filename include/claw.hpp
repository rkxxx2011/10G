#pragma once

#include "api.h"
#include "lemlib/pid.hpp"
#include <cstdint>
#include <functional>
#include <algorithm>
#include <cmath>

enum class ClawState : int32_t {
    STOPPED = 0, //power level
    INTAKING = 127, //power level
    OUTAKING = -80 //power level — softer than full power so the pin drops instead of flinging
};

namespace claw_angle {
    static inline constexpr float LOADING = 0; // loading position angle
    // Positive — the claw's real ~120 degrees of free travel from loading is
    // on this side; the negative direction jams within a few degrees.
    static inline constexpr float SCORING = 95; // scoring position angle
    static inline constexpr float CLIMB = 120; // the claw's real max travel — as high as it can go
};

class Claw {
private:
    ClawState state_ {ClawState::STOPPED};
    lemlib::PID angular_pid_;
    float target_angle_ = 0;
    float stopping_threshold_ = 0;
    float feed_forward_ = 0;
    std::function<void(float)> rotate_callback_;
    std::function<void(float)> flex_wheel_callback_;
    std::function<float()> position_callback_;

    // Low-pass filtered sensor reading. Smooths out single-tick noise spikes
    // (e.g. magnetic interference from nearby motors) so a bad reading causes
    // a small nudge instead of a sudden jerk.
    bool filter_initialized_ = false;
    float filtered_position_ = 0;
public:
    Claw(
        const float kp, const float ki, const float kd,
        const std::function<void(float)> rotate_callback,
        const std::function<float()> position_callback,
        const std::function<void(float)> flex_wheel_callback,
        const float feed_forward = 0,
        const float stopping_threshold = 1
    ) : angular_pid_ {kp, ki, kd},
        stopping_threshold_ {stopping_threshold},
        feed_forward_ {feed_forward},
        rotate_callback_ {rotate_callback},
        position_callback_ {position_callback},
        flex_wheel_callback_ {flex_wheel_callback} {}
public:
    ClawState get_state() {return state_;}
    void set_state(const ClawState state) {state_ = state;}

    float get_target_angle() {
        return target_angle_;
    }
    void set_target_angle(const float target_angle) {
        target_angle_ = target_angle;
    }

    float get_feed_forward() {return feed_forward_;}
    void set_feed_forward(const float feed_forward) {feed_forward_ = feed_forward;}

    float get_stopping_threshold() {return stopping_threshold_;}
    void set_stopping_threshold(const float stopping_threshold) {stopping_threshold_ = stopping_threshold;}

    bool is_within_stopping_threshold() {
        return std::abs(target_angle_ - position_callback_()) <= stopping_threshold_;
    }

    void update();
};