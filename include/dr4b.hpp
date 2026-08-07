// FourBar.hpp

#pragma once
#include "lemlib/pid.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/motor_group.hpp"
#include <algorithm>
#include <cmath>
#include <functional>

namespace init_heights {
  constexpr float NEUTRAL = 12;
  constexpr float ALLIANCE = 5;
  constexpr float MIDDLE = 26;
  constexpr float INTAKE = 0;
}

inline float get_height_at_level(const float level, const float init_height, const float height_increment = 20) {
    return level > 1 ? init_height + height_increment * (level - 2) : init_heights::INTAKE;
}

class Dr4b {
private:
    std::function<void(float)> move_callback_;
    std::function<float()> position_callback_;
    lemlib::PID pid_;
    float target_ {};
    float stopping_threshold_;
    float feed_forward_ {};
    float min_position_ {};
    float max_position_ {};
public:
    Dr4b(
        const float kp, const float ki, const float kd,
        const std::function<void(float)> move_callback,
        const std::function<float()> position_callback,
        const float feed_forward = 0,
        const float min_position = 0,
        const float max_position = 100,
        const float stopping_threshold = 1
    ) : pid_{kp, ki, kd},
        stopping_threshold_{stopping_threshold},
        move_callback_{move_callback},
        position_callback_{position_callback}, feed_forward_{feed_forward}, max_position_{max_position}, min_position_{min_position}, target_{min_position} {}
public:
    float get_target() {
        return target_;
    }
    void set_target(const float target) {
        target_ = std::clamp(target, min_position_, max_position_);
    }

    float get_stopping_threshold() {return stopping_threshold_;}
    void set_stopping_threshold(const float stopping_threshold) {stopping_threshold_ = stopping_threshold;}

    bool is_within_target_threshold() {
        return std::abs(target_ - position_callback_()) <= stopping_threshold_;
    }

    void reset() {
        pid_.reset();
    }

    void update();
};