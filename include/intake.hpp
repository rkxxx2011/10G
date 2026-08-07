#pragma once

#include <functional>
#include <cstdint>
#include "api.h"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include <cmath>

enum class IntakeMode : int32_t {
    STOPPED = 0,
    INTAKE = 127,
    OUTAKE = -127
};

class Intake {
private:
    IntakeMode mode_ = IntakeMode::STOPPED;
    std::function<void(int32_t)> move_callback_;
    std::function<float()> rpm_callback_;
    pros::Mutex mutex_ = {};
    float threshold_ = 0;
    bool is_detecting_jam_ = true;
    pros::Task task_{[&]() {
        size_t accel_time = 0;
        while (true) {
            if (mode_ == IntakeMode::OUTAKE) {
                move_callback_(-127);
            } else if (mode_ == IntakeMode::INTAKE) {
                move_callback_(127);
            } else if (mode_ == IntakeMode::STOPPED) {
                move_callback_(0);

                accel_time = 0;
            }

            pros::delay(10);
        }
    }};
public:
    Intake(const std::function<void(int32_t)> move_callback,
        const std::function<float()> rpm_callback,
        const float threshold)
    : move_callback_(move_callback), rpm_callback_(rpm_callback), threshold_(threshold) {}
public:
    IntakeMode get_mode() {
        mutex_.take();

        IntakeMode mode = mode_;

        mutex_.give();

        return mode;
    }
    void set_mode(const IntakeMode mode) {
        mutex_.take();

        mode_ = mode;

        mutex_.give();
    }

    void move(const int32_t v) {
        if (v == 0) {
            set_mode(IntakeMode::STOPPED);
        }

        set_mode(v > 0 ? IntakeMode::INTAKE : IntakeMode::OUTAKE);
    }

    void pause_task() {
        task_.suspend();
    }

    void resume_task() {
        task_.resume();
    }

    void disable_jam_detection() {
        is_detecting_jam_ = false;
    }

    void enable_jam_detection() {
        is_detecting_jam_ = true;
    }
};