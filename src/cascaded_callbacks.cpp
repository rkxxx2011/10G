#include "cascaded_callbacks.hpp"

void cascaded_callbacks::move_callback(pros::MotorGroup& motors, const float target_velocity, const float max_motor_rpm) {
    motors.move_velocity(target_velocity / 127.0 * max_motor_rpm);
    // use move_velocity to use the built in pid to go target velocity
    // this means that even though we only have one pid for the lift in this codebase
    // it is technically cascaded because of the internal one
}

void cascaded_callbacks::move_callback(pros::Motor& motor, const float target_velocity, const float max_motor_rpm) {
    motor.move_velocity(target_velocity / 127.0 * max_motor_rpm);
    // use move_velocity to use the built in pid to go target velocity
    // this means that even though we only have one pid for the lift in this codebase
    // it is technically cascaded because of the internal one
}