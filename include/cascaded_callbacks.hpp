#pragma once

#include "pros/motors.h"
#include "pros/motor_group.hpp"

namespace cascaded_callbacks {

extern void move_callback(pros::MotorGroup&, float target_velocity, float max_motor_rpm);
extern void move_callback(pros::Motor&, float target_velocity, float max_motor_rpm);

} // namespace cascaded_callbacks