#pragma  once

#include "ports.hpp"
#include "pros/rtos.hpp"

extern pros::Mutex lift_mutex;
extern pros::Mutex claw_mutex;
extern pros::Task lift_task;
extern pros::Task claw_task;