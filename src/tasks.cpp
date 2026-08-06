// Lift control is updated once through orchestrator.update() in opcontrol.cpp.
// Keeping a second global lift task here caused two loops to command the same motors.

#include "ports.hpp"
#include "pros/rtos.hpp"

pros::Mutex lift_mutex {};
pros::Mutex claw_mutex {};

pros::Task lift_task([]() {
    
    while (true) {
        lift_mutex.take();

        lift.update();

        lift_mutex.give();

        pros::delay(10);
    }
});

pros::Task claw_task([]() {
    while (true) {
        claw_mutex.take();

        claw.update();

        claw_mutex.give();

        pros::delay(10);
    }
});