#include "ports.hpp"
#include "cascaded_callbacks.hpp"
#include <memory>

pros::MotorGroup left_motors {{-1, -4}, pros::MotorGearset::green};
pros::MotorGroup right_motors {{2, 3}, pros::MotorGearset::green};
pros::IMU imu {6};
pros::Imu& imu_sensor = imu; // alias — same physical sensor, avoids a second IMU object on the same port
pros::Rotation odom_rotation_sensor {5};

lemlib::Drivetrain drivetrain {&left_motors, // left motor group
                              &right_motors, // right motor group
                              12, // track width in inches — measure and tune on the real robot
                              lemlib::Omniwheel::NEW_325, // 3.25-inch drive wheels
                              200.f * 60.f / 36.f, // effective drivetrain RPM: about 333
                              4 // horizontal drift — tune using LemLib's calibration procedure
};

// Anti-tip: pitch_thresh=35deg, roll_thresh=15deg, max_voltage=8000mV (~67% power).
// These three numbers are untested starting guesses, carried over from the old
// placeholder line — tune them on the real robot (see anti_tip.cpp for how
// they're used).
AntiTip anti_tip {&imu_sensor, &left_motors, &right_motors, 35, 15, 8000};
pros::Motor intake_motor(-7);
pros::MotorGroup lift_motors({16, -15}, pros::MotorGearset::blue); // verify direction — flip the sign of whichever port fights the other if the two lift motors aren't matched
pros::Rotation lift_rotation_sensor(-13);
pros::Motor claw_flex_wheels {17};
pros::Motor claw_rotation_motor {19};
pros::Rotation claw_rotation_sensor {18};


// Port 4 is a vertical tracking-wheel rotation sensor. The diameter and offset
// must match the physical wheel. Start with these values, then measure/tune them.
static constexpr double ODOM_WHEEL_OFFSET = 0.0;
static lemlib::TrackingWheel vertical_tracking_wheel {
    &odom_rotation_sensor,
    lemlib::Omniwheel::NEW_2,
    ODOM_WHEEL_OFFSET
};

static lemlib::OdomSensors odom_pods {
    &vertical_tracking_wheel,
    nullptr,
    nullptr,
    nullptr,
    &imu
};

// lateral PID controller
static lemlib::ControllerSettings lateral_controller {15, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
};

// angular PID controller
static lemlib::ControllerSettings angular_controller {2.9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              26, // derivative gain (kD)
                                              3, // anti windup
                                              2, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
};

lemlib::Chassis chassis {drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        odom_pods // odometry sensors
};

// EZ-Template-style wrapper around the chassis above — see ports.hpp for how
// this relates to `chassis`.
EzChassis ez_chassis {chassis};

pros::Controller ctrler {pros::E_CONTROLLER_MASTER};

Claw claw {
    2, 0, 0,
    [](const auto target_rpm) {
        claw_rotation_motor.move(target_rpm);
    },
    []() {
        return claw_rotation_sensor.get_position() / 100.0;
    },
    [](const auto target_power) {
        claw_flex_wheels.move(target_power);
    },
    // Gravity-holding feedforward (out of -127..127) — previously nonexistent
    // on the claw entirely, meaning nothing fought gravity and it could only
    // hold a target by sagging toward 0/face-down first. UNTESTED starting
    // guess — tune on the real robot: still sags -> increase, drifts/creeps
    // upward at rest -> decrease.
    10
};

// The rotation sensor is mounted on the 24-tooth center gear. The original
// project used a 3:1 sensor-to-lift conversion, so that existing conversion is
// preserved here without assuming an unconfirmed mating-gear tooth count.
static constexpr float LIFT_SENSOR_TO_LIFT_RATIO = 3.0f;

Dr4b lift {
    5, 0, 7,

//  kd        kp
    [](const auto motor_power) {
        // Use direct power for the lift. A gravity feedforward value represents
        // holding torque and must not be interpreted as a target velocity.
        lift_motors.move(static_cast<std::int32_t>(motor_power));
    },
    []() {
        return static_cast<float>(lift_rotation_sensor.get_position()) / 100.0f;
    },
    // Gravity-holding feedforward (out of -127..127, same units as motor_power
    // above). This was previously left at its default of 0, meaning nothing
    // actively fought gravity — the lift only held a level by sagging just
    // far enough for the resulting PID error to generate holding torque,
    // which gets worse the higher/more extended the arm is (visible as the
    // arm drooping around the 4th stack and up). This is an UNTESTED
    // starting guess — tune it up/down on the real robot: too low and it'll
    // still sag, too high and it'll drift/creep upward on its own at rest.
    15
};

// Non-owning shared_ptrs: these just wrap the existing globals above so
// Orchestrator can hold shared_ptrs without us restructuring lift/claw/ctrler
// into heap-allocated objects. The no-op deleters mean Orchestrator never
// tries to delete these globals.
Orchestrator orchestrator {
    std::shared_ptr<Dr4b>(&lift, [](Dr4b*) {}),
    std::shared_ptr<Claw>(&claw, [](Claw*) {}),
    std::shared_ptr<pros::Controller>(&ctrler, [](pros::Controller*) {})
};

Intake intake(
    [](const int32_t v) {intake_motor.move(v);},
    []() {return intake_motor.get_actual_velocity();},
    20
);