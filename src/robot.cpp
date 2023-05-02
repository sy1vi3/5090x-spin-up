#include "config.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include "pros/apix.h"


pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);


//motors 



// sylib::Addrled leftSkirt(22,7,32);
// sylib::Addrled rightSkirt(22,6,32);
sylib::SpeedControllerInfo flywheelController (
        [](double rpm){return std::pow(M_E, (-0.001*rpm* 3600 / 3600 + 1)) + 3.065;}, // kV function
        10, // kP
        0.001, // kI
        0, // kD
        0, // kH
        true, // anti-windup enabled
        50, // anti-windup range
        true, // p controller bounds threshold enabled
        50, // p controller bounds cutoff enabled
        0.01, // kP2 for when over threshold
        50, // range to target to apply max voltage
        false, // coast down enabled
        0,  // coast down theshhold
        1 // coast down constant
);

sylib::Motor flywheel(1, 3600,false, flywheelController);


sylib::Addrled caracal(22,3,52);


sylib::Addrled letters_left(2,1,18);
sylib::Addrled letters_right(2,3,18);


sylib::Addrled letters_back(2,2,32);

sylib::Addrled left_track(22,6,32);
sylib::Addrled right_track(22,8,32);


pros::Motor intake(10);

pros::Motor l1(11,pros::E_MOTOR_GEARSET_06,true);
pros::Motor l2(12,pros::E_MOTOR_GEARSET_06,true);
pros::Motor l3(13,pros::E_MOTOR_GEARSET_06,true);
pros::Motor r1(18,pros::E_MOTOR_GEARSET_06, false);
pros::Motor r2(19,pros::E_MOTOR_GEARSET_06, false);
pros::Motor r3(20,pros::E_MOTOR_GEARSET_06, false);

pros::Motor_Group rightDrive({r1, r2, r3});
pros::Motor_Group leftDrive({l1, l2, l3});

pros::ADIDigitalOut blooper(1, false);
pros::ADIDigitalOut string(2, false);
pros::ADIDigitalOut zapper({2,8}, false);


pros::Distance diskSensor(9);

int trackControlMode = 0;


pros::Gps gps(4);
pros::Imu imu(15);

lemlib::Drivetrain_t drivetrain{
    &leftDrive, // left drivetrain motors
    &rightDrive, // right drivetrain motors
    10.375, // track width
    3.318, // wheel diameter
    360 // wheel rpm  
};

lemlib::OdomSensors_t odom{
  nullptr,
  nullptr,
  nullptr,
  nullptr,
  &imu
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    10, // kP
    50, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    5, // kP
    25, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

lemlib::Chassis robor(drivetrain, lateralController, angularController, odom);

// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]
// [REDACTED]

//sensors
