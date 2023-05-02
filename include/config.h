#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

extern pros::Controller master;
extern pros::Controller partner;

//motors
extern pros::Motor l1;
extern pros::Motor l2;
extern pros::Motor l3;
extern pros::Motor r1;
extern pros::Motor r2;
extern pros::Motor r3;


// extern sylib::Addrled leftSkirt;
// extern sylib::Addrled rightSkirt;


extern sylib::Motor flywheel;


extern sylib::Addrled caracal;


extern sylib::Addrled letters_left;
extern sylib::Addrled letters_right;


extern sylib::Addrled letters_back;

extern sylib::Addrled left_track;
extern sylib::Addrled right_track;

extern pros::ADIDigitalOut blooper;
extern pros::ADIDigitalOut string;
extern pros::ADIDigitalOut zapper;

extern pros::ADIDigitalIn sled_sensor;


extern pros::Motor intake;
extern pros::Motor spool;

extern pros::Motor_Group leftDrive;
extern pros::Motor_Group rightDrive;

extern pros::Gps gps;
//sensors
extern pros::Distance diskSensor;

extern int trackControlMode;

//functions
void drive(double left, double right);
void intakeCont();
void odomControlLoop(void * param);
void turnToAngle();
void fwControl();
void skillsAuto();

void sledControl();
void simpleRollerAutoFar();
void simpleRollerAutoClose();
void fullAWP();
void farSide();
void closeSideHalf();
int senseDisks();
void pulseTrackLights();

void matchLoads();
// [REDACTED]
// [REDACTED]
extern lemlib::Drivetrain_t drivetrain;
extern pros::Imu imu;
extern lemlib::OdomSensors_t odom;
extern lemlib::Chassis robor;
extern lemlib::ChassisController_t lateralController;
extern lemlib::ChassisController_t angularController;
