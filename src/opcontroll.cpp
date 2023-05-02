#include "config.h"
#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "sylib/system.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <queue>
/*
    here is where driver control functions will go
    flywheel code is in a separate function because it will likely be complex
*/
double real_left = 0;
double real_right = 0;
double deadzone = 5;

int output_left = 0;
int output_right = 0;

int previous_left = 0;
int previous_right = 0;

int drive_ticks = 0;


void drive(double left, double right){
    drive_ticks++;
   
    if (std::abs(left) > deadzone){
      real_left=left*1.05 + 6.35;
    // real_left = std::pow(left, 3)/17000;
      leftDrive.move_voltage(real_left*100);
    }
    else {
      leftDrive.move_velocity(0);
    }

    if (std::abs(right) > deadzone){
      real_right=right*1.05 + 6.35;
    // real_right = std::pow(right, 3)/17000;
    //   if(std::abs(real_right > pre))
      previous_right = real_right;
      rightDrive.move_voltage(real_right*100);
    }
    else{
        previous_right = 0;
      rightDrive.move_velocity(0);
    }

}
int timeSinceLastFlash = 0;
void intakeCont()
{
    if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        intake.move_voltage(12000);
    }
    else if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intake.move_voltage(-12000);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        intake.move_voltage(12000);
    } 
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intake.move_voltage(-12000);
        if(pros::millis() - timeSinceLastFlash > 500){
            pulseTrackLights();
            right_track.update();
            left_track.update();

            timeSinceLastFlash = pros::millis();
        }
    }
    else{
        intake.move_voltage(0);
    }
}

int flywheelRPMTarget = 1950;

void fwControl(){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        //shift state
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            blooper.set_value(true);
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            blooper.set_value(false);
        }


        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            flywheelRPMTarget = 0;
            flywheel.set_velocity_custom_controller(flywheelRPMTarget); 
            return;
        }
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            
            if(std::abs(flywheel.get_velocity_error()) < 100){
                flywheel.set_voltage(-5000);
            }
            return;
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            flywheelRPMTarget = 3400;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            flywheelRPMTarget = 5000;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            flywheelRPMTarget = 3600;
        }
    }

    else{
        //unshifted state
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            flywheelRPMTarget = 0;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            flywheelRPMTarget = 1950;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            flywheelRPMTarget = 3100;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            flywheelRPMTarget = 2500;
        }
        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            flywheelRPMTarget += 50;
        }
        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            flywheelRPMTarget -= 50;
        }
    }
    
    if(flywheelRPMTarget > -10){
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            flywheel.set_velocity_custom_controller(flywheelRPMTarget*1.5); 
        }
        else{
            flywheel.set_velocity_custom_controller(flywheelRPMTarget); 
        }
    }

    if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        blooper.set_value(true);
    }
    if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        blooper.set_value(false);
    }
}

