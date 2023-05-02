#include "main.h"
#include "config.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "sylib/system.hpp"
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include "static.h"


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
int autonRoutine = 0;


double actualCurrentLimit(double temperature){
    double currentLimit;
    if(temperature>=70){
        currentLimit = 0;
    }
    else if(temperature >= 65){
        currentLimit = 312.5;
    }
    else if(temperature >= 60){
        currentLimit = 625;
    }
    else if(temperature >= 55){
        currentLimit = 1250;
    }
    else{
        currentLimit = 2500;
    }
    return currentLimit;
}

// const int CHASSIS_COLOR_START = 0x440044;
// const int CHASSIS_COLOR_END = 0x004444;
const int CHASSIS_COLOR_START = 0xFF00FF;
const int CHASSIS_COLOR_END = 0x00FFFF;

int chass_r = 0;
int chass_g = 0;
int chass_b = 0;
void chassis_solid_update(){
	if(chass_r < 0)
		chass_r = 0;
	if(chass_g < 0)
		chass_g = 0;
	if(chass_r < 0)
		chass_b = 0;
	
	if(chass_r > 255)
		chass_r = 255;
	if(chass_g > 255)
		chass_g = 255;
	if(chass_r > 255)
		chass_b = 255;
	int hex = sylib::Addrled::rgb_to_hex(chass_r % 255, chass_g% 255, chass_b% 255);
	letters_left.set_all(hex);
	letters_right.set_all(hex);
	caracal.set_all(hex);
	letters_back.set_all(hex);

}

void chassis_light_default(){
	// leftSkirt.gradient(CHASSIS_COLOR_START, CHASSIS_COLOR_END, 0, 0, true, true);
	// rightSkirt.gradient(CHASSIS_COLOR_START, CHASSIS_COLOR_END, 0, 0, false, true);
	letters_back.gradient(CHASSIS_COLOR_START, CHASSIS_COLOR_END, 0, 0, true, false);
	caracal.gradient(CHASSIS_COLOR_START, CHASSIS_COLOR_END, 0, 0, false, false);
	// leftSkirt.cycle(*leftSkirt, 15, 0, true);
	// rightSkirt.cycle(*rightSkirt, 15);
	// backSkirt.cycle(*backSkirt, 5);
	caracal.cycle(*caracal, 15);

	letters_left.gradient(CHASSIS_COLOR_START, CHASSIS_COLOR_END, 0, 0, true, false);
	letters_right.gradient(CHASSIS_COLOR_START, CHASSIS_COLOR_END, 0, 0, true, false);
	letters_back.cycle(*letters_back, 15);
	letters_left.cycle(*letters_left, 15);
	letters_right.cycle(*letters_right, 15);


}

const double current_draw_cutoff = 50;
int TRACK_LIGHT_SET_4_START = 0x250040;
int TRACK_LIGHT_SET_4_END = 0x400040;

int TRACK_LIGHT_SET_3_START = 0x420000;
int TRACK_LIGHT_SET_3_END = 0x420f00;

int TRACK_LIGHT_SET_2_START = 0x3e4000;
int TRACK_LIGHT_SET_2_END = 0x3d4200;

int TRACK_LIGHT_SET_1_START = 0x004206;
int TRACK_LIGHT_SET_1_END = 0x00400b;

int TRACK_LIGHT_SET_0_START = 0x003540;
int TRACK_LIGHT_SET_0_END = 0x001b40;


void pulseTrackLights(){
	static sylib::hsv pulseColor = sylib::hsv();
    pulseColor.s = 1;
    pulseColor.v = 1;
	pulseColor.h = std::rand() % 360;
	left_track.pulse(sylib::Addrled::hsv_to_rgb(pulseColor), 2, 30);
	right_track.pulse(sylib::Addrled::hsv_to_rgb(pulseColor), 2, 30);
}
void chassis_light_control(){
	static int leftCurrentDraw;
	static int rightCurrentDraw;

	static int leftCurrentLimit;
	static int rightCurrentLimit;

	static double leftSpeed;
	static double rightSpeed;

	static std::uint32_t shift_amount;

	static double current_draw_speed_ratio;

	leftCurrentDraw = (leftDrive.get_current_draws()[0]+leftDrive.get_current_draws()[1]+leftDrive.get_current_draws()[2])/3;
	rightCurrentDraw = (rightDrive.get_current_draws()[0]+rightDrive.get_current_draws()[1]+rightDrive.get_current_draws()[2])/3;

	leftCurrentLimit = (actualCurrentLimit(l1.get_temperature()) +
					    actualCurrentLimit(l2.get_temperature()) + 
						actualCurrentLimit(l3.get_temperature()))/3;

	rightCurrentLimit = (actualCurrentLimit(r1.get_temperature()) +
					     actualCurrentLimit(r2.get_temperature()) + 
						 actualCurrentLimit(r3.get_temperature()))/3;

	leftSpeed = std::abs((leftDrive.get_actual_velocities()[0] + leftDrive.get_actual_velocities()[1]+leftDrive.get_actual_velocities()[2])/3);
	rightSpeed = std::abs((rightDrive.get_actual_velocities()[0] + rightDrive.get_actual_velocities()[1]+rightDrive.get_actual_velocities()[2])/3);


	current_draw_speed_ratio = std::abs((2500*(double)(leftCurrentDraw+rightCurrentDraw)/(double)(leftCurrentLimit+rightCurrentLimit))/(std::abs(((leftSpeed+rightSpeed)+1))/2));

	shift_amount = (std::uint32_t)(((current_draw_speed_ratio-current_draw_cutoff)));

	if(current_draw_speed_ratio > current_draw_cutoff){
		letters_left.color_shift(shift_amount, -shift_amount, -shift_amount);
		letters_right.color_shift(shift_amount, -shift_amount, -shift_amount);
		letters_back.color_shift(shift_amount, -shift_amount, -shift_amount);
		caracal.color_shift(shift_amount, -shift_amount, -shift_amount);
	}
	else{
		letters_left.color_shift(0, 0, 0);
		letters_right.color_shift(0, 0, 0);
		letters_back.color_shift(0, 0, 0);
		caracal.color_shift(0, 0, 0);
	}

	if(leftCurrentLimit <= 1250){
		letters_left.color_shift(0, 255, 0);
	}
	else{
		letters_left.color_shift(0, 0, 0);
	}

	if(rightCurrentLimit <= 1250){
		letters_right.color_shift(0, 255, 0);
	}
	else{
		letters_right.color_shift(0, 0, 0);
	}
}

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = robor.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}
void setIndicatorLights(int color){
	caracal.set_all(color);
	letters_left.set_all(color);
	letters_right.set_all(color);
	letters_back.set_all(color);
}
void initialize(){
	// [REDACTED]
	pros::Task screenInit(initImages);
	pros::Task odomTask(odomControlLoop);
	// pros::lcd::initialize();
	// pros::Task screenTask(screen); // create a task to print the position to the screen
	sylib::initialize();
	robor.calibrate();
	chassis_light_default();
	blooper.set_value(false);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	setIndicatorLights(0xffd900);
	// leftSkirt.set_all(0xfa8b02);
	// rightSkirt.set_all(0xfa8b02);
	// backSkirt.set_all(0xfa8b02);
	// caracalPlate.set_all(0xfa8b02);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

void competition_initialize() {
	while(pros::competition::is_disabled()){
		if(std::abs((int)flywheel.get_position()) % 300 < 100){
			autonRoutine = 1;
		}
		else if(std::abs((int)flywheel.get_position()) % 300 < 200){
			autonRoutine = 2;
		}
		else{
			autonRoutine = 3;
		}


		if(autonRoutine == 1){
			setIndicatorLights(0xff00ff); // Magenta
		}
		else if(autonRoutine == 2){
			setIndicatorLights(0xffffff); // white
		}
		else if(autonRoutine == 3){
			setIndicatorLights(0x00ff00); // green
		}
		else{
			setIndicatorLights(0xff0000); // green
		}
		sylib::delay(10);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() {
	// [REDACTED]
	setIndicatorLights(0xff00aa);
	if(autonRoutine==1){
		fullAWP();
	}
	else if(autonRoutine == 2){
		closeSideHalf();
	}
	else if(autonRoutine == 3){
		farSide();
	}
	else{
		setIndicatorLights(0xff0000);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

int disks_in_intake = 0;
double distanceSensorReading = 0;
int senseDisks(){
	distanceSensorReading = diskSensor.get();
	if(distanceSensorReading > 75){
		left_track.set_all(0x002b33);
		right_track.set_all(0x002b33);
		return 0;
	}
	else if(distanceSensorReading > 55){
		left_track.set_all(0x053300);
		right_track.set_all(0x053300);
		return 1;
	}
	else if(distanceSensorReading > 30){
		left_track.set_all(0x323300);
		right_track.set_all(0x323300);
		return 2;
	}
	else if(distanceSensorReading > 15){
		left_track.set_all(0x330000);
		right_track.set_all(0x330000);
		return 3;
	}
	else{
		left_track.set_all(0x190033);
		right_track.set_all(0x3190033);
		return 4;
	}
}

void opcontrol() {
	uint32_t control_ticks = 0;
	int flyVel = 0;
	int flyVelTarget = 0;
	int flyVelError = 0;
	bool shiftPressed = false;
	bool booped = false;
	int boopedTime = 0;
	chassis_light_default();
	uint32_t clock = sylib::millis();
	while (true){
		control_ticks++;
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){ // SHIFT KEY
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
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){ // SHIFT KEY
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
				leftDrive.move_velocity(125);
				rightDrive.move_velocity(-125);
			}
			else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
				leftDrive.move_velocity(-125);
				rightDrive.move_velocity(125);
			}
			else{
				drive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
			}

		}
		else{
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
				leftDrive.move_velocity(-125);
				rightDrive.move_velocity(-125);
			}
			else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        		leftDrive.move_velocity(125);
        		rightDrive.move_velocity(125);
			}
			else{
				drive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
			}
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            	zapper.set_value(true);
        	}
			else{
				zapper.set_value(false);
			}
		}
		
		
			
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
			master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && 
			master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && 
			master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){

			string.set_value(true);
		}
		else{
			string.set_value(false);
			intakeCont();
			fwControl();
		}
		if(!partner.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
				chass_r += 10;
			}
			else if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
				chass_g += 10;
			}
			else if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
				chass_b += 10;
			}
		}
		else{
			if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
				chass_r -= 10;
			}
			else if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
				chass_g -= 10;
			}
			else if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
				chass_b -= 10;
			}
		}
		
		chassis_light_control();
		// senseDisks();
		// chassis_solid_update();
		flyVel = (int)flywheel.get_velocity();
		flyVelTarget = (int)flywheel.get_velocity_target();
		flyVelError = flyVelTarget-flyVel;

		if((control_ticks+6) % 12 == 0){
			if(std::abs(flyVelError) > 50 && std::abs(flyVelTarget) < 3900 && sylib::millis() > 2000){
				master.rumble("-");
			}
		}
		else if ((control_ticks) % 12 == 0) {
				master.set_text(0,0,std::to_string(flyVel) + " | " + std::to_string(flyVelTarget)+ " | " + std::to_string(flyVelError) + "    ");
		}
		sylib::delay_until(&clock,10);
	}
}

