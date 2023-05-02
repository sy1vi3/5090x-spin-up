#include "main.h"
#include "config.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "sylib/system.hpp"
#include <cmath>
#include <cstdio>
#include <memory>
#include <ostream>
#include <string>
#include <vector>
/* 
    auton and auton specific functions will go here
    feel free to make additional files if necessary
*/


const double TRACKING_WIDTH = 11.6;
const double WHEEL_DIAMETER = 3.25;
double theta = 0;
double theta_target = 0;
double current_left = 0;
double current_right = 0;
double current_avg = 0;

void moveChassis(double left, double right){
    // printf("%d,%f,%f\n", sylib::millis(), left, right);
    leftDrive.move_voltage(left);
    rightDrive.move_voltage(right);
}

void driveDistance(double target_distance, int timeout, int maxSpeed = 150){
	double start_avg = (current_left+current_right)/2.0;

	current_avg = (current_left+current_right)/2.0 - start_avg;

	double error = target_distance - current_avg;

	double prevTime = sylib::millis();
    int startTime = sylib::millis();

    int cutoffTime = startTime + timeout;


    bool endMovement = false;
	int endMovementTime = 0;

    bool withinErrorBounds = false;
    bool prev_withinErrorBounds = false;

	double prevError = error;
	double integral = 0;
	double power = 0;
	const double kP = 15;
	const double kI = 0;
	const double kD = 200;

	const double kP2 = 0.05;

	double theta_error = 0;

    double turnCorrection = 0;
    double left_voltage = 0;
    double right_voltage = 0;

    int tick = 0;

	while(!endMovement && sylib::millis() < cutoffTime){
        tick++;
		theta_error = std::fmod(theta_target - theta, 360);
		current_avg = ((current_left+current_right)/2.0 - start_avg) * WHEEL_DIAMETER * M_PI/86450 * 87.4 * 2;
		error = target_distance - current_avg;

		power = error * kP;
        if(tick%20 == 0){
        printf("%f\n", current_avg);

        }
		power += ((error-prevError)/(sylib::millis()-prevTime)) * kD;
		integral += (error * (sylib::millis() - prevTime));
        if(std::abs(error) < 15){
            power += integral * kI;
        }
		if(prevError/error < 0){
            integral = 0;
        }

		prevError = error;
		prevTime = sylib::millis();

		if(power > maxSpeed){
			power = maxSpeed;
		}
		else if(power < -maxSpeed){
			power = -maxSpeed;
		}
        
        turnCorrection = theta_error*kP2*power;

        left_voltage = (-power-turnCorrection)*55;
        right_voltage = (-power+turnCorrection)*55;
        if(left_voltage > 100){
            left_voltage += 900;
        }
        else if(left_voltage < 100){
            left_voltage -= 900;
        }

        if(right_voltage > 100){
            right_voltage += 900;
        }
        else if(right_voltage < 100){
            right_voltage -= 900;
        }
        
        moveChassis(left_voltage,right_voltage);

        

        if(std::abs(error) < 0.5){
            withinErrorBounds = true;
        }
        else{
            withinErrorBounds = false;
        }

        if(withinErrorBounds != prev_withinErrorBounds && withinErrorBounds){
            endMovementTime = sylib::millis();
        }

        if(withinErrorBounds && sylib::millis() > endMovementTime + 500){
            endMovement = true;
        }

        prev_withinErrorBounds = withinErrorBounds;

		sylib::delay(10);

	}
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
	
}

void turnToAngle(double angle, int timeout){
	theta_target = angle;
	double error = std::fmod(theta_target - gps.get_heading(), 360);
	double prevError = error;
	double integral = 0;
	double power = 0;
	const double kP = 8;
	const double kI = 0;
	const double kD = 200;
	double prevTime = sylib::millis();
	bool endMovement = false;
	int endMovementTime = 0;

    int startTime = sylib::millis();

    int cutoffTime = startTime + timeout;

    bool withinErrorBounds = false;
    bool prev_withinErrorBounds = false;

    double left_voltage = 0;
    double right_voltage = 0;

	while(!endMovement && sylib::millis() < cutoffTime){
		error = std::fmod(theta_target - theta, 360);
		power = error * kP;
		power += ((error-prevError)/(sylib::millis()-prevTime)) * kD;
        if(std::abs(error) < 5){
            integral += (error * (sylib::millis() - prevTime));
        }
        if(prevError/error < 0){
            integral = 0;
        }
		power += integral * kI;

		prevError = error;
		prevTime = sylib::millis();

		if(power > 125){
			power = 125;
		}
		else if(power < -125){
			power = -125;
		}

        left_voltage = (-power)*45;
        right_voltage = (power)*45;

        if(left_voltage > 100){
            left_voltage += 1500;
        }
        else if(left_voltage < 100){
            left_voltage -= 1500;
        }

        if(right_voltage > 100){
            right_voltage += 1500;
        }
        else if(right_voltage < 100){
            right_voltage -= 1500;
        }
        
        moveChassis(left_voltage,right_voltage);


        if(std::abs(error) < .5){
            withinErrorBounds = true;
        }
        else{
            withinErrorBounds = false;
        }

        if(withinErrorBounds != prev_withinErrorBounds && withinErrorBounds){
            endMovementTime = sylib::millis();
        }

        if(withinErrorBounds && sylib::millis() > endMovementTime + 500){
            endMovement = true;
        }

        prev_withinErrorBounds = withinErrorBounds;
		sylib::delay(10);
	}
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
}

void odomControlLoop(void * param){
	static int ticks = 0;
    int flyVel = 0;
	int flyVelTarget = 0;
	int flyVelError = 0;
    leftDrive.tare_position();
    rightDrive.tare_position();
    std::vector<double> leftDrivePos = leftDrive.get_positions();
    std::vector<double> rightDrivePos = rightDrive.get_positions();
	double start_left = -(leftDrivePos[0] + leftDrivePos[1] + leftDrivePos[2])/3;
    double start_right = -(rightDrivePos[0] + rightDrivePos[1] + rightDrivePos[2])/3;
	while(1){ //pros::competition::is_autonomous())
		ticks++;
        leftDrivePos = leftDrive.get_positions();
        rightDrivePos = rightDrive.get_positions();
		current_left = -(leftDrivePos[0] + leftDrivePos[1] + leftDrivePos[2])/3;
		current_right = -(rightDrivePos[0] + rightDrivePos[1] + rightDrivePos[2])/3;
		
		theta = (WHEEL_DIAMETER*((current_left-start_left)*202.8 - (current_right-start_right)*202.8)/(TRACKING_WIDTH))*(180.0/86450.0);
        if(ticks % 20 == 0){
            printf("theta: %f | left: %f | right: %f\n", theta, current_left, current_left);
        }
		sylib::delay(10);
	}
}


void driveToPoint(double x, double y, double theta){
    
    double kPturn = 1;
    double kPdrive = 1;
    double kD = 0;
    double kI = 0;
    
    double current_x = 0;
    double current_y = 0;
    gps.get_offset(&current_x, &current_y);
    double current_theta = gps.get_heading();

    double x_offset = x - current_x;
    double y_offset = y - current_y;
    double abs_offset = std::sqrt(x_offset*x_offset + y_offset*y_offset);
    double theta_error = current_theta - std::atan2(y_offset, x_offset);
    
    // sylib::DerivativeController driveToPointDeriv(kD, std::shared_ptr<double>(&abs_offset));
    // sylib::DerivativeController turnToPointDeriv(kD, std::shared_ptr<double>(&theta_error));

    double power = 0;
    double left_voltage = 0;
    double right_voltage = 0;

    while(abs_offset > 1){
        gps.get_offset(&current_x, &current_y);
        double current_theta = gps.get_heading();
        x_offset = x - current_x;
        y_offset = y - current_y;
        abs_offset = std::sqrt(x_offset*x_offset + y_offset*y_offset);
        theta_error = current_theta - std::atan2(y_offset, x_offset);
        power = kPturn * theta_error;
        if(power > 125){
            power = 125;
        }
        else if(power < -125){
            power = -125;
        }
        left_voltage = (-power)*45;
        right_voltage = (power)*45;

        if(left_voltage > 100){
            left_voltage += 1500;
        }
        else if(left_voltage < 100){
            left_voltage -= 1500;
        }

        if(right_voltage > 100){
            right_voltage += 1500;
        }
        else if(right_voltage < 100){
            right_voltage -= 1500;
        }
        if(theta_error < 15){
            right_voltage += kPdrive * abs_offset;
            left_voltage += kPdrive * abs_offset; 
        }
        moveChassis(left_voltage,right_voltage);
        sylib::delay(10);
    }
    moveChassis(0,0);
    theta_error = theta - current_theta;
    while(theta_error > 1){
        gps.get_offset(&current_x, &current_y);
        double current_theta = gps.get_heading();
        theta_error = theta - current_theta;
        power = kPturn * theta_error;
        if(power > 125){
            power = 125;
        }
        else if(power < -125){
            power = -125;
        }
        left_voltage = (-power)*45;
        right_voltage = (power)*45;

        if(left_voltage > 100){
            left_voltage += 1500;
        }
        else if(left_voltage < 100){
            left_voltage -= 1500;
        }

        if(right_voltage > 100){
            right_voltage += 1500;
        }
        else if(right_voltage < 100){
            right_voltage -= 1500;
        }
        moveChassis(left_voltage,right_voltage);
        sylib::delay(10);
    }
    moveChassis(0,0);

}

bool boostPullback = false;
void pullbackSling(void * param){

}

void fireSlingAuto(bool pullbackExtra = false){

}

void intakeUnder3(){
    // if(diskSensor.get() >= 190){
        intake.move_voltage(-12000);
    // }
    // else{
        // intake.move_voltage(0);
    // }
}

void simpleRollerAutoFar(){

}

void simpleRollerAutoClose(){

}

void shootOne(){
    intake.move_voltage(-12000);
    sylib::delay(100);
    intake.move_voltage(0);
    sylib::delay(25);
    intake.move_voltage(12000);
    sylib::delay(100);
    intake.move_voltage(0);
}
void roller(){
    moveChassis(-2000, -2000);
    intake.move_voltage(8000);
    sylib::delay(400);
    intake.move_voltage(0);
    moveChassis(0, 0);
}

void fullAWP(){
    robor.setPose(-60,36,90);
    flywheel.set_velocity_custom_controller(3300);
    zapper.set_value(true);
    roller();
    moveChassis(7000, 0);
    sylib::delay(610);
    moveChassis(0,0);
    robor.turnTo(53, 59, 1000);
    sylib::delay(75);
    shootOne();
    sylib::delay(300);
    shootOne();
    flywheel.set_velocity_custom_controller(3300);
    robor.turnTo(-43, 29.25, 1000);
    zapper.set_value(true);
    intake.move_voltage(12000);
    robor.moveTo(-43, 29.25, 1000, 75);
    zapper.set_value(false);
    flywheel.set_velocity_custom_controller(3200);
    sylib::delay(1000);
    robor.turnTo(53, 58.5, 1000);
    sylib::delay(600);
    shootOne();
    // intake.move_voltage(12000);
    sylib::delay(300);
    shootOne();
    // intake.move_voltage(12000);
    sylib::delay(300);
    shootOne();
    intake.move_voltage(12000);
    flywheel.set_velocity_custom_controller(3350);
    robor.turnTo(30, -39, 1000);
    if(senseDisks() > 0){
        shootOne();
    }
    intake.move_voltage(12000);
    
    robor.moveTo(30, -36, 2000,100);
    robor.turnTo(43, 55, 1000);
    sylib::delay(50);
    shootOne();
    sylib::delay(300);
    shootOne();
    sylib::delay(300);
    shootOne();
    // robor.turnTo(-30, 0, 500);
    robor.moveTo(35, -40, 700);
    robor.turnTo(35, 0, 700);
    moveChassis(-6000, -6000);
    sylib::delay(50);
    moveChassis(0, 0);
    roller();
    intake.move_voltage(8000);

    sylib::delay(50);
    intake.move_voltage(0);

    
}

void matchLoads(){

}

void closeSideHalf(){
   robor.setPose(-60,36,90);
    flywheel.set_velocity_custom_controller(3300);
    zapper.set_value(true);
    roller();
    moveChassis(7000, 0);
    sylib::delay(610);
    moveChassis(0,0);
    robor.turnTo(53, 59, 1000);
    sylib::delay(75);
    shootOne();
    sylib::delay(300);
    shootOne();
    flywheel.set_velocity_custom_controller(3200);

    robor.turnTo(-45, 40, 600);
    robor.moveTo(-45, 40, 1000);

    intake.move_voltage(12000);
    zapper.set_value(false);
    sylib::delay(1500);
    robor.moveTo(-46, 33, 1000);
    robor.turnTo(53, 59, 1000);
    sylib::delay(100);
    shootOne();
    sylib::delay(500);
    shootOne();
    sylib::delay(500);
    shootOne();
    zapper.set_value(true);

    robor.turnTo(-40, 30, 1000);
    flywheel.set_velocity_custom_controller(3150);
    intake.move_voltage(-12000);
    sylib::delay(300);
    intake.move_voltage(12000);
    robor.moveTo(-40, 30, 1000, 75);
    zapper.set_value(false);
    sylib::delay(750);
    robor.turnTo(53, 58, 1000);
    sylib::delay(500);
    shootOne();
    sylib::delay(500);
    shootOne();
    sylib::delay(500);
    shootOne();
}

void rightSideSaved(){
    robor.setPose(12,60, 180);
    flywheel.set_velocity_custom_controller(3100);
    intake.move_voltage(12000);
    robor.moveTo(12, 12, 2000, 150);
    robor.turnTo(-56, -60, 1000);
    intake.move_voltage(0);
    sylib::delay(100);
    shootOne();
    sylib::delay(150);
    shootOne();
    sylib::delay(150);
    shootOne();
    robor.turnTo(-10, 28, 800);
    flywheel.set_velocity_custom_controller(3300);

    intake.move_voltage(12000);
    robor.moveTo(-10, 28, 1000);
    robor.turnTo(-17, 20, 1000);
    robor.moveTo(-17, 20, 1000);
    robor.moveTo(-17, 36, 2000);
    robor.turnTo(-52, -60, 1000);
    sylib::delay(25);
    flywheel.set_velocity_custom_controller(3600);

    shootOne();
    sylib::delay(75);
    flywheel.set_velocity_custom_controller(3800);

    shootOne();
    sylib::delay(125);
    flywheel.set_velocity_custom_controller(3300); 

    shootOne();
    zapper.set_value(true);
    intake.move_voltage(12000);
    flywheel.set_velocity_custom_controller(3400);

    robor.turnTo(-25, 30, 1000);
    robor.moveTo(-25, 30, 1000);
    zapper.set_value(false);
    sylib::delay(750);
    robor.turnTo(-55, -60, 1000);
    sylib::delay(250);

    shootOne();
    flywheel.set_velocity_custom_controller(3600);

    sylib::delay(75);
    shootOne();
    sylib::delay(75);
    shootOne();
    robor.moveTo(-30, 54, 1000);
    robor.turnTo(-30, 0, 200);
    roller();
}

void farSide(){
    robor.setPose(16,-54,0);
    flywheel.set_velocity_custom_controller(3300);
    zapper.set_value(true);
    robor.moveTo(24.5, -41.5, 1000);

    robor.turnTo(58, 53, 500); // get to shooting position for first two


    sylib::delay(300);
    shootOne();
    sylib::delay(300);
    shootOne();   // shoot first 2

    robor.turnTo(26.5, -38.5, 1000);
    robor.moveTo(26.5, -38.5, 1000);
    flywheel.set_velocity_custom_controller(3350); //speed for 2nd volley

    intake.move_voltage(12000);
    // robor.moveTo(36, -36, 750);
    zapper.set_value(false);
    sylib::delay(700); //yoink 3 stack, don't shoot


    robor.moveTo(29, -59, 2000); // edit timout
    intake.move_voltage(0);
    sylib::delay(75);
    intake.move_voltage(12000);
    sylib::delay(600);
    intake.move_voltage(0);



    robor.turnTo(32, 0, 2000); // edit timout
    moveChassis(-4000, -4000);
    sylib::delay(200);
    moveChassis(0, 0);
    roller();
    intake.move_voltage(12000);
    sylib::delay(100);
    robor.moveTo(20, -48, 1000);
    robor.turnTo(50, 53, 1000); //line up for 3 shot


    sylib::delay(150);
    shootOne();
    sylib::delay(300);
    shootOne();
    sylib::delay(300);
    shootOne();  // shoot 3


    robor.turnTo(-24, -4, 750);
    intake.move_voltage(12000);
    flywheel.set_velocity_custom_controller(3100); //speed for 3rd volley
    robor.moveTo(-24, -4, 2000, 100);
    robor.turnTo(51, 55, 1000); // intake line of 3


    sylib::delay(200);
    shootOne();
    sylib::delay(300);
    shootOne();
    sylib::delay(300);
    shootOne();  //shoot 3

    //maybe go for one more on the line or barrier disks depending on time
}

void gpsSet(){
    double x=0;
    double y=0;
    double angle;
    angle = (gps.get_heading());
    // x = gps.get_status().x;
    // y = gps.get_status().y;
    printf("%f\n", robor.getPose().theta);

    robor.setPose(robor.getPose().x,robor.getPose().y,angle);
    printf("%f\n", robor.getPose().theta);

}


void skillsAuto(){

}