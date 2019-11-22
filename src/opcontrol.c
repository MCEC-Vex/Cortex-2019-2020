/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */

#define LOOP_DELAY 20
#define JOYSTICK_MASTER 1

// Define motor ports
#define LEFT_MOTOR_FRONT 5
#define LEFT_MOTOR_BACK 4
#define RIGHT_MOTOR_FRONT 2
#define RIGHT_MOTOR_BACK 3

#define TRAY 6
#define RIGHT_ROLLER 7
#define LEFT_ROLLER 8
#define RIGHT_ARM 9
#define LEFT_ARM 10

#define BACKUP_SPEED 70

// Contributes every 20ms to a max of 4096
#define IDEAL_ARM_INCREMENT 30

// Bounds for the arm (with calibration, the lowered state is 0)
#define ARM_LOWER_BOUND 0
#define ARM_UPPER_BOUND 4000

/**
 * Convenience function to get the sign of an integer while avoiding branches
 * See https://stackoverflow.com/questions/14579920/fast-sign-of-integer-in-c
 *
 * @param x The integer to get the sign of
 * @return The integer sign as -1 or 1
 */
int sign(int x)
{
    return (x > 0) - (x < 0);
}

/**
 * This function sets the motor power for the right and left side of the robot
 * @param left The left motor voltage. -127 to 127
 * @param right The right motor voltage. -127 to 127
 */
void setMotorPower(int left, int right)
{
    motorSet(LEFT_MOTOR_FRONT, left);
    motorSet(LEFT_MOTOR_BACK, left);
    motorSet(RIGHT_MOTOR_FRONT, right * -1); // Multiply right side by -1 because
    motorSet(RIGHT_MOTOR_BACK, right * -1); //  the motors are facing the opposite direction
}

void dropOffCubes()
{
    // Move the tray all the way up
    motorSet(TRAY, 127);
    delay(800);
    motorSet(TRAY, 0);

    delay(2000);

    // Bump the robot forward
    setMotorPower(60, 60);
    delay(100);
    setMotorPower(-60, -60);
    delay(100);
    setMotorPower(0, 0);

    delay(2000);

    // Back up and roll out
    motorSet(RIGHT_ROLLER, 80);
    motorSet(LEFT_ROLLER, -80);
    setMotorPower(BACKUP_SPEED * -1, BACKUP_SPEED * -1);
    delay(700);
    motorSet(RIGHT_ROLLER, 0);
    motorSet(LEFT_ROLLER, 0);
    setMotorPower(0, 0);
}

void operatorControl()
{
    int forwardPower;
    int turningPower;

    // For the tray
    int liftPowerScale = 0;
    int idealLiftPos = 0;

    while(1)
    {
        forwardPower = joystickGetAnalog(JOYSTICK_MASTER, 3);
        turningPower = joystickGetAnalog(JOYSTICK_MASTER, 1);

        // Scale the turning power to be less sensitive
        turningPower /= 1.4;

        // Set min values to avoid controller drift
        if(turningPower < 15 && turningPower > -15)
        {
            turningPower = 0;
        }
        if(forwardPower < 15 && forwardPower > -15)
        {
            forwardPower = 0;
        }

        // Adjust left and right powers
        int leftPower = forwardPower + turningPower;
        int rightPower = forwardPower - turningPower;

        // Set the drive motors
        setMotorPower(leftPower, rightPower);

        // Roller motors (make one side negative so they both spin in the same direction)
        if(joystickGetDigital(JOYSTICK_MASTER, 6, JOY_UP))
        {
            motorSet(RIGHT_ROLLER, -127);
            motorSet(LEFT_ROLLER, 127);
        }
        else if(joystickGetDigital(JOYSTICK_MASTER, 6, JOY_DOWN))
        {
            // Make the rollers go slower when releasing cubes for precision moves
            motorSet(RIGHT_ROLLER, 60);
            motorSet(LEFT_ROLLER, -60);
        }
        else
        {
            motorSet(RIGHT_ROLLER, 0);
            motorSet(LEFT_ROLLER, 0);
        }

        // Tray
        if(joystickGetDigital(JOYSTICK_MASTER, 5, JOY_UP))
        {
            if(liftPowerScale < 60)
            {
                liftPowerScale++;
            }

            motorSet(TRAY, ((127.0 / 88.0) * (89 - liftPowerScale)));
        }
        else if(joystickGetDigital(JOYSTICK_MASTER, 5, JOY_DOWN))
        {
            liftPowerScale = 0;
            motorSet(TRAY, -127);
        }
        else
        {
            liftPowerScale = 0;
            motorSet(TRAY, 0);
        }

        // Intake arm
        //TODO use manual override in case new algorithm fails miserably during competition
        /*if(joystickGetDigital(JOYSTICK_MASTER, 7, JOY_UP))
        {
            motorSet(RIGHT_ARM, -127);
            motorSet(LEFT_ARM, -127);
        }
        else if(joystickGetDigital(JOYSTICK_MASTER, 7, JOY_DOWN))
        {
            motorSet(RIGHT_ARM, 127);
            motorSet(LEFT_ARM, 127);
        }
        else
        {
            motorSet(RIGHT_ARM, 0);
            motorSet(LEFT_ARM, 0);
        }*/

        // New algorithm designed to keep (mostly) constant position
        // It works by using the buttons to adjust ideal position instead of voltage
        // and trying to automatically set the voltage to reach (and stay at) the ideal position
        // TODO set proper bounds based on sensor testing
        if(joystickGetDigital(JOYSTICK_MASTER, 7, JOY_UP))
        {
            if((idealLiftPos + IDEAL_ARM_INCREMENT) > ARM_UPPER_BOUND)
            {
                idealLiftPos = ARM_UPPER_BOUND;
            }
            else
            {
                idealLiftPos += IDEAL_ARM_INCREMENT;
            }
        }
        else if(joystickGetDigital(JOYSTICK_MASTER, 7, JOY_DOWN))
        {
            if((idealLiftPos - IDEAL_ARM_INCREMENT) < ARM_LOWER_BOUND)
            {
                idealLiftPos = ARM_LOWER_BOUND;
            }
            else
            {
                idealLiftPos -= IDEAL_ARM_INCREMENT;
            }
        }

        // Get difference in (angular) position for proportional applied voltage
        int proportional = (idealLiftPos - analogReadCalibrated(ARM_POTENTIOMETER)) * -0.1;
        // Set the arm motors
        motorSet(RIGHT_ARM, proportional);
        motorSet(LEFT_ARM, proportional);

        // Reset on the left key
        if(joystickGetDigital(JOYSTICK_MASTER, 7, JOY_LEFT))
        {
            idealLiftPos = ARM_LOWER_BOUND;
        }

        // Back up and turn rollers out
        if(joystickGetDigital(JOYSTICK_MASTER, 8, JOY_DOWN))
        {
            motorSet(RIGHT_ROLLER, 80);
            motorSet(LEFT_ROLLER, -80);


            motorSet(LEFT_MOTOR_FRONT, BACKUP_SPEED * -1);
            motorSet(LEFT_MOTOR_BACK, BACKUP_SPEED * -1);
            motorSet(RIGHT_MOTOR_FRONT, BACKUP_SPEED);
            motorSet(RIGHT_MOTOR_BACK, BACKUP_SPEED);
        }

        // Drop off cubes macro
        if(joystickGetDigital(JOYSTICK_MASTER, 8, JOY_RIGHT))
        {
            dropOffCubes();
        }

        delay(20);
    }
}
//push test 11.22.19_2
