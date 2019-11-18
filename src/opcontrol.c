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

// Contributes every 20ms to a max of 4096
#define IDEAL_ARM_INCREMENT 40

// Bounds for the arm (with calibration, the lowered state is 0)
#define ARM_LOWER_BOUND 0
#define ARM_UPPER_BOUND 3000

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

void operatorControl()
{
    int forwardPower;
    int turningPower;
    int analogOut;

    // For the tray
    int liftPowerScale = 0;

    float liftPower = 0;
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
        motorSet(LEFT_MOTOR_FRONT, leftPower);
        motorSet(LEFT_MOTOR_BACK, leftPower);
        motorSet(RIGHT_MOTOR_FRONT, rightPower * -1); // Multiply right side by -1 because
        motorSet(RIGHT_MOTOR_BACK, rightPower * -1); //  the motors are facing the opposite direction

        // Roller motors (make one side negative so they both spin in the same direction)
        if(joystickGetDigital(JOYSTICK_MASTER, 6, JOY_UP))
        {
            motorSet(RIGHT_ROLLER, -127);
            motorSet(LEFT_ROLLER, 127);
        }
        else if(joystickGetDigital(JOYSTICK_MASTER, 6, JOY_DOWN))
        {
            // Make the rollers go slower when releasing cubes for precision moves
            motorSet(RIGHT_ROLLER, 80);
            motorSet(LEFT_ROLLER, -80);
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
        /*
        if(joystickGetDigital(JOYSTICK_MASTER, 7, JOY_UP))
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
            if(idealLiftPos + IDEAL_ARM_INCREMENT > ARM_UPPER_BOUND)
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
            if(idealLiftPos - IDEAL_ARM_INCREMENT < ARM_LOWER_BOUND)
            {
                idealLiftPos = ARM_LOWER_BOUND;
            }
            else
            {
                idealLiftPos -= IDEAL_ARM_INCREMENT;
            }
        }

        // Get difference in (angular) position
        // Note: this can be negative (-4096 to 4096 theoretically for full 250 degree arm motion)
        int currentPos = analogReadCalibrated(ARM_POTENTIOMETER);
        int positionDiff = idealLiftPos - currentPos;
        int absolutePositionDiff = abs(positionDiff);

        // Change power based on distance to travel
        // These will likely need to be fine-tuned for real-world performance
        // TODO evaluate effectiveness of this approach vs calculating speed of travel or a combination of speed and distance
        if(currentPos < ARM_LOWER_BOUND + (IDEAL_ARM_INCREMENT * 3))
        {
            // Don't slam the arm into the starting pos in case the sensor can't reach a perfect 0
            liftPower = 0;
        }
        else
        {
            if(absolutePositionDiff < 500)
            {
                // Adds 0.1 or subtracts 0.1
                liftPower += sign(positionDiff) * 0.1;
            }
            else if(absolutePositionDiff < 1500)
            {
                // Adds 2 or subtracts 2
                liftPower += sign(positionDiff) * 1;
            }
            else if(absolutePositionDiff < 2500)
            {
                liftPower += sign(positionDiff) * 3;
            }
            else
            {
                liftPower += sign(positionDiff) * 5;
            }
        }

        // Ensure repetitive adding during travel doesn't create *too* much "voltage momentum"
        if(liftPower > 127)
        {
            liftPower = 127;
        }
        else if(liftPower < -127)
        {
            liftPower = -127;
        }

        // Set the arm motors
        motorSet(RIGHT_ARM, liftPower);
        motorSet(LEFT_ARM, liftPower);

        delay(20);
    }
}
