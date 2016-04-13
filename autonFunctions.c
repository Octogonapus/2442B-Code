#ifndef AUTONOMOUSFUNCTIONS_H_INCLUDED
#define AUTONOMOUSFUNCTIONS_H_INCLUDED

#include "Bulldog_Core_Includes.h"

/***************************************************************************/
/*                                                                         */
/* Macros - Easy control of motors                                         */
/*                                                                         */
/***************************************************************************/
//START AUTO GENERATED MACROS
#define setLeftDriveMotors(power) setMotorSpeed(leftDriveFront, power); setMotorSpeed(leftDriveBack, power)
#define setLeftDriveMotorsRaw(power) motor[leftDriveFront] = power; motor[leftDriveBack] = power
#define setRightDriveMotors(power) setMotorSpeed(rightDriveFront, power); setMotorSpeed(rightDriveBack, power)
#define setRightDriveMotorsRaw(power) motor[rightDriveFront] = power; motor[rightDriveBack] = power
#define setDriveMotors(power) setMotorSpeed(leftDriveFront, power); setMotorSpeed(leftDriveBack, power); setMotorSpeed(rightDriveFront, power); setMotorSpeed(rightDriveBack, power)
#define setDriveMotorsRaw(power) motor[rightDriveFront] = power; motor[rightDriveBack] = power
#define setIntakeMotors(power) setMotorSpeed(rightDriveFront, power); setMotorSpeed(rightDriveBack, power)
#define setIntakeMotorsRaw(power) motor[rightDriveFront] = power; motor[rightDriveBack] = power
//END AUTO GENERATED MACROS

#define setLeftLauncherMotors(power) setMotorSpeed(leftOuter, power); setMotorSpeed(leftInner, power)
#define setRightLauncherMotors(power) setMotorSpeed(rightInner, power); setMotorSpeed(rightOuter, power)
#define setLauncherMotors(power) setMotorSpeed(leftOuter, power); setMotorSpeed(leftInner, power); setMotorSpeed(rightInner, power); setMotorSpeed(rightOuter, power)

//Sensor redo
#define _sensorResetTypeTo(sensor, type) SensorType[sensor] = sensorNone; SensorType[sensor] = type

/***************************************************************************/
/*                                                                         */
/* Subroutine - Initializes and clears sensors                             */
/*                                                                         */
/***************************************************************************/
void initializeSensors()
{
	clearDebugStream();

	_sensorResetTypeTo(leftDriveQuad, sensorQuadEncoder);
	SensorValue[leftDriveQuad] = 0;
	_sensorResetTypeTo(rightDriveQuad, sensorQuadEncoder);
	SensorValue[rightDriveQuad] = 0;
	_sensorResetTypeTo(leftLauncherQuad, sensorQuadEncoder);
	SensorValue[leftLauncherQuad] = 0;
	_sensorResetTypeTo(rightLauncherQuad, sensorQuadEncoder);
	SensorValue[rightLauncherQuad] = 0;

	//writeDebugStreamLine("calibrating gyro");
	SensorType[gyro] = sensorNone;
	wait1Msec(500);
	SensorType[gyro] = sensorGyro;
	wait1Msec(1100);
	//writeDebugStreamLine("done calibrating gyro");
}

/***************************************************************************/
/*                                                                         */
/* Subroutine - Uses the LCD to select an autonomous                       */
/*                                                                         */
/***************************************************************************/
int selectAutonomous()
{
	string currentAuton, specifier;

	int autonColor = 1, autonTile = 1, autonLevel = 1;
	string autonColorString, autonTileString, autonLevelString;

	sprintf(specifier, "Clr, Tl, Lvl");
	sprintf(autonColorString, "Red");
	sprintf(autonTileString, "Left");
	sprintf(autonLevelString, "Pri");

	timer t;
	timer_Initialize(&t);

	while (true)
	{
		//Left button changes auton color
		if (nLCDButtons & kButtonLeft)
		{
			autonColor = autonColor == 1 ? 2 : 1;
			autonColorString = autonColor == 1 ? "Red" : "Blue";
			waitForLCDRelease();
		}

		//Center button changes auton tile or exits
		if (nLCDButtons & kButtonCenter)
		{
			autonTile = autonTile == 1 ? 2 : 1;
			autonTileString = autonTile == 1 ? "Left" : "Right";
			waitForLCDRelease();
		}

		//Right button changes auton level
		if (nLCDButtons & kButtonRight)
		{
			//Exit when holding right button
			timer_PlaceHardMarker(&t);
			while (nLCDButtons & kButtonRight)
			{
				if (timer_GetDTFromHardMarker(&t) >= 250)
				{
					return (autonColor * 100) + (autonTile * 10) + autonLevel;
				}

				wait1Msec(5);
			}
			timer_ClearHardMarker(&t);

			autonLevel = autonLevel + 1 > 3 ? 1 : autonLevel + 1;

			if (autonLevel == 1)
			{
				autonLevelString = "Pri";
			}
			else if (autonLevel == 2)
			{
				autonLevelString = "Sec";
			}
			else
			{
				autonLevelString = "Ter";
			}

			waitForLCDRelease();
		}

		sprintf(currentAuton, "%s,%s,%s", autonColorString, autonTileString, autonLevelString);

		displayLCDCenteredString(0, currentAuton);
		displayLCDCenteredString(1, specifier);

		wait1Msec(25);
	}
}

/***************************************************************************/
/*                                                                         */
/* Subroutine - Drives for time in milliseconds                            */
/*                                                                         */
/***************************************************************************/
byte driveTime(const int leftPower, const int rightPower, const int timeMs)
{
	int startingTime = time1[T1];             //Record function start

	setLeftDriveMotorsRaw(leftPower);         //Set left side to its power
	setRightDriveMotorsRaw(rightPower);       //Set right side to its power

	while (startingTime + timeMs > time1[T1]) //Wait for timeMs
	{}

	setDriveMotorsRaw(0);                     //Stop

	return 0;
}

#ifdef USING_QUADS

/***************************************************************************/
/*                                                                         */
/* Subroutine - Drives for distance in units (default: inches)             */
/*                                                                         */
/***************************************************************************/
byte driveQuad(const int power, const int ticks)
{
	//Clear encoders
	SensorValue[leftDriveQuad] = 0;
	SensorValue[rightDriveQuad] = 0;

	//Loop timeout
	const int timeout = 1000;

	//Timer for timeout
	timer t;
	timer_Initialize(&t);

	//Position PID controllers for driving
	pos_PID leftPID, rightPID;
	pos_PID_InitController(&leftPID, leftDriveQuad, 0.5, 0, 0.04);
	pos_PID_InitController(&rightPID, rightDriveQuad, 0.5, 0, 0.04);

	//Set controllers' target position
	pos_PID_SetTargetPosition(&leftPID, ticks);
	pos_PID_SetTargetPosition(&rightPID, ticks);

	//Marker for timeout
	timer_PlaceMarker(&t);

	//Drive to target
	do
	{
		//Step controllers
		pos_PID_StepController(&leftPID);
		pos_PID_StepController(&rightPID);

		//Set drive to controllers' outputs
		setLeftDriveMotorsRaw(pos_PID_GetOutput(&leftPID));
		setRightDriveMotorsRaw(pos_PID_GetOutput(&rightPID));

		//writeDebugStreamLine("%d, %d", pos_PID_GetError(&leftPID), pos_PID_GetError(&rightPID));

		//Exit if taking too long
		if (timer_GetDTFromMarker(&t) > timeout)
		{
			setDriveMotorsRaw(0);
			return 1;
		}

		//Exit if both controller's outputs have dropped too low to move the robot
		if (abs(pos_PID_GetOutput(&leftPID)) <= 10 && abs(pos_PID_GetOutput(&rightPID)) <= 10)
		{
			setDriveMotorsRaw(0);
			return 1;
		}

		wait1Msec(10);
	}
	while (abs(pos_PID_GetError(&leftPID)) > 0 || abs(pos_PID_GetError(&rightPID)) > 0);

	// //Difference between sides
	// int rDiff;

	// //10% of power in the direction of rDiff
	// int rMod;

	// //Full power for 90% of ticks
	// while (abs(SensorValue[leftDriveQuad]) < abs(ticks) * 0.9)
	// {
	// 	//Difference between left and right sides
	// 	rDiff = abs(SensorValue[leftDriveQuad]) - abs(SensorValue[rightDriveQuad]);

	// 	//10% of power in the direction of rDiff, determined by which side is lagging
	// 	rMod = sgn(rDiff) * power * 0.1;

	// 	//Directly contorl left side and have right side follow
	// 	setLeftDriveMotorsRaw(power);
	// 	setRightDriveMotorsRaw(power + rMod);

	// 	wait1Msec(1);
	// }

	// //1/3 power for last 10% of ticks
	// while (abs(SensorValue[leftDriveQuad]) < abs(ticks) - 10)
	// {
	// 	//Difference between sides
	// 	rDiff = abs(SensorValue[leftDriveQuad]) - abs(SensorValue[rightDriveQuad]);

	// 	//10% of power in the direction of rDiff, determined by which side is lagging
	// 	rMod = sgn(rDiff) * power * 0.1;

	// 	//Directly control left side and have right side follow
	// 	setLeftDriveMotorsRaw(power / 3);
	// 	setRightDriveMotorsRaw((power / 3) + rMod);

	// 	wait1Msec(1);
	// }

	// driveTime(-1 * (power / 2), -1 * (power / 2), 50);    //Brake at -50% power for a short time to eliminate momentum
	setDriveMotorsRaw(0);                                 //Stop

	return 0;
}

/***************************************************************************/
/*                                                                         */
/* Subroutine - Drives for distance in units (default: inches) using PID   */
/*                                                                         */
/***************************************************************************/
byte driveQuad(const int ticks)
{
	//Clear encoders
	SensorValue[leftDriveQuad] = 0;
	SensorValue[rightDriveQuad] = 0;

	//Loop timeout
	const int timeout = 1000;

	//Timer for timeout
	timer t;
	timer_Initialize(&t);

	//Position PID controllers for driving
	pos_PID leftPID, rightPID;
	pos_PID_InitController(&leftPID, leftDriveQuad, 0.5, 0, 0.04);
	pos_PID_InitController(&rightPID, rightDriveQuad, 0.5, 0, 0.04);

	//Set controllers' target position
	pos_PID_SetTargetPosition(&leftPID, ticks);
	pos_PID_SetTargetPosition(&rightPID, ticks);

	//Marker for timeout
	timer_PlaceMarker(&t);

	//Drive to target
	do
	{
		//Step controllers
		pos_PID_StepController(&leftPID);
		pos_PID_StepController(&rightPID);

		//Set drive to controllers' outputs
		setLeftDriveMotorsRaw(pos_PID_GetOutput(&leftPID));
		setRightDriveMotorsRaw(pos_PID_GetOutput(&rightPID));

		//writeDebugStreamLine("%d, %d", pos_PID_GetError(&leftPID), pos_PID_GetError(&rightPID));

		//Exit if taking too long
		if (timer_GetDTFromMarker(&t) > timeout)
		{
			setDriveMotorsRaw(0);
			return 1;
		}

		//Exit if both controller's outputs have dropped too low to move the robot
		if (abs(pos_PID_GetOutput(&leftPID)) <= 10 && abs(pos_PID_GetOutput(&rightPID)) <= 10)
		{
			setDriveMotorsRaw(0);
			return 1;
		}

		wait1Msec(10);
	}
	while (abs(pos_PID_GetError(&leftPID)) > 0 || abs(pos_PID_GetError(&rightPID)) > 0);

	// //Difference between sides
	// int rDiff;

	// //10% of power in the direction of rDiff
	// int rMod;

	// //Full power for 90% of ticks
	// while (abs(SensorValue[leftDriveQuad]) < abs(ticks) * 0.9)
	// {
	// 	//Difference between left and right sides
	// 	rDiff = abs(SensorValue[leftDriveQuad]) - abs(SensorValue[rightDriveQuad]);

	// 	//10% of power in the direction of rDiff, determined by which side is lagging
	// 	rMod = sgn(rDiff) * power * 0.1;

	// 	//Directly contorl left side and have right side follow
	// 	setLeftDriveMotorsRaw(power);
	// 	setRightDriveMotorsRaw(power + rMod);

	// 	wait1Msec(1);
	// }

	// //1/3 power for last 10% of ticks
	// while (abs(SensorValue[leftDriveQuad]) < abs(ticks) - 10)
	// {
	// 	//Difference between sides
	// 	rDiff = abs(SensorValue[leftDriveQuad]) - abs(SensorValue[rightDriveQuad]);

	// 	//10% of power in the direction of rDiff, determined by which side is lagging
	// 	rMod = sgn(rDiff) * power * 0.1;

	// 	//Directly control left side and have right side follow
	// 	setLeftDriveMotorsRaw(power / 3);
	// 	setRightDriveMotorsRaw((power / 3) + rMod);

	// 	wait1Msec(1);
	// }

	// driveTime(-1 * (power / 2), -1 * (power / 2), 50);    //Brake at -50% power for a short time to eliminate momentum
	setDriveMotorsRaw(0);                                 //Stop

	return 0;
}

#endif //USING_QUADS

#ifdef USING_IMES

/***************************************************************************/
/*                                                                         */
/* Subroutine - Drives for distance in units (default: inches)             */
/*                                                                         */
/***************************************************************************/
byte driveIME(const int power, const int ticks, const float percentage = 0.1)
{
	nMotorEncoder[leftDriveFront] = 0;                          //Clear left IME
	nMotorEncoder[rightDriveFront] = 0;                         //Clear right IME

	int rDiff;                                                  //Difference between sides
	int rMod;                                                   //10% of power in the direction of rDiff

	//Full power for 60% of ticks
	while (abs(nMotorEncoder[leftDriveFront]) < abs(ticks) * 0.6)
	{
		rDiff = abs(nMotorEncoder[leftDriveFront]) - abs(nMotorEncoder[rightDriveFront]);    //Difference between sides
		rMod = sgn(rDiff) * power * percentage;                                              //10% of power in the direction of rDiff
		setLeftDriveMotorsRaw(power);                                                        //Directly control left side
		setRightDriveMotorsRaw(power + rMod);                                                //Have right side adjust to keep in tune with left side
	}

	//1/3 power for last 40% of ticks
	while (abs(nMotorEncoder[leftDriveFront]) < abs(ticks))
	{
		rDiff = abs(nMotorEncoder[leftDriveFront]) - abs(nMotorEncoder[rightDriveFront]);    //Difference between sides
		rMod = sgn(rDiff) * power * percentage;                                              //10% of power in the direction of rDiff
		setLeftDriveMotorsRaw(power / 3);                                                    //Directly control left side
		setRightDriveMotorsRaw((power / 3) + rMod);                                          //Have right side adjust to keep in tune with left side
	}

	driveTime(-1 * (power / 2), -1 * (power / 2), 50);    //Brake at -50% power for a short time to eliminate momentum
	setAllDriveMotorsRaw(0);                              //Stop

	return 0;
}

/***************************************************************************/
/*                                                                         */
/* Subroutine - Drives for distance in units (default: inches) using PID   */
/*                                                                         */
/***************************************************************************/
byte driveIME_PID(const int ticks)
{
	nMotorEncoder[leftDriveFront] = 0;                          //Clear left IME
	nMotorEncoder[rightDriveFront] = 0;                         //Clear right IME

	//Difference between sides
	float rDiff = 0;

	//Loop timeout
	const int timeout = 1000;

	//Timer for timeout
	timer t;
	timer_Initialize(&t);

	//Position PID controllers for driving
	pos_PID leftPID, rightPID;
	pos_PID_InitController(&leftPID, leftDriveFront, 0.5, 0, 0.04);
	pos_PID_InitController(&rightPID, &rDiff, 0.5, 0, 0.04);

	//Marker for timeout
	timer_PlaceMarker(&t);

	//Drive to target
	do
	{
		//Update rDiff
		rDiff - nMotorEncoder[leftDriveFront] - nMotorEncoder[rightDriveFront];

		//Step controllers
		pos_PID_StepController(&leftPID);
		pos_PID_StepController(&rightPID);

		//Set drive to controllers' outputs
		setLeftDriveMotorsRaw(pos_PID_GetOutput(&leftPID));
		setRightDriveMotorsRaw(pos_PID_GetOutput(&leftPID) - pos_PID_GetOutput(&rightPID));

		//writeDebugStreamLine("%d, %d", pos_PID_GetError(&leftPID), pos_PID_GetError(&rightPID));

		//Exit if taking too long
		if (timer_GetDTFromMarker(&t) > timeout)
		{
			setDriveMotorsRaw(0);
			return 1;
		}

		//Exit if controller's output has dropped too low to move the robot
		if (abs(pos_PID_GetOutput(&leftPID)) <= 10)
		{
			setDriveMotorsRaw(0);
			return 1;
		}

		wait1Msec(10);

	} while (abs(pos_PID_GetError(&leftPID)) > 0);

	setAllDriveMotorsRaw(0);                              //Stop

	return 0;
}

#endif //USING_IMES

/***************************************************************************/
/*                                                                         */
/* Subroutine - Turns for time in milliseconds                             */
/*                                                                         */
/***************************************************************************/
byte turnTime(const int power, const int timeMs)
{
	int startingTime = time1[T1];

	setLeftDriveMotorsRaw(power);             //Set left side to its power
	setRightDriveMotorsRaw(-1 * power);       //Set right side to its power

	while (startingTime + timeMs > time1[T1]) //Wait for timeMs
	{}

	setDriveMotorsRaw(0);                     //Stop

	return 0;
}

#ifdef USING_QUADS

/***************************************************************************/
/*                                                                         */
/* Subroutine - Turns for distance in units (default: inches)              */
/*                                                                         */
/***************************************************************************/
byte turnQuad(const int power, const int ticks)
{
	SensorValue[leftDriveQuad] = 0;  //Clear left encoder
	SensorValue[rightDriveQuad] = 0; //Clear right encoder

	int rDiff;                       //Difference between sides
	int rMod;                        //10% of power in the direction of rDiff

	//Full power for 60% of ticks
	while (abs(SensorValue[leftDriveQuad]) < abs(ticks) * 0.6)
	{
		rDiff = abs(SensorValue[leftDriveQuad]) - abs(SensorValue[rightDriveQuad]); //Difference between sides
		rMod = sgn(rDiff) * power * 0.1;                                            //10% of power in the direction of rDiff
		setLeftDriveMotorsRaw(power);                                               //Directly control left side
		setRightDriveMotorsRaw((-1 * power) - rMod);                                //Have right side adjust to keep in tune with left side
	}

	//1/3 power for last 40% of ticks
	while (abs(SensorValue[leftDriveQuad]) < abs(ticks))
	{
		rDiff = abs(SensorValue[leftDriveQuad]) - abs(SensorValue[rightDriveQuad]); //Difference between sides
		rMod = sgn(rDiff) * power * 0.1;                                            //10% of power in the direction of rDiff
		setLeftDriveMotorsRaw(power / 3);                                           //Directly control left side
		setRightDriveMotorsRaw((-1 * (power * 0.8)) - rMod);                        //Have right side adjust to keep in tune with left side
	}

	turnTime(-1 * (power / 2), 50);  //Brake at -50% power for a short time to eliminate momentum
	setDriveMotorsRaw(0);         //Stop

	return 0;
}

#endif //USING_QUADS

#ifdef USING_IMES

/***************************************************************************/
/*                                                                         */
/* Subroutine - Turns for distance in units (default: inches)              */
/*                                                                         */
/***************************************************************************/
byte turnIME(const int power, const int ticks)
{
	nMotorEncoder[leftDriveFront] = 0;  //Clear left IME
	nMotorEncoder[rightDriveFront] = 0; //Clear right IME

	int rDiff;                          //Difference between sides
	int rMod;                           //10% of power in the direction of rDiff

	//Full power for 60% of ticks
	while (abs(nMotorEncoder[leftDriveFront]) < abs(ticks) * 0.6)
	{
		rDiff = abs(nMotorEncoder[leftDriveFront]) - abs(nMotorEncoder[rightDriveFront]); //Difference between sides
		rMod = sgn(rDiff) * power * 0.1;                                                  //10% of power in the direction of rDiff
		setLeftDriveMotorsRaw(power);                                                     //Directly control left side
		setRightDriveMotorsRaw((-1 * power) - rMod);                                      //Have right side adjust to keep in tune with left side
	}

	//1/3 power for last 40% of ticks
	while (abs(nMotorEncoder[leftDriveFront]) < abs(ticks))
	{
		rDiff = abs(nMotorEncoder[leftDriveFront]) - abs(nMotorEncoder[rightDriveFront]); //Difference between sides
		rMod = sgn(rDiff) * power * 0.1;                                                  //10% of power in the direction of rDiff
		setLeftDriveMotorsRaw(power / 3);                                                 //Directly control left side
		setRightDriveMotorsRaw((-1 * (power * 0.8)) - rMod);                              //Have right side adjust to keep in tune with left side
	}

	turnTime(-1 * (power / 2), 50);     //Brake at -50% power for a short time to eliminate momentum
	setAllDriveMotorsRaw(0);            //Stop

	return 0;
}

#endif //USING_IMES

#ifdef USING_GYRO

/***************************************************************************/
/*                                                                         */
/* Subroutine - Turns for distance in degrees                              */
/*                                                                         */
/***************************************************************************/
byte turnGyro(const float deg)
{
	//Clear the gyro
	SensorValue[gyro] = 0;

	//Scale degrees down to gyro ticks
	int ticks = (int)(deg * 10);

	//Loop timeout
	const int timeout = 1000;

	//Timer for timeout
	timer t;
	timer_Initialize(&t);

	//Position PID controller for turning
	pos_PID pid;
	pos_PID_InitController(&pid, gyro, 0.5, 0, 0.04);

	//Set controller's target position
	pos_PID_SetTargetPosition(&pid, ticks);

	//Marker for timeout
	timer_PlaceMarker(&t);

	//Turn to target
	do
	{
		//Step controller
		pos_PID_StepController(&pid);

		//Set drive to controller's output
		setLeftDriveMotorsRaw(-1 * pos_PID_GetOutput(&pid));
		setRightDriveMotorsRaw(pos_PID_GetOutput(&pid));

		//Exit if taking too long
		if (timer_GetDTFromMarker(&t) > timeout)
		{
			setDriveMotorsRaw(0);
			return 1;
		}

		//Exit if output is too small to turn the robot
		if (abs(pos_PID_GetOutput(&pid)) <= 10)
		{
			setDriveMotorsRaw(0);
			return 1;
		}

		wait1Msec(10);
	}
	while (abs(pos_PID_GetError(&pid)) > 0);

	//Stop drive motors
	setDriveMotorsRaw(0);

	return 0;
}

#endif //USING_GYRO

#endif //AUTONOMOUSFUNCTIONS_H_INCLUDED
