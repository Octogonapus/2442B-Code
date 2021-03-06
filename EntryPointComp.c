#pragma config(Sensor, in1,    powerExpander,  sensorAnalog)
#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  leftLauncherQuad, sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  leftDriveQuad,  sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  rightDriveQuad, sensorQuadEncoder)
#pragma config(Sensor, dgtl10, testPin,        sensorDigitalOut)
#pragma config(Sensor, dgtl11, rightLauncherQuad, sensorQuadEncoder)
#pragma config(Motor,  port1,           intakeRight,   tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           intakeLeft,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           rightOuter,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           rightInner,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           leftInner,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           leftOuter,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           leftDriveFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           rightDriveFront, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           leftDriveBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          rightDriveBack, tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"

//Setup LCD
#define LCD_SAFETY_REQ_COMP_SWITCH
#define MENU_NUM 8
#define USING_QUADS
#define USING_GYRO

#include "Bulldog_Core_Includes.h"
#include "autonFunctions.c"
#include "autonIncludes.h"
#include "programmingSkills.c"

//Auton function prototype
void startAutonomous();

//Menus
menu *driverSkillsMenu;
menu *programmingSkillsMenu;
menu *autonSelectionMenu;
menu *endPreAutonMenu;
menu *batteryVoltageMenu;
menu *powerExpanderVoltageMenu;
menu *backupBatteryVoltageMenu;

//Whether or not to end pre auton
bool endPreAuton = false;

//Auton selection
int autonSelection = -1;

//Launcher velocity controller
vel_TBH tbh;
vel_PID pidLeft, pidRight;
bangBang bbLeft, bbRight;

void pre_auton()
{
  bStopTasksBetweenModes = true;

  //Setup launcher controller
  vel_TBH_InitController(&tbh, leftLauncherQuad, 0.25, 69);
  vel_PID_InitController(&pidLeft, leftLauncherQuad, 0.05, 0.008);
  vel_PID_InitController(&pidRight, rightLauncherQuad, 0.055, 0.02);
  bangBang_InitController(&bbLeft, leftLauncherQuad, 110, 60);
  bangBang_InitController(&bbRight, rightLauncherQuad, 100, 40);

  //Setups sensors
  initializeSensors();

  //Setup motors
  addMotor(intakeRight, MOTOR_FAST_SLEW_RATE);
  addMotor(intakeLeft, MOTOR_FAST_SLEW_RATE);
  addMotor(leftOuter, 30);
  addMotor(leftInner, 30);
  addMotor(rightInner, 30);
  addMotor(rightOuter, 30);
  addMotor(leftDriveFront, 50);
  addMotor(leftDriveBack, 50);
  addMotor(rightDriveFront, 50);
  addMotor(rightDriveBack, 50);

	//Setup menu system
	autonSelectionMenu = newMenu("Select Auton", 2);
	programmingSkillsMenu = newMenu("Prog Skills", 3);
	driverSkillsMenu = newMenu("Driver Skills", 4);
	endPreAutonMenu = newMenu("Confirm", 1);

	string batteryVoltage;
	sprintf(batteryVoltage, "Main: %1.2f%c", nAvgBatteryLevel / 1000.0, 'V');
	batteryVoltageMenu = newMenu(batteryVoltage);

	string powerExpanderVoltage;
	sprintf(powerExpanderVoltage, "Expander: %1.2f%c", SensorValue[powerExpander] / ANALOG_IN_TO_V, 'V');
	powerExpanderVoltageMenu = newMenu(powerExpanderVoltage);

	string backupBatteryVoltage;
	sprintf(backupBatteryVoltage, "Backup: %1.2f%c", BackupBatteryLevel / 1000.0, 'V');
	backupBatteryVoltageMenu = newMenu(backupBatteryVoltage);

	linkMenus(driverSkillsMenu, programmingSkillsMenu, autonSelectionMenu, endPreAutonMenu, batteryVoltageMenu, powerExpanderVoltageMenu, backupBatteryVoltageMenu);

	bLCDBacklight = true;

	//Wait for user to confirm
	//startTask(updateLCDTask);
	//while (!getLCDSafetyState() && !endPreAuton) { wait1Msec(50); }
}

task autonomous()
{
	//startAutonomous();
	driveQuad_PID(500);
}

//int targetVelocity = 190, targetVelocity_Last = 0;
//int launcherCurrentPower_Left = 0, launcherCurrentPower_Right = 0;

//void test_15ms();
//void test_5ms();

task usercontrol()
{
	startTask(autonomous);
	wait1Msec(15000);

	startTask(motorSlewRateTask);

	//Drivetrain variables
	int leftV = 0, rightV = 0;

	//Launcher variables
	bool launcherOn = false;
	int targetVelocity = 190, targetVelocity_Last = 0, targetVelocity_Increment = 2;
	int launcherCurrentPower_Left = 0, launcherCurrentPower_Right = 0;

	timer t;
	timer_Initialize(&t);
	string lcdLine1, lcdLine2;

	float avgError_left = 0, avgError_right = 0;
	int iter = 0;

	//writeDebugStreamLine("Left Error, Right Error");
	//while (timer_GetDTFromStart(&t) < 5000)
	while (true)
	{
		if (timer_Repeat(&t, 100))
		{
			sprintf(lcdLine1, "TV: %d, CV: %d", targetVelocity, bbLeft.currentVelocity);
			displayLCDCenteredString(0, lcdLine1);

			//sprintf(lcdLine2, "LE: %d, RE: %d", bbLeft.error, bbRight.error);
			displayLCDCenteredString(1, lcdLine2);
		}

		//writeDebugStreamLine("%d,%d", pidLeft.error, pidRight.error);

		/* ------------ DRIVETRAIN ------------ */

		//Grab values from joystick
		leftV = vexRT[JOY_JOY_LV];
		rightV = vexRT[JOY_JOY_RV];

		//Bound joystick values
		leftV = abs(leftV) < JOY_THRESHOLD ? 0 : leftV;
		rightV = abs(rightV) < JOY_THRESHOLD ? 0 : rightV;

		//Send these values to the drivetrain
		setLeftDriveMotors(leftV);
		setRightDriveMotors(rightV);

		/* -------------- INTAKE -------------- */

		if (vexRT[JOY_TRIG_LU])
		{
			setMotorSpeed(intakeRight, 127);
			setMotorSpeed(intakeLeft, 127);
		}
		else if (vexRT[JOY_TRIG_LD])
		{
			setMotorSpeed(intakeRight, -127);
			setMotorSpeed(intakeLeft, -127);
		}
		else
		{
			setMotorSpeed(intakeRight, 0);
			setMotorSpeed(intakeLeft, 0);
		}

		/* -------------- FLYWHEEL -------------- */

		if (vexRT[JOY_TRIG_RU])
		{
			launcherOn = !launcherOn;
			waitForZero(vexRT[JOY_TRIG_RU]);
		}

		if (launcherOn)
		{
			//Set new target velocity if target changed
			if (targetVelocity != targetVelocity_Last)
			{
				//vel_TBH_SetTargetVelocity(&tbh, targetVelocity);
				vel_PID_SetTargetVelocity(&pidLeft, targetVelocity);
				vel_PID_SetTargetVelocity(&pidRight, targetVelocity);
				//bangBang_SetTargetVelocity(&bbLeft, targetVelocity);
				//bangBang_SetTargetVelocity(&bbRight, targetVelocity);
			}

			//Remember old target velocity
			targetVelocity_Last = targetVelocity;

			//Step controller
			//launcherCurrentPower = vel_TBH_StepController(&tbh);
			launcherCurrentPower_Left = vel_PID_StepController(&pidLeft);
			launcherCurrentPower_Right = vel_PID_StepController(&pidRight);
			//launcherCurrentPower_Left = bangBang_StepController(&bbLeft);
			//launcherCurrentPower_Right = bangBang_StepController(&bbRight);

			//Bound power
			//launcherCurrentPower = launcherCurrentPower < 0 ? 0 : launcherCurrentPower;
			launcherCurrentPower_Left = launcherCurrentPower_Left < 0 ? 0 : launcherCurrentPower_Left;
			launcherCurrentPower_Right = launcherCurrentPower_Right < 0 ? 0 : launcherCurrentPower_Right;

			//Send power to motors
			//setLauncherMotors(launcherCurrentPower);
			setLeftLauncherMotors(launcherCurrentPower_Left);
			setRightLauncherMotors(launcherCurrentPower_Right);
		}
		else
		{
			vel_PID_StepVelocity(&pidLeft);
			vel_PID_StepVelocity(&pidRight);
			//bangBang_StepVelocity(&bbLeft);
			//bangBang_StepVelocity(&bbRight);
			setLauncherMotors(0);
		}

		//Adjust launcher target velocity
		if (vexRT[JOY_BTN_RU])
		{
			targetVelocity += targetVelocity_Increment;
			waitForZero(vexRT[JOY_BTN_RU]);
		}
		else if (vexRT[JOY_BTN_RD])
		{
			targetVelocity -= targetVelocity_Increment;
			waitForZero(vexRT[JOY_BTN_RD]);
		}

		//Target velocity presets
		if (vexRT[JOY_BTN_RL])
		{
			targetVelocity = 180;
			waitForZero(vexRT[JOY_BTN_RL]);
		}
		else if (vexRT[JOY_BTN_RR])
		{
			targetVelocity = 210;
			waitForZero(vexRT[JOY_BTN_RR]);
		}

		avgError_left += bangBang_GetError(&bbLeft);
		avgError_right += bangBang_GetError(&bbRight);
		iter++;

		//Average and reset
		if (iter >= 500)
		{
			avgError_left /= iter;
			avgError_right /= iter;
			iter = 0;

			sprintf(lcdLine2, "L:%1.2f, R:%1.2f", avgError_left, avgError_right);

			avgError_left = 0;
			avgError_right = 0;
		}

		//Main loop wait
		wait1Msec(15);
	}

	//test_15ms();
	//test_15ms();
	//test_15ms();
	//test_15ms();
	//test_15ms();
	////test_5ms();
	////test_5ms();

	//stopTask(motorSlewRateTask);
	//allMotorsOff();
}

//Run an autonomous function based on current selection
void startAutonomous()
{
	//Naming convention: <red side = 1, blue side = 2><left side = 1, right side = 2><primary = 1, secondary = 2, tertiary = 3>

	switch (autonSelection)
	{
		case 111:
			redLeftAutonPrimary();
			break;

		case 112:
			redLeftAutonSecondary();
			break;

		case 113:
			redLeftAutonTertiary();
			break;

		case 121:
			redRightAutonPrimary();
			break;

		case 122:
			redRightAutonSecondary();
			break;

		case 123:
			redRightAutonTertiary();
			break;

		case 211:
			blueLeftAutonPrimary();
			break;

		case 212:
			blueLeftAutonSecondary();
			break;

		case 213:
			blueLeftAutonTertiary();
			break;

		case 221:
			blueRightAutonPrimary();
			break;

		case 222:
			blueRightAutonSecondary();
			break;

		case 223:
			blueRightAutonTertiary();
			break;

		default:
			break;
	}
}

void invoke(int func)
{
	switch (func)
	{
		case 1:
			endPreAuton = true;
			stopTask(updateLCDTask);
			break;

		case 2:
			autonSelection = selectAutonomous();
			break;

		case 3:
			//Run programming skills
			break;

		case 4:
			//Run driver skills
			break;

		default:
			break;
	}
}

//void test_15ms()
//{
//	timer lt;
//	timer_Initialize(&lt);

//	float avgError_left = 0, avgError_right = 0;
//	int iter = 0;

//	writeDebugStreamLine("15 ms test");

//	timer_PlaceMarker(&lt);
//	while (timer_GetDTFromMarker(&lt) < 5000)
//	{
//		//Set new target velocity if target changed
//		if (targetVelocity != targetVelocity_Last)
//		{
//			//vel_TBH_SetTargetVelocity(&tbh, targetVelocity);
//			//vel_PID_SetTargetVelocity(&pid, targetVelocity);
//			bangBang_SetTargetVelocity(&bbLeft, targetVelocity);
//			bangBang_SetTargetVelocity(&bbRight, targetVelocity);
//		}

//		//Remember old target velocity
//		targetVelocity_Last = targetVelocity;

//		//Step controller
//		//launcherCurrentPower = vel_TBH_StepController(&tbh);
//		//launcherCurrentPower = vel_PID_StepController(&pid);
//		launcherCurrentPower_Left = bangBang_StepController(&bbLeft);
//		launcherCurrentPower_Right = bangBang_StepController(&bbRight);

//		//Bound power
//		//launcherCurrentPower = launcherCurrentPower < 0 ? 0 : launcherCurrentPower;
//		launcherCurrentPower_Left = launcherCurrentPower_Left < 0 ? 0 : launcherCurrentPower_Left;
//		launcherCurrentPower_Right = launcherCurrentPower_Right < 0 ? 0 : launcherCurrentPower_Right;

//		//Send power to motors
//		//setLauncherMotors(launcherCurrentPower);
//		setLeftLauncherMotors(launcherCurrentPower_Left);
//		setRightLauncherMotors(launcherCurrentPower_Right);

//		avgError_left += bangBang_GetError(&bbLeft);
//		avgError_right += bangBang_GetError(&bbRight);
//		iter++;

//		//Main loop wait
//		wait1Msec(15);
//	}

//	avgError_left /= iter;
//	avgError_right /= iter;

//	writeDebugStreamLine("Average Error at 15ms; Left: %1.2f, Right: %1.2f", avgError_left, avgError_right);
//}

//void test_5ms()
//{
//	timer lt;
//	timer_Initialize(&lt);

//	float avgError_left = 0, avgError_right = 0;
//	int iter = 0;

//	writeDebugStreamLine("5 ms test");

//	timer_PlaceMarker(&lt);
//	while (timer_GetDTFromMarker(&lt) < 5000)
//	{
//		//Set new target velocity if target changed
//		if (targetVelocity != targetVelocity_Last)
//		{
//			//vel_TBH_SetTargetVelocity(&tbh, targetVelocity);
//			//vel_PID_SetTargetVelocity(&pid, targetVelocity);
//			bangBang_SetTargetVelocity(&bbLeft, targetVelocity);
//			bangBang_SetTargetVelocity(&bbRight, targetVelocity);
//		}

//		//Remember old target velocity
//		targetVelocity_Last = targetVelocity;

//		//Step controller
//		//launcherCurrentPower = vel_TBH_StepController(&tbh);
//		//launcherCurrentPower = vel_PID_StepController(&pid);
//		launcherCurrentPower_Left = bangBang_StepController(&bbLeft);
//		launcherCurrentPower_Right = bangBang_StepController(&bbRight);

//		//Bound power
//		//launcherCurrentPower = launcherCurrentPower < 0 ? 0 : launcherCurrentPower;
//		launcherCurrentPower_Left = launcherCurrentPower_Left < 0 ? 0 : launcherCurrentPower_Left;
//		launcherCurrentPower_Right = launcherCurrentPower_Right < 0 ? 0 : launcherCurrentPower_Right;

//		//Send power to motors
//		//setLauncherMotors(launcherCurrentPower);
//		setLeftLauncherMotors(launcherCurrentPower_Left);
//		setRightLauncherMotors(launcherCurrentPower_Right);

//		avgError_left += bangBang_GetError(&bbLeft);
//		avgError_right += bangBang_GetError(&bbRight);
//		iter++;

//		//Main loop wait
//		wait1Msec(5);
//	}

//	avgError_left /= iter;
//	avgError_right /= iter;

//	writeDebugStreamLine("Average Error at 5ms; Left: %1.2f, Right: %1.2f", avgError_left, avgError_right);
//}
