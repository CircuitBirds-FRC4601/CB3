#include "WPILib.h"
#include "Robot.h"
#include <math.h>
#include <fstream>

/*******************************/
/**********DRIVE BASE***********/
/*******************************/
//#define MECHANUM
#define MECHANUM

/*******************************/
/**********BUILD FLAGS**********/
/*******************************/

//#define CLRFAULTS

float rx,ry,x,y;
//computed from 45*M_PI/180
float sncs = 0.707106781187;


class Robot: public IterativeRobot
{


	Joystick gamePad;
	LiveWindow *lw;
	int autoLoopCounter;
	PowerDistributionPanel PDB;
	ITable *hi;
	Gyro rateGyro;
	Talon fRight,fLeft,bRight,bLeft;
	RobotDrive robotDrive;

public:
	Robot() :
		gamePad(0),
		lw(NULL),
		autoLoopCounter(0),
		PDB(),
		hi(),
		rateGyro(RATEGYRO1),
		fRight(0),
		fLeft(2),
		bRight(1),
		bLeft(3),
		robotDrive(fLeft,bLeft,fRight,bRight)
	{
		PDB.InitTable(hi);
		PDB.StartLiveWindowMode();
		//rateGyro.InitTable(hi);
		//rateGyro.StartLiveWindowMode();

	}

private:

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		#ifdef CLRFAULTS
			PDB.ClearStickyFaults();
		#endif
	}
	void RobotPeriodic()
	{
		PDB.UpdateTable();
		writeDS();
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		x = gamePad.GetRawAxis(0);
		y = gamePad.GetRawAxis(1);
		#ifdef MECHANUM
			rx = x * 0.707106781187 - y * 0.707106781187;
			ry = x * 0.707106781187 + y * 0.707106781187;
			robotDrive.MecanumDrive_Cartesian(rx,ry,rateGyro.GetRate(),rateGyro.GetAngle());
		#endif
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void writeDS()
	{
		SmartDashboard::PutNumber("PDB Temp",(float)PDB.GetTemperature());
		SmartDashboard::PutNumber("PDB Current",PDB.GetCurrent(4));
		SmartDashboard::PutNumber("RateGyro",(int)rateGyro.GetRate());
	}
	void RotateVector(float ox, float oy)
	{

	}


};









START_ROBOT_CLASS(Robot);
