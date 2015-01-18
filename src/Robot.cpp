#include "WPILib.h"
#include "Robot.h"
#include <math.h>
#include <fstream>

/*******************************/
/**********DRIVE BASE***********/
/*******************************/
//#define MECHANUM
#define TANK

/*******************************/
/**********BUILD FLAGS**********/
/*******************************/

//#define CLRFAULTS



float rx,ry,x,y;
//int lastLiftCount,currentLiftCount;
//computed from 45*M_PI/180
#define sncs 0.707106781187;


class Robot: public IterativeRobot
{


	Joystick gamePad,leftStick,rightStick;
	LiveWindow *lw;
	int autoLoopCounter;
	PowerDistributionPanel PDB;
	ITable *hi;
	ITable *table;
	Gyro rateGyro;
	Talon fRight,fLeft,bRight,bLeft;
	Victor gearMotor;
	RobotDrive robotDrive;
	Encoder liftPos;


public:
	Robot() :
		gamePad(0),
		leftStick(1),
		rightStick(2),
		lw(NULL),
		autoLoopCounter(0),
		PDB(),
		hi(),
		table(),
		rateGyro(RATEGYRO1),
		fRight(0),
		fLeft(2),
		bRight(1),
		bLeft(3),
		gearMotor(4),
		robotDrive(fLeft,bLeft,fRight,bRight),
		liftPos(11,12)
	{
		PDB.InitTable(hi);
		PDB.StartLiveWindowMode();
		liftPos.InitTable(table);
		liftPos.StartLiveWindowMode();
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
		liftPos.Reset();
	}
	void RobotPeriodic()
	{
		PDB.UpdateTable();
		liftPos.UpdateTable();
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		liftPos.Reset();
	}

	void TeleopPeriodic()
	{
		x = gamePad.GetRawAxis(0);
		y = gamePad.GetRawAxis(1);
		gearMotor.Set(y);
		//liftPos.Get();
		//std::ofstream myfile;
		//myfile.open ("example.txt");
		//myfile << currentLiftCount;
		if(gamePad.GetRawButton(1))
		{

		}

		#ifdef MECHANUM
			rx = x * sncs - y * sncs;
			ry = x * sncs + y * sncs;
			robotDrive.MecanumDrive_Cartesian(rx,ry,rateGyro.GetRate(),rateGyro.GetAngle());
		#endif
		#ifdef TANK
			robotDrive.TankDrive(leftStick,rightStick);
		#endif
			SmartDashboard::PutNumber("PDB Temp",(float)PDB.GetTemperature());
					SmartDashboard::PutNumber("PDB Current",PDB.GetCurrent(14));
					SmartDashboard::PutNumber("RateGyro",(int)rateGyro.GetRate());
					SmartDashboard::PutNumber("EncoderGet",liftPos.Get());
					SmartDashboard::PutNumber("Encoder",liftPos.GetDistance());
	}

	void DisabledInit()
	{

	}

	void DisabledPeriodic()
	{


	}

	void TestPeriodic()
	{
		lw->Run();
	}

};









START_ROBOT_CLASS(Robot);
