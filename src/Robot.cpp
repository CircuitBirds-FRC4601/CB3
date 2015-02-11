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





float rx,ry,rax,ray,lay,lax;
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
	VictorSP gearMotor,rightArm,leftArm;
	Relay claw;
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
		rightArm(5),
		leftArm(6),
		claw(0),
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
		lax = gamePad.GetRawAxis(0);
		lay = gamePad.GetRawAxis(1);

		rax = gamePad.GetRawAxis(2);
		ray = gamePad.GetRawAxis(3);

		gearMotor.Set(lay);
		leftArm.Set(lax);
		rightArm.Set(rax);

		//X
		if(gamePad.GetRawButton(1))
		{
			claw.Set(Relay::kForward);
			claw.Set(Relay::kOff);
		}
		//A
		if(gamePad.GetRawButton(2))
		{

		}
		//B
		if(gamePad.GetRawButton(3))
		{

		}
		//Y
		if(gamePad.GetRawButton(4))
		{

		}
		//LB
		if(gamePad.GetRawButton(5))
		{

		}
		//RB
		if(gamePad.GetRawButton(6))
		{

		}
		//LT
		if(gamePad.GetRawButton(7))
		{

		}
		//RT
		if(gamePad.GetRawButton(8))
		{

		}
		//BACK/SEL
		if(gamePad.GetRawButton(9))
		{

		}
		//START
		if(gamePad.GetRawButton(10))
		{

		}
		//L stick
		if(gamePad.GetRawButton(11))
		{

		}
		//R stick
		if(gamePad.GetRawButton(12))
		{

		}


		#ifdef MECHANUM
			rx = x * sncs - y * sncs;
			ry = x * sncs + y * sncs;
			robotDrive.MecanumDrive_Cartesian(rx,ry,rateGyro.GetRate(),rateGyro.GetAngle());
		#endif
		#ifdef TANK
			robotDrive.TankDrive(-leftStick.GetY(),-rightStick.GetY());
		#endif






			SmartDashboard::PutNumber("PDB Temp",(float)PDB.GetTemperature());
			SmartDashboard::PutNumber("PDB Current",PDB.GetCurrent(14));
			SmartDashboard::PutNumber("RateGyro",(int)rateGyro.GetRate());
			SmartDashboard::PutNumber("EncoderGet",liftPos.Get());
			SmartDashboard::PutNumber("Encoder",liftPos.GetDistance());
			SmartDashboard::PutNumber("Right Stick",rightStick.GetY());
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
