#include "WPILib.h"
#include "Robot.h"

class Robot: public IterativeRobot
{

	//RobotDrive myRobot; // robot drive system
	//Joystick stick; // only joystick
	LiveWindow *lw;
	int autoLoopCounter;
	//DigitalOutput Dout;
	DigitalInput ButtonTest;
	PowerDistributionPanel PDB;
	ITable *hi;
	Gyro rateGyro;
	AnalogInput pot;
	DigitalOutput led;

public:
	Robot() :
		//myRobot(0, 1),
		//stick(0),
		lw(NULL),
		autoLoopCounter(0),
		ButtonTest(0),
		PDB(),
		hi(),
		rateGyro(RATEGYRO1),
		pot(POTE),
		led(1)


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
		PDB.ClearStickyFaults();
	}
	void RobotPeriodic()
	{
		PDB.UpdateTable();
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
		led.Set(true);
		SmartDashboard::PutBoolean("DigitalRead",ButtonTest.Get());
		SmartDashboard::PutNumber("PDB Temp",(float)PDB.GetTemperature());
		SmartDashboard::PutNumber("PDB Current",PDB.GetCurrent(4));
		SmartDashboard::PutNumber("RateGyro",(int)rateGyro.GetRate());
		SmartDashboard::PutNumber("Pot",pot.GetValue());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
