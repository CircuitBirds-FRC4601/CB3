#include "WPILib.h"
#include "Robot.h"

class Robot: public IterativeRobot
{

	//RobotDrive myRobot; // robot drive system
	//Joystick stick; // only joystick
	LiveWindow *lw;
	int autoLoopCounter;
	DigitalOutput Dout;
	PowerDistributionPanel PDB;
	ITable *hi;
	//SmartDashboard smDash;
public:
	Robot() :
		//myRobot(0, 1),
		//stick(0),
		lw(NULL),
		autoLoopCounter(0),
		Dout(1),
		PDB(),
		hi()
		//smDash()
	{
		PDB.InitTable(hi);
		PDB.StartLiveWindowMode();

		//myRobot.SetExpiration(0.1);
	}

private:

	void RobotInit()
	{

		lw = LiveWindow::GetInstance();

		Dout.Set(false);
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


		SmartDashboard::PutNumber("PDB Temp",(float)PDB.GetTemperature());
		SmartDashboard::PutNumber("PDB Current",PDB.GetCurrent(4));

	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
