#include "WPILib.h"
#include "Robot.h"
#include <math.h>
#include <fstream>
//#include <bitset>
//#include <string>
#include <iostream>
#include <string>

/*******************************/
/**********DRIVE BASE***********/
/*******************************/
//#define MECHANUM
#define TANK

/*******************************/
/**********BUILD FLAGS**********/
/*******************************/

#define CLRFAULTS



float rx,ry,rax,ray,lay,lax;
int counti2c = 0;
uint8_t data;
long EncoderOffset1,EncoderOffset2,EncoderOffset3;
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
	//Gyro rateGyro;
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
		lw(),
		autoLoopCounter(0),
		PDB(),
		hi(),
		table(),
		//rateGyro(0),
		fRight(0),
		fLeft(2),
		bRight(1),
		bLeft(3),
		gearMotor(4),
		rightArm(5),
		leftArm(6),
		claw(0),
		robotDrive(fLeft,bLeft,fRight,bRight),
		liftPos(0,1)
	{
		PDB.InitTable(hi);
		PDB.StartLiveWindowMode();
		liftPos.InitTable(table);
		liftPos.StartLiveWindowMode();
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
		data = 0;
	}

	void TeleopPeriodic()
	{

		lax = gamePad.GetRawAxis(0);
		lay = gamePad.GetRawAxis(1);

		rax = gamePad.GetRawAxis(2);
		ray = gamePad.GetRawAxis(3);

		if (!(0 || 0)) gearMotor.Set(-lay);
		if (!(0 || 0)) leftArm.Set(-lax);
		if (!(0 || 0)) rightArm.Set(rax);


		//X
		if(gamePad.GetRawButton(1))
		{
			getHeading(0x0);
		}
		//A
		if(gamePad.GetRawButton(2))
		{
			ArduinoRW(0xA);
		}
		//B
		if(gamePad.GetRawButton(3))
		{
			ArduinoRW(0xB);
		}
		//Y
		if(gamePad.GetRawButton(4))
		{
			ArduinoRW(0xC);
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
			SmartDashboard::PutNumber("PDB Current Gear",PDB.GetCurrent(2));
			SmartDashboard::PutNumber("PDB Current Left Arm",PDB.GetCurrent(3));
			SmartDashboard::PutNumber("PDB Current Right Arm",PDB.GetCurrent(12));
			//SmartDashboard::PutNumber("RateGyro",(int)rateGyro.GetRate());
			SmartDashboard::PutNumber("EncoderGet",liftPos.Get());
			SmartDashboard::PutNumber("Encoder",liftPos.GetDistance());
			//SmartDashboard::PutNumber("Right Stick",rightStick.GetY());
			SmartDashboard::PutNumber("Lift Throttle",(int)25*lay);
			SmartDashboard::PutNumber("I2Cmsgs",counti2c);
			SmartDashboard::PutNumber("Last received", data);
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

	void ArduinoRW(uint8_t payload)
	{
		I2C Wire(I2C::kOnboard, 4);

		//std::bitset<8> data("");
		//std::bitset<8> dataIn;
		//bitset::count returns # of 1s
		//bitset::size is total length

		Wire.Write(0xA,payload);
		uint8_t *buffer[1];
		if(!Wire.ReadOnly(1, *buffer))
		{
			counti2c++;
			data = *buffer[0];
		}
	}
	void getHeading(uint8_t payload)
	{
		I2C Wire(I2C::kOnboard, 0x1E);

		Wire.Write(0xA,payload);
		uint8_t *bufferz[8];
		if(!Wire.Read(0x03,6, *bufferz))
		{
			counti2c++;
			data = *bufferz[1];

		}
	}


};




START_ROBOT_CLASS(Robot);
