// CB3 Ver .75
//
//     Change log
// 16 Feb wiremap at bottom, I2C sections vestigial (not workin')
// 16 Feb mapped claw to a spare victor/PWM pair.
// 14 Feb added accelerometer tilt output
//
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
float tilt_theta, tilt_phi, gmag, gmin, gmax, Bx, By, Bz, Bmag, Bmin, Bmax, heading;
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
	VictorSP gearMotor,rightArm,leftArm, claw;
	RobotDrive robotDrive;
	Encoder liftPos,leftArmEncoder,rightArmEncoder,lDriveEncoder,rDriveEncoder;
	Accelerometer *accel;

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
		liftPos(8,9),
		leftArmEncoder(6,7),
		rightArmEncoder(4,5),
		lDriveEncoder(2,3),
		rDriveEncoder(0,1)
	{
		PDB.InitTable(hi);
		PDB.StartLiveWindowMode();
		liftPos.InitTable(table);
		liftPos.StartLiveWindowMode();
		leftArmEncoder.InitTable(table);
		leftArmEncoder.StartLiveWindowMode();
		rightArmEncoder.InitTable(table);
		rightArmEncoder.StartLiveWindowMode();
		lDriveEncoder.InitTable(table);
		lDriveEncoder.StartLiveWindowMode();
		rDriveEncoder.InitTable(table);
		rDriveEncoder.StartLiveWindowMode();
	}

private:

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		#ifdef CLRFAULTS
			PDB.ClearStickyFaults();
		#endif
		liftPos.Reset();
		rightArmEncoder.Reset();
		leftArmEncoder.Reset();
		lDriveEncoder.Reset();
		rDriveEncoder.Reset();

		accel = new BuiltInAccelerometer(Accelerometer::Range::kRange_2G);
	             // setup values
	gmax=12.0;   //  in m/s^2
	gmin=8.0;
	Bmax=680.0;  //  in milligauss
	Bmin=410.0;
	}
	void RobotPeriodic()
	{
		PDB.UpdateTable();
		liftPos.UpdateTable();
		rightArmEncoder.UpdateTable();
		leftArmEncoder.UpdateTable();
		lDriveEncoder.UpdateTable();
		rDriveEncoder.UpdateTable();
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
		rightArmEncoder.Reset();
		leftArmEncoder.Reset();
		lDriveEncoder.Reset();
		rDriveEncoder.Reset();

		data = 0 ;
	}
	void TeleopPeriodic()
	{
		lax = gamePad.GetRawAxis(0);
		lay = gamePad.GetRawAxis(1);
		rax = gamePad.GetRawAxis(2);
		ray = gamePad.GetRawAxis(3);
		if (!(0 || 0))
		{
			if (lay>0) lay=lay*.4;
			gearMotor.Set(-lay);
		}
		if (!(0 || 0)) leftArm.Set(-lax);
		if (!(0 || 0)) rightArm.Set(rax);
		if (!(0 || 0)) claw.Set(ray*.3);
		// determine tilt
			double xVal = accel->GetX();
			double yVal = accel->GetY();
			double zVal = accel->GetZ();
			gmag = xVal*xVal+yVal*yVal+zVal*zVal;
			if ((gmag<gmax*gmax)&&(gmag>gmin*gmin)) { //validate
					gmag = pow(gmag,0.5);
					tilt_theta = acos(zVal/gmag);
					tilt_phi   = atan(yVal/xVal);
			}
			// end get tilt
			// compute heading
			Bmag = Bx*Bx+By*By+Bz*Bz;
			if ((Bmag<Bmax*Bmax)&&(Bmag>Bmin*Bmin)){ //validate
				Bmag = Bx;
				Bx = cos(tilt_phi)*Bmag + sin(tilt_phi)*(cos(tilt_theta)*By+sin(tilt_theta)*Bz);
				By = -sin(tilt_phi)*Bmag + cos(tilt_phi)*(cos(tilt_theta)*By+sin(tilt_theta)*Bz);
				heading = atan(By/Bx);
			}
			// end compute heading
		//button mappings : GAMEPAD
		//X
		if(gamePad.GetRawButton(1))
		{
			getHeading();
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
			SmartDashboard::PutNumber("PDB Current Lift",PDB.GetCurrent(2));
			SmartDashboard::PutNumber("PDB Current Left Arm",PDB.GetCurrent(3));
			SmartDashboard::PutNumber("PDB Current Right Arm",PDB.GetCurrent(12));

			SmartDashboard::PutNumber("Lift Throttle",(int)25*lay);
			SmartDashboard::PutNumber("Right Drive Throttle",(int)25*lay);
			SmartDashboard::PutNumber("Left Drive Throttle",(int)25*lay);
			SmartDashboard::PutNumber("Left Arm Throttle",(int)25*lay);
			SmartDashboard::PutNumber("Right Arm Throttle",(int)25*lay);

			SmartDashboard::PutNumber("I2Cmsgs",counti2c);

			SmartDashboard::PutNumber("Lift Pos", liftPos.GetDistance());
			SmartDashboard::PutNumber("Right Arm Pos", rightArmEncoder.GetRaw());
			SmartDashboard::PutNumber("Left Arm Pos", leftArmEncoder.GetRaw());
			SmartDashboard::PutNumber("Left Drive", lDriveEncoder.GetRaw());
			SmartDashboard::PutNumber("Right Drive", rDriveEncoder.GetRaw());

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
			printf("Arduino: %X\n",*buffer[1]);
		}
	}
	void getHeading()
	{
		I2C Wire(I2C::kOnboard, 0x1E);

		Wire.Write(0x02,0x00);
		//Wire.Write(0xA,payload);

		try
		{
			uint8_t *bufferz[7];
			if(!Wire.Read(0x03,6, *bufferz))
			{
				counti2c++;
				for(int q=0;q <=6;q++)
				{
				printf("byte:%i data:%X\n",q,*bufferz[q]);
				}
			}
		}
		catch(int e)
		{
			printf("HI %i\n",e);
		}
	}


};
START_ROBOT_CLASS(Robot);

/*
/---------------------------\
|          PORT MAP         |
-----------------------------------------\
                                          |
                                          |
PWM                                       |
0  Drive RF   Talon1   R Stick            |
1  Drive RB   Talon2   R Stick            |
2  Drive LF   Talon3   L Stick            |
3  Drive LB   Talon4   L Stick            |
4  Lift       V.SP1    gamepd, left Y     |
5  R. Arm     V.SP2    gamepd, right x    |
6  L. Arm     V.SP3     '', left x        |
7  Claw FWD/REV    V.SP4   '', right y    |
8                                         |
9                                         |
                                          |
                                          |
digital I/O                               |
0  R Base optical-A                       |
1  R Base optical-B                       |
2  L Base optical-A                       |
3  L Base optical-B                       |
4  R Arm capacitive-A                     |
5  R Arm capacitive-B                     |
6  L Arm capacitive-A                     |
7  L Arm capacitive-B                     |
8  Lift capacitive-A                      |
9  Lift capacitive-B                      |
                                          |
Expansion Digital I/O (these are not )    |
10 Claw closed limit IN                   |
11 R. Arm capacitive sync IN              |
12 L. Arm capacitive sync IN              |
13                                        |
14                                        |
15                                        |
16 rPi sync pulse         OUT             |
17 rPi power reset line   OUT             |
                                          |
Relay                                     |
1                   |
2                                         |
3                                         |
4                                         |
                                          |
                                          |
I2C  : Magnetometer : Arduino             |
                                          |
Arduino (switches)                                  |
3    lift T                               |
4    lift B                               |
5  R. Arm R                               |
6  R. Arm L                               |
7  R. Arm T                               |
8  L. Arm R                               |
9  L. Arm L                               |
10 L. Arm T                               |
                                          |
built in accelerometer: detecting tilt    |
------------------------------------------/
*/
