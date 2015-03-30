// CB3 Ver .81
//
//     Change log
//  1 Mar Verified BuiltInAccel working, installed gyro too (mechanum!)
//          finished first version of auton.
// 28 Feb reset for Arduino mapped to gamepad "start" button
//          a first touch of Auton code. Fixed BuiltinAccelerometer calls
// 26 Feb basic arm and gantry limit switch stops implemented
// 16 Feb wiremap at bottom, I2C sections vestigial (not workin')
// 16 Feb mapped claw to a spare victor/PWM pair.
// 14 Feb added accelerometer tilt output
//
#include "WPILib.h"
#include "Robot.h"
#include <math.h>
//#include <fstream>
//#include <bitset>
//#include <string>
#include <iostream>
#include <string>
#include "Joysticks.h"
#include "GamepadX.h"
/*******************************/
/**********DRIVE BASE***********/
/*******************************/
#define MECHANUM
//#define TANK




/*******************************/
/**********BUILD FLAGS**********/
/*******************************/

#define CLRFAULTS

bool LeftLim = false;
bool RightLim = false;
float rx,ry,rax,ray,lay,lax, time_auton, pi, lsz, lsx, lsy;
float lastcurrent, currentcurrent, maxcurrent;
float tilt_theta, tilt_phi, gmag, gmin, gmax, Bx, By, Bz, Bmag, Bmin, Bmax, heading;
int counti2c, switches, autoPeriodicLoops, liftPosn, rightArmPosn, leftArmPosn;
int leftDrivePosn, rightDrivePosn;
// all Auton Behavior state specs
int firstDrive, secondDrive, RarmPreset,LarmPreset,speed_x,speed_y, backup;
int delay, lift_this_high, drop_this_low,  behavior_auton, completel, completer;
uint8_t data;
// all of these booleans are images of the limit microswitches. '1' is switch closed
bool lifttop, liftbottom, LarmL, LarmR, RarmL, RarmR, clawbottom, driveDone;
//int lastLiftCount,currentLiftCount;
//computed from 45*M_PI/180
#define sncs 0.707106781187;


class Robot: public IterativeRobot
{
	Joystick gamePad,leftStick/*,rightStick*/;
	LiveWindow *lw;
	int autoLoopCounter;
	PowerDistributionPanel PDB;
	ITable *hi;
	ITable *table;
	Gyro rateGyro;
	Talon fRight,fLeft,bRight,bLeft;
	VictorSP gearMotor,rightArm,leftArm, claw;
	RobotDrive robotDrive;
	Encoder liftPos,leftArmEncoder,rightArmEncoder,lDriveEncoder,rDriveEncoder;
	Accelerometer *accel;
	DigitalOutput arduinoReset;
	DigitalInput LeftLimit, RightLimit;

public:
	Robot() :
		gamePad(0),
		leftStick(1),
		//rightStick(2),
		lw(),
		autoLoopCounter(0),
		PDB(),
		hi(),
		table(),
		rateGyro(0),
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
		rDriveEncoder(0,1),
		arduinoReset(14),
		LeftLimit(10),
		RightLimit(11)
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

		accel = new BuiltInAccelerometer(Accelerometer::Range::kRange_4G);
		// setup values
		gmax=12.0;   //  in m/s^2
		gmin=8.0;
		Bmax=680.0;  //  in milligauss
		Bmin=410.0;
		pi = 4.0*atan(1.0);
	}
	void RobotPeriodic()
	{
		PDB.UpdateTable();
		liftPos.UpdateTable();
		rightArmEncoder.UpdateTable();
		leftArmEncoder.UpdateTable();
		lDriveEncoder.UpdateTable();
		rDriveEncoder.UpdateTable();
		/*
		// TEST LINES FOR ACCELEROMETER : can remove these in final code.
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
						tilt_theta = acos(zVal/gmag);
						tilt_phi   = atan(yVal/xVal);
						printf(" tilt theta %f3.1 and  phi %f3.3\n", 180.0*tilt_theta/pi, 180.0*tilt_phi/pi);
						// end get tilt, REMOVE WHEN DONE TESTING!
		 *
		 */
	}

	void AutonomousInit()
	{
		autoPeriodicLoops = 0;
		liftPos.Reset();
		rightArmEncoder.Reset();
		leftArmEncoder.Reset();
		lDriveEncoder.Reset();
		rDriveEncoder.Reset();
		//drive spec init
		//     Here driving forward for 'firstDrive' then picking up tote and driving for secondDrive' '
		firstDrive=48;
		behavior_auton=1; //lsb=drive1, bit2=pickup1, bit3=drive2, bit4=lower package, bit5=drive3
		speed_x = 50; //drive speed each side, for TANK DRIVE
		speed_y = 50;
		delay = 2;             //time delay spec
		completel = 0;         // flags for completion of arm motions to presets below.
		completer = 0;
		RarmPreset = 1200;     // right and left arm preset targets for first tote
		LarmPreset = 1350;
		lift_this_high = 1870; //  lift it run for secondDrive
		drop_this_low = 12;    //  height at which to release the tote
		secondDrive = 80;      // complete drive and lower and release tote
		backup = 13;           // then backup away from tote. any subsequent motion?
		// end of drive spec init
	}


	/*******************************/
	/*************AUTON*************/
	/*******************************/

	/*I commented out your code because I didn't have time to go through it and just wanted something quick.
	 * Also, I would like to make presets for the arms, as soon as the encoders are mounted.
	 * */

	//Since the drive encoders are 360 ppr, 1 pulse = 1 degree
	//I have no idea what this should be...
	#define AUTON_POS_DIF_THRESHOLD 5000
	#define AUTON_POS_SLOWDOWN_THRESHOLD 10000
	#define AUTON_POS_STOP_THRESHOLD 100
	//76 inches: 19 in/rev,360 pulses/rev
	//the 4 is arbitrary
	#define AUTON_POS_TARGET 360*40
	#define AUTON_LEFT_DRIVE_FAST .6
	#define AUTON_RIGHT_DRIVE_FAST .6
	#define AUTON_LEFT_DRIVE_SLOW .1
	#define AUTON_RIGHT_DRIVE_SLOW .1

	void AutonomousPeriodic()
	{
		int leftpos = -lDriveEncoder.GetRaw() ;
		int rightpos= rDriveEncoder.GetRaw() ;
		int diff = abs(leftpos - rightpos);
		int avgPos = (leftpos + leftpos)/2;
		if(diff < AUTON_POS_DIF_THRESHOLD)//encoders are close-ish
		{
			if(abs(avgPos - AUTON_POS_TARGET) >= AUTON_POS_SLOWDOWN_THRESHOLD)//pos is less than target, by a lot
			{
				robotDrive.TankDrive(AUTON_LEFT_DRIVE_FAST,AUTON_RIGHT_DRIVE_FAST);
			}
			else if((abs(avgPos - AUTON_POS_TARGET) < AUTON_POS_SLOWDOWN_THRESHOLD) && (abs(avgPos - AUTON_POS_TARGET) >= AUTON_POS_STOP_THRESHOLD))//pos is less than target, by a little
			{
				robotDrive.TankDrive(AUTON_LEFT_DRIVE_SLOW,AUTON_RIGHT_DRIVE_SLOW);
			}
			else if(abs(avgPos - AUTON_POS_TARGET) <= AUTON_POS_STOP_THRESHOLD)//pos is really close, stop
			{
				robotDrive.TankDrive(0.0,0.0);
			}
			else//HONEY, I THINK WE MISSED THE TURN
			{
				//TODO We can backup if we overshoot our target
			}
		}
		else//encoders are not close... correct before continuing
		{
			//TODO this will take time to do.
		}

	}
	void TeleopInit()
	{
		liftPos.Reset();
		rightArmEncoder.Reset();
		leftArmEncoder.Reset();
		lDriveEncoder.Reset();
		rDriveEncoder.Reset();
		data = 0 ;
		maxcurrent = 0;
	}
	void TeleopPeriodic()
	{
		arduinoReset.Set(1);

		//ArduinoRW(0xC);
		float x = .85;

		lax = gamePad.GetRawAxis(lXA);
		lay = gamePad.GetRawAxis(lYA);
		rax = gamePad.GetRawAxis(rXA);
		ray = gamePad.GetRawAxis(rYA);


		#ifdef SRC_GAMEPADX_H_
		lat = gamePad.GetRawAxis(lTA);
		rat = gamePad.GetRawAxis(rTA);
		gamePad.SetRumble(gamePad.kLeftRumble,rat);
		#endif

		lsz = leftStick.GetZ();
		lsx = leftStick.GetX();
		lsy = -leftStick.GetY();

		/*Threshholds*/
		const float GPthresh = .9;
		const float Jthresh = .1;

		/*
		if(abs(lax) < GPthresh){
			lax = 0;
		}
		if(abs(ray) < GPthresh){
			ray = 0;
		}
*/

		/*tilt*/

		double xVal = accel->GetX();
		double yVal = accel->GetY();
		double zVal = accel->GetZ();

		LeftLim = LeftLimit.Get();
		RightLim = RightLimit.Get();

		if ((lay>0)&&(!lifttop)&&!gamePad.GetRawButton(RB))
		{
			gearMotor.Set(-lay);
		}
		if ((lay<0)&&(!liftbottom)&&!gamePad.GetRawButton(RB))
		{
			gearMotor.Set(-lay);
		}
		if ((lax>0)&&(!LarmL)&&!gamePad.GetRawButton(LB))
		{
			leftArm.Set(-lax);
		}
		if ((lax<0)&&(!LarmR)&&!gamePad.GetRawButton(LB))
		{
			leftArm.Set(-lax);
		}
		if ((rax>0)&&(!RarmR)&&!gamePad.GetRawButton(LB))
		{
			rightArm.Set(-rax);
		}
		if ((rax<0)&&(!RarmR)&&!gamePad.GetRawButton(LB))
		{
			rightArm.Set(-rax);
		}
		if ((rax>0)&&(!RarmR))
		{
			//rightArm.Set(rax);
		}
		if ((lax<0)&&(!LarmL))
		{
		//	leftArm.Set(lax);
		}
		// determine tilt



/*
		gmag = xVal*xVal+yVal*yVal+zVal*zVal;
		if ((gmag<gmax*gmax)&&(gmag>gmin*gmin)) { //validate
			gmag = pow(gmag,0.5);
			tilt_theta = acos(zVal/gmag);
			tilt_phi   = atan(yVal/xVal);
		}
		tilt_theta = acos(zVal/gmag);
		tilt_phi   = atan(yVal/xVal);
		// end get tilt
		// compute heading
		Bmag = Bx*Bx+By*By+Bz*Bz;
		if ((Bmag<Bmax*Bmax)&&(Bmag>Bmin*Bmin)){ //validate
			Bmag = Bx;
			Bx = cos(tilt_phi)*Bmag + sin(tilt_phi)*(cos(tilt_theta)*By+sin(tilt_theta)*Bz);
			By = -sin(tilt_phi)*Bmag + cos(tilt_phi)*(cos(tilt_theta)*By+sin(tilt_theta)*Bz);
			heading = 180.0*atan(By/Bx)/pi;
		}
	*/
		// end compute heading


		if(gamePad.GetRawButton(X)){ }
		if(gamePad.GetRawButton(A)){ }
		if(gamePad.GetRawButton(B)){ }
		if(gamePad.GetRawButton(Y)){ }
		if(gamePad.GetRawButton(LB)){ }
		if(gamePad.GetRawButton(RB)){ }
		if(gamePad.GetRawButton(Sel)){ }
		if(gamePad.GetRawButton(Sta)){arduinoReset.Set(0);}
		if(gamePad.GetRawButton(lStick)){ }
		if(gamePad.GetRawButton(rStick)){ }

		if(!leftStick.GetButton(leftStick.kTriggerButton)){lsz = 0;}
		if(leftStick.GetRawButton(2)){x = .4;}

		#ifdef SRC_GAMEPADD_H_
		if(gamePad.GetRawButton(Lt)){ }
		if(gamePad.GetRawButton(Rt)){ }
		#endif

		#ifdef MECHANUM
			//strafe, rotate, forwardback
			robotDrive.MecanumDrive_Cartesian(-lsx*x,-lsz*x,lsy*x);
		#endif

		#ifdef TANK
			robotDrive.TankDrive(-leftStick.GetY(),-rightStick.GetY());
		#endif

		currentcurrent = PDB.GetCurrent(14);

		if(currentcurrent > maxcurrent)
		{
			maxcurrent = currentcurrent;
		}

		SmartDashboard::PutNumber("PDB Temp",(float)PDB.GetTemperature());
		SmartDashboard::PutNumber("PDB Current Lift",PDB.GetCurrent(2));
		SmartDashboard::PutNumber("PDB Current Left Arm",PDB.GetCurrent(3));
		SmartDashboard::PutNumber("PDB Lift Current",maxcurrent);

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

		SmartDashboard::PutNumber("tilt theta", 180.0*tilt_theta/pi);
		SmartDashboard::PutNumber("tilt phi", 180.0*tilt_phi/pi);
		SmartDashboard::PutNumber("accel. magnitude", gmag);

		SmartDashboard::PutNumber("POV Number",leftStick.GetPOV());
		SmartDashboard::PutNumber("Button Number",leftStick.GetButton(leftStick.kNumButtonTypes));
		SmartDashboard::PutBoolean("R arm Limit", RightLim);
		SmartDashboard::PutBoolean("L arm limit",LeftLim);


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
		// parse switch state
		switches = (int)(buffer[1]);
		lifttop = switches & 1;
		liftbottom = switches & 2;
		RarmR = switches & 4;
		RarmL = switches & 8;
		LarmR = switches & 16;
		LarmR = switches & 32;
		clawbottom = switches & 64;
	}
	void getHeading()
	{
		I2C Wire(I2C::kOnboard, 0x1E);
		Wire.Write(0x02,0x00);
		//Wire.Write(0xA,payload);
		try{
			uint8_t *bufferz[7];
			if(!Wire.Read(0x03,6, *bufferz)){
				counti2c++;
				for(int q=0;q <=6;q++){
					printf("byte:%i data:%X\n",q,*bufferz[q]);
				}
			}
		}
		catch(int e){
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
Expansion Digital I/O                     |
10 Claw closed limit IN                   |
11 R. Arm capacitive sync IN              |
12 L. Arm capacitive sync IN              |
13                                        |
14 Arduino reset line     OUT             |
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
