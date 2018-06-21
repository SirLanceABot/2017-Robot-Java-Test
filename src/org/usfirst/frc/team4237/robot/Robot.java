package org.usfirst.frc.team4237.robot;

import org.usfirst.frc.team4237.robot.components.Climber;
import org.usfirst.frc.team4237.robot.components.DriveTrain;
import org.usfirst.frc.team4237.robot.components.GearBox;
import org.usfirst.frc.team4237.robot.components.GearHolder;
import org.usfirst.frc.team4237.robot.components.LightRing;
import org.usfirst.frc.team4237.robot.control.DriverXbox;
import org.usfirst.frc.team4237.robot.control.Xbox;
import org.usfirst.frc.team4237.robot.sensors.ITG3200;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * Main robot class
 * @author Mark Washington
 *
 */
public class Robot extends IterativeRobot
{
	enum VisionStage {kNoTargetFound, kHalfTarget, kFullTarget, kDriveForward, kRelease, kDone, kNothing};
	enum ExposureMode {kUp, kDown};

	//TODO: Complete this enum
	enum AutoGearChoice 
	{
		kLeft(3), kCenter(1), kRight(2), kNone(4);
		private final int num;
		private AutoGearChoice(int num)
		{
			this.num = num;
		}
	}

	//TODO: Complete this enum

	enum AutoBoilerChoice
	{
		kBoilerLeft(3), kBoilerRight(2);
		private final int num;
		private AutoBoilerChoice(int num)
		{
			this.num = num;
		}
	}

	public static final I2C.Port SIX_DOF = I2C.Port.kMXP;
	public static final double SENSITIVITY = ITG3200.Constants.SENSITIVITY_SCALE_FACTOR;
	public static final int NUM_TAPS = 20;

	private DriverXbox xbox;
	private DriveTrain drivetrain;
	private GearHolder gearHolder;
	private Climber climber;
	private Vision vision;
	private Compressor compressor;
	private DriverStation driverStation;
	private LightRing lightRing;

	private VisionStage visionStage;
	private AutoGearChoice gearPosition;
	private AutoBoilerChoice boilerPosition;

	private double hueLow = 51.0;
	private double hueHigh = 99.0;
	private double saturationLow = 198.0;
	private double saturationHigh = 255.0;
	private double luminanceLow = 40.0;
	private double luminanceHigh = 113.0;
	private int batteryID = 0;
	private boolean isAutoShooting = false;
	private boolean isDroppingGear = false;
	private ExposureMode exposureMode = ExposureMode.kDown;

	private LightRing.Voltage lightIntensity = LightRing.Voltage.k135v;
	private LightRing.Voltage previousLightIntensity = LightRing.Voltage.k135v;

	int loop = 1;
	double move;
	double rotate;

	public Robot()
	{
		drivetrain = new DriveTrain(
				DriveTrain.Ports.LEFT_DRIVE_MASTER,
				DriveTrain.Ports.LEFT_DRIVE_SLAVE_1,			
				DriveTrain.Ports.RIGHT_DRIVE_MASTER,
				DriveTrain.Ports.RIGHT_DRIVE_SLAVE_1,
				GearBox.Ports.HIGH_SPEED,
				GearBox.Ports.LOW_SPEED,
				SIX_DOF,
				SENSITIVITY
				);

		xbox = DriverXbox.getInstance();

		gearHolder = GearHolder.getInstance();

		climber = new Climber(Climber.Ports.CLIMBER_MASTER, Climber.Ports.CLIMBER_SLAVE);
		vision = new Vision();
		compressor = new Compressor();
		driverStation = DriverStation.getInstance();
		lightRing = new LightRing(
				LightRing.Ports.ON_OFF,
				LightRing.Ports.FOURS,
				LightRing.Ports.TWOS,
				LightRing.Ports.ONES
				);

		compressor.start();
	}

	public void robotInit()
	{
		
	}

	@Override
	public void teleopInit() //Replaces what was "OperatorControl()" in the original code
	{
		System.out.println("Entering Teleop");

		//TODO: Change this to a function
		exposureMode = ExposureMode.kUp;
		lightRing.turnLightsOff();
		drivetrain.setSafetyEnabled(false);
		drivetrain.setRampRate(120);
	}

	public void teleopPeriodic() //Replaces what was "Operator()" in the original code
	{	

		//A giant try-catch loop is the new way to do things
		try
		{
			updateXbox();

			move = xbox.getRawAxis(Xbox.Constants.LEFT_STICK_Y_AXIS);
			rotate = xbox.getRawAxis(Xbox.Constants.RIGHT_STICK_X_AXIS);

			if (Math.abs(move) < 0.05)
			{
				move = 0.0;
			}
			if (Math.abs(rotate) < 0.05)
			{
				rotate = 0.0;
			}

			drivetrain.arcadeDrive(move, rotate, true);

			//LEFT BUMPER - Controls shifting
			//Driver can only shift once per press

			if (xbox.getRawButtonPressed(Xbox.Constants.LEFT_BUMPER))
			{
				System.out.println("Shifting");
				drivetrain.shiftSpeed();
			}


			if (xbox.isTriggerPressed(XboxJoystick.RIGHT_TRIGGER))
			{
				System.out.println("Climbing");
				compressor.stop();
				climber.climbRope(xbox.getTrigger(XboxJoystick.RIGHT_TRIGGER));
			}
			else
			{
				compressor.start();
				climber.stopClimbing();
			}
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}

		loop++;
		//System.out.println("Loop: " + loop);
	}

	class Autonomous
	{
		static final double FINAL_SPEED = 0.8;
		static final double SIDE_SPEED = 0.8;
		static final double TURN_SPEED = 0.7;
		static final double BACKWARD_SPEED = -0.7;
		static final double VISION_SPEED = 0.4;
	}
	
	public void autonomousInit()
	{
		System.out.println("Entered Autonomous");
		System.out.println("Battery Voltage: " + driverStation.getBatteryVoltage());
		exposureMode = ExposureMode.kDown;
		lightRing.turnLightRingOn();
		lightRing.setIntensity(LightRing.Voltage.k135v);
		vision.visionThread();
		
		switch(driverStation.getAlliance())
		{
		case Red:
			System.out.println("Alliance Color: Red");
			break;
		case Blue:
			System.out.println("Alliance Color: Blue");
			break;
		case Invalid:
			System.out.println("Alliance Color: Invalid");
			break;
		}
		
		System.out.println("Driver Station: " + driverStation.getLocation());
		//System.out.println("Gear Position: " + mAutoSelect4237.getGearPosition2017());
		//System.out.println("Boiler Position: " + mAutoSelect4237.getBoilerPosition2017());
	
		drivetrain.setSafetyEnabled(false);
		drivetrain.setControlModeVoltage();
		drivetrain.downShift();
		Util.wait(0.1);
		autoDrive();
		
		//TODO Stop vision thread here
		
		Util.wait(0.5);
		drivetrain.upShift();
	}

	/**
	 * Just loops after vision and the rest of autonomous runs, it does nothing.
	 */
	public void autonomousPeridioc()
	{
		
	}

	/**
	 * Driving for autonomous (going forward, turning).
	 */
	public void autoDrive()
	{
		double angle;
		vision.setExposureLow();
		
		//TODO Reset Pi Timer
		//vision.resetPiTimer();
		
		lightRing.turnLightRingOn();
		lightRing.setIntensity(LightRing.Voltage.k135v);
		
		drivetrain.resetEncoders();
		drivetrain.makeIsDoneDrivingTrue();
		drivetrain.resetGyro();
		
		switch(gearPosition)
		{
		case kLeft:
			drivetrain.driveDistance(Autonomous.SIDE_SPEED, 70);
			drivetrain.spinCW(Autonomous.TURN_SPEED, 55);
			Util.wait(0.2);
			runVision();
			drivetrain.stopDriving();
			drivetrain.makeIsDoneDrivingTrue();
			drivetrain.resetEncoders();
			
			while (mGearHolder.dropGearSequence()){}
			drivetrain.driveDistance(Autonomous.BACKWARD_SPEED, 12.0);
			mGearHolder.close();
			
			if (drivetrain.isGyroWorking())
			{
				angle = Math.abs(drivetrain.getZ());
				drivetrain.spinCCW(Autonomous.TURN_SPEED, angle - 5);
			}
			else
			{
				System.out.println("ERROR: Gyro not working");
				drivetrain.spinCCW(Autonomous.TURN_SPEED, 45);
			}
			
			//switch()
		}
	}

	public void visionMain()
	{
		boolean isUpdateGood;
		int numTargets;
		drivetrain.stopDriving();
		isUpdateGood = vision.update();
		
		if (isUpdateGood)
		{
			numTargets = vision.getNumTargets();
		}
		else
		{
			numTargets = 0;
		}
		
		if (numTargets == 0)
		{
			System.out.println("[Vision]: No targets found in auto");
			visionStage = VisionStage.kNoTargetFound;
			while (driverStation.isAutonomous() && driverStation.isEnabled() && (numTargets <= 0))
			{
				System.out.println("[Vision]: Trying again after not finding targets");
				isUpdateGood = vision.update();
				if (isUpdateGood)
				{
					numTargets = vision.getNumTargets();
				}
			}
			System.out.println("[Vision]: Found a target, moving to next stage");
		}
		
		if (numTargets == 1)
		{
			System.out.println("[Vision]: Half of a target found");
			visionStage = VisionStage.kFullTarget;
		}
		else
		{
			System.out.println("[Vision]: Full target found in auto");
			visionStage = VisionStage.kFullTarget;
		}
	}

	void runVision()
	{
		System.out.println("Starting Vision");
		if (driverStation.isAutonomous() && driverStation.isEnabled())
		{
			//TODO: Pi code
			//vision.startPiTimer();
			visionMain();
		}
		System.out.println("Done running vision");
	}

	void dropGear()
	{

	}

	void closeGearHolder()
	{

	}

	/**
	 * These replace systemCheck()
	 */
	public void testInit()
	{
		System.out.println("Entering test mode");
	}

	public void testPeriodic()
	{
		updateXbox();
		System.out.println("Left bumper previous state: " + this.previousButtonMap.get(XboxJoystick.LEFT_BUMPER));
		System.out.println("Left bumper current state: " + xbox.getRawButton(XboxJoystick.LEFT_BUMPER));
		if (xbox.getRawButton(XboxJoystick.LEFT_BUMPER) == true && this.previousButtonMap.get(XboxJoystick.LEFT_BUMPER) == false)
		{
			System.out.println("Shifting");
		}
	}
	
	public void updateXbox()
	{
		for(int i : XboxJoystick.BUTTON_ARRAY)
		{
			previousButtonMap.put(i, buttonMap.get(i));
			buttonMap.put(i, xbox.getRawButton(i));
		}
	}
}