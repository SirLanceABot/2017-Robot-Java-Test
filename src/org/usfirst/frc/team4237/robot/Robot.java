package org.usfirst.frc.team4237.robot;

import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team4237.robot.components.Climber;
import org.usfirst.frc.team4237.robot.components.DriveTrain;
import org.usfirst.frc.team4237.robot.components.GearBox;
import org.usfirst.frc.team4237.robot.components.GearHolder;
import org.usfirst.frc.team4237.robot.components.LightRing;
import org.usfirst.frc.team4237.robot.control.DriverXbox;
import org.usfirst.frc.team4237.robot.control.Xbox;

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
	public static final int NUM_TAPS = 20;

	private DriverXbox xbox = DriverXbox.getInstance();
	private DriveTrain drivetrain = DriveTrain.getInstance();
	private GearHolder gearHolder = GearHolder.getInstance();
	private Climber climber = Climber.getInstance();
	private Vision vision;
	private DriverStation driverStation = DriverStation.getInstance();
	private LightRing lightRing = LightRing.getInstance();
	private Compressor compressor = new Compressor();


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

	private LightRing.Constants.Voltage lightIntensity = LightRing.Constants.Voltage.k135v;
	private LightRing.Constants.Voltage previousLightIntensity = LightRing.Constants.Voltage.k135v;

	int loop = 1;
	double move;
	double rotate;

	public Robot()
	{

		vision = new Vision();

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
			move = -xbox.getRawAxis(Xbox.Constants.LEFT_STICK_Y_AXIS);
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


			if (Math.abs(xbox.getRawAxis(Xbox.Constants.RIGHT_TRIGGER_AXIS)) > 0.1)
			{
				System.out.println("Climbing");
				compressor.stop();
				climber.climbRope(Math.abs(xbox.getRawAxis(Xbox.Constants.RIGHT_TRIGGER_AXIS)));
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
		lightRing.setIntensity(LightRing.Constants.Voltage.k135v);
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
		Timer.delay(0.1);
		autoDrive();
		
		//TODO Stop vision thread here
		
		Timer.delay(0.5);
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
		lightRing.setIntensity(LightRing.Constants.Voltage.k135v);
		
		drivetrain.resetEncoders();
		drivetrain.makeIsDoneDrivingTrue();
		drivetrain.resetGyro();
		
		switch(gearPosition)
		{
		case kLeft:
			drivetrain.driveDistance(Autonomous.SIDE_SPEED, 70);
			drivetrain.spinCW(Autonomous.TURN_SPEED, 55);
			Timer.delay(0.5);
			runVision();
			drivetrain.stopDriving();
			drivetrain.makeIsDoneDrivingTrue();
			drivetrain.resetEncoders();
			
			while (gearHolder.dropGearSequence()){}
			drivetrain.driveDistance(Autonomous.BACKWARD_SPEED, 12.0);
			gearHolder.close();
			
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
}