package org.usfirst.frc.team4237.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain extends DifferentialDrive
{
	private static WPI_TalonSRX leftMasterMotor = new WPI_TalonSRX(Constants.Ports.LEFT_MASTER_MOTOR_PORT);;
	private static WPI_TalonSRX leftFollowerMotor = new WPI_TalonSRX(Constants.Ports.LEFT_FOLLOWER_MOTOR_PORT);;
	
	private static WPI_TalonSRX rightMasterMotor = new WPI_TalonSRX(Constants.Ports.RIGHT_MASTER_MOTOR_PORT);;
	private static WPI_TalonSRX rightFollowerMotor = new WPI_TalonSRX(Constants.Ports.RIGHT_FOLLOWER_MOTOR_PORT);;

	private GearBox gearBox = new GearBox(GearBox.Constants.Ports.LOW_SPEED, GearBox.Constants.Ports.HIGH_SPEED);

	private boolean isDoneDriving = false;


    private static DriveTrain instance = new DriveTrain();
	public static DriveTrain getInstance()
    {
        return instance;
    }

	private DriveTrain()
	{
		super(leftMasterMotor, rightMasterMotor);

		leftFollowerMotor.follow(leftMasterMotor);
		rightFollowerMotor.follow(rightMasterMotor);

		rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightMasterMotor.setInverted(false);
		//FIXME: Problems porting below line to Phoenix
		//rightMasterMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 8);

		leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		leftMasterMotor.setInverted(false);
		//FIXME: Problems porting below line to Phoenix
		//leftMasterMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 8);

		resetEncoders();
		
		rightMasterMotor.setNeutralMode(NeutralMode.Brake);
		leftMasterMotor.setNeutralMode(NeutralMode.Brake);
	}

	public void shiftSpeed()
	{
		if (gearBox.getGearPosition() == GearBox.Constants.GearPosition.kLow)
		{
			System.out.println("Retracting gearbox");
			gearBox.retract();
		}
		else if (gearBox.getGearPosition() == GearBox.Constants.GearPosition.kHigh || gearBox.getGearPosition() == GearBox.Constants.GearPosition.kOff)
		{
			System.out.println("Extending gearbox");
			gearBox.extend();
		}
		else if (gearBox.getGearPosition() == GearBox.Constants.GearPosition.kError)
		{
			System.out.println("Error shifting");
		}
	}
	
	public void downShift()
	{
		gearBox.extend();
	}
	
	public void upShift()
	{
		gearBox.retract();
	}
	
	public boolean spinCW(double speed, double angle)
	{
		if (isDoneDriving())
		{
			notDoneDriving();
			resetEncoders();
			Timer.delay(0.1);
			arcadeDrive(0, speed);
		}
		else if (getDistance() >= (angle / 360.0) * Constants.WHEEL_BASE_CIRCUMFERENCE);
		{
			stopDriving();
			resetEncoders();
			Timer.delay(0.1);
			doneDriving();
		}
		return isDoneDriving();
	}
	
	public void spinCCW(double speed, double angle)
	{
		
	}
	
	public void newSpinCW(double speed, double angle)
	{
		
	}
	
	public void newSpinCCW(double speed, double angle)
	{
		
	}
	
	public boolean driveDistance(double speed, double distance)
	{
		if (isDoneDriving())
		{
			notDoneDriving();
			resetEncoders();
			Timer.delay(0.1);
			arcadeDrive(speed, 0.0);
		}
		else if (getDistance() >= distance)
		{
			stopDriving();
			resetEncoders();
			Timer.delay(0.1);
			doneDriving();
		}
		return isDoneDriving();
	}
	
	public double getDistance()
	{
		//25.2627908 is ticks per inch driven
		//(360 * 4 / 4) / (6 * pi) = 25.2627908
		
		int enc = getLeftEncoder();
		if (enc < 0)
		{
			return (double)-enc / Constants.TICKS_PER_INCH;
		}
		else
		{
			return (double)enc / Constants.TICKS_PER_INCH;
		}
	}
	
	public void resetEncoders()
	{
		rightMasterMotor.setSelectedSensorPosition(0, 0, 10);
		leftMasterMotor.setSelectedSensorPosition(0, 0, 10);
	}
	
	public void stopDriving()
	{
		arcadeDrive(0.0, 0.0);
	}
	
	public int getLeftEncoder()
	{
		return leftMasterMotor.getSelectedSensorPosition(0) * -1;
	}
	
	public int getRightEncoder()
	{
		return rightMasterMotor.getSelectedSensorPosition(0);
	}

	public void makeIsDoneDrivingTrue()
	{
		isDoneDriving = true;
	}
	
	public boolean newDriveDistance(double speed, double distance)
	{
		if (isDoneDriving())
		{
			doneDriving();
			resetEncoders();
			arcadeDrive(speed, 0.0);
			System.out.println("Drove: " + distance + "\"");
		}
		else if (Math.abs(leftMasterMotor.getSelectedSensorPosition(0)) >= distance || Math.abs(rightMasterMotor.getSelectedSensorPosition(0)) >= distance)
		{
			stopDriving();
			resetEncoders();
			doneDriving();
		}
		else
		{
			arcadeDrive(0.0, 0.0);
		}
		return isDoneDriving();
	}
	
	public boolean getGearPosition()
	{
		if (this.gearBox.getGearPosition() == GearBox.Constants.GearPosition.kLow)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
	
	public boolean isGyroWorking()
	{
		return false;
	}
	
	public boolean isDoneDriving()
	{
		return this.isDoneDriving;
	}
	
	public void doneDriving()
	{
		this.isDoneDriving = true;
	}
	
	public void notDoneDriving()
	{
		this.isDoneDriving = false;
	}
	
	public void setRampRate(int rate)
	{
		leftMasterMotor.configClosedloopRamp(rate, 10);
		rightMasterMotor.configClosedloopRamp(rate, 10);
	}
	
	public void setGyroAngle(double angleX, double angleY, double angleZ)
	{
		double[] angle = {angleX, angleY, angleZ};
	}
	
	public double getZ()
	{
		return 0.0;
	}
	
	public void resetGyro()
	{
		this.setGyroAngle(0.0, 0.0, 0.0);
	}
	
	public void setControlModeVoltage()
	{
		leftMasterMotor.set(ControlMode.Current, 0.0);
		rightMasterMotor.set(ControlMode.Current, 0.0);
	}
	
	public void setControlMovePercentVBus()
	{
		leftMasterMotor.set(ControlMode.PercentOutput, 0.0);
        rightMasterMotor.set(ControlMode.PercentOutput, 0.0);
    }

	public static class Constants
	{
		public enum ShifterPosition {kLow, kHigh};

		private static final double WHEEL_BASE_CIRCUMFERENCE = 23.75 * 3.1415926;
		private static final double WHEEL_CIRCUMFERENCE = 6.0 * 3.1415926;
		private static final double TICKS_PER_INCH = (360.0 * 4.0 / 3.0) / WHEEL_CIRCUMFERENCE;
		private static final double ENCODER_CODES_PER_REVOLUTION = (360.0 / 3.0) / WHEEL_CIRCUMFERENCE;

		public static class Ports
		{
			public static final int LEFT_FOLLOWER_MOTOR_PORT = 21;
			public static final int LEFT_MASTER_MOTOR_PORT = 22;
			public static final int RIGHT_FOLLOWER_MOTOR_PORT = 34;
			public static final int RIGHT_MASTER_MOTOR_PORT = 35;
		}
	}
}