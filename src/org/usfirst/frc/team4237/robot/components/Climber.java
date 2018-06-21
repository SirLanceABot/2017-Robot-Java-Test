package org.usfirst.frc.team4237.robot.components;

import org.usfirst.frc.team4237.robot.Util;

import com.ctre.CANTalon;

//import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;

public class Climber
{
	private CANTalon mMasterClimber;
	private CANTalon mSlaveClimber;
	private Timer mAmpTimer;
	
	public class Ports
	{
		public static final int CLIMBER_MASTER = 32;
		public static final int CLIMBER_SLAVE = 33;
	}
	
	public Climber(int masterClimber, int slaveClimber)
	{
		Util.printObjectInfo(this);
		mMasterClimber = new CANTalon(masterClimber);
		mSlaveClimber = new CANTalon(slaveClimber);
		
		mMasterClimber.enableBrakeMode(true);
		mSlaveClimber.enableBrakeMode(true);
		mSlaveClimber.changeControlMode(CANTalon.TalonControlMode.Follower);
		mSlaveClimber.set(masterClimber);
		mMasterClimber.setEncPosition(0);
		
		mMasterClimber.setStatusFrameRateMs(CANTalon.StatusFrameRate.AnalogTempVbat, 10);
		mSlaveClimber.setStatusFrameRateMs(CANTalon.StatusFrameRate.AnalogTempVbat, 10);
		
		mMasterClimber.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 10);
		mSlaveClimber.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 10);
		
		mAmpTimer = new Timer();
		mAmpTimer.stop();
		mAmpTimer.reset();
	}
	
	public void climbRope(double speed)
	{
		double masterCurrent = mMasterClimber.getOutputCurrent();
		double slaveCurrent = mSlaveClimber.getOutputCurrent();
		double masterVoltage = mMasterClimber.getBusVoltage();
		double slaveVoltage = mSlaveClimber.getBusVoltage();
		
		System.out.println("Master Amperage: " + masterCurrent);
		System.out.println(" Master Voltage: " + masterVoltage);
		System.out.println(" Slave Amperage: " + slaveCurrent);
		System.out.println("  Slave Voltage: " + slaveVoltage);
		System.out.println("      Amp Timer: " + mAmpTimer.get());
		
		if (masterCurrent < 50.0 && slaveCurrent < 50.0)
		{
			mAmpTimer.stop();
			mAmpTimer.reset();
		}
		else if ((masterCurrent >= 50.0 || slaveCurrent >= 50.0) && mAmpTimer.get() > 0.2)
		{
			System.out.println("Amperage Timer has been exceeded");
		}
		else if (masterCurrent >= 50.0 || slaveCurrent >= 50.0)
		{
			mAmpTimer.start();
		}
		mMasterClimber.set(-speed);
	}
	
	public int getEncoder()
	{
		return -mMasterClimber.getEncPosition();
	}
	
	public void stopClimbing()
	{
		mMasterClimber.set(0);
	}
}
