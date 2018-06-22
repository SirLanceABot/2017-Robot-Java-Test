package org.usfirst.frc.team4237.robot.components;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.usfirst.frc.team4237.robot.Util;

import com.ctre.CANTalon;

//import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;

public class Climber
{
	private WPI_TalonSRX masterMotor = new WPI_TalonSRX(Constants.Ports.CLIMBER_MASTER);
	private WPI_TalonSRX followerMotor = new WPI_TalonSRX(Constants.Ports.CLIMBER_SLAVE);
	private Timer mAmpTimer = new Timer();

	private static Climber instance = new Climber();
	public static Climber getInstance()
	{
		return instance;
	}

	private Climber()
	{

		masterMotor.setNeutralMode(NeutralMode.Brake);
		followerMotor.follow(masterMotor);

		masterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		masterMotor.setSelectedSensorPosition(0, 0, 0);

		mAmpTimer.stop();
		mAmpTimer.reset();
	}
	
	public void climbRope(double speed)
	{
		double masterCurrent = masterMotor.getOutputCurrent();
		double slaveCurrent = followerMotor.getOutputCurrent();
		double masterVoltage = masterMotor.getBusVoltage();
		double slaveVoltage = followerMotor.getBusVoltage();
		
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
		masterMotor.set(-speed);
	}
	
	public int getEncoder()
	{
		return -masterMotor.getSelectedSensorPosition(0);
	}
	
	public void stopClimbing()
	{
		masterMotor.set(0);
	}

	public static class Constants
	{
		public class Ports
		{
			public static final int CLIMBER_MASTER = 32;
			public static final int CLIMBER_SLAVE = 33;
		}
	}
}
