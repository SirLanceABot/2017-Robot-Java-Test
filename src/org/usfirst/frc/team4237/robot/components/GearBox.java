package org.usfirst.frc.team4237.robot.components;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class GearBox extends EnhancedDoubleSolenoid
{
	public GearBox(int extendPort, int retractPort)
	{
		super(extendPort, retractPort);
	}
	
	public Constants.GearPosition getGearPosition()
	{
		if (this.getPosition() == DoubleSolenoid.Value.kForward)
		{
			return Constants.GearPosition.kLow;
		}
		else if (this.getPosition() == DoubleSolenoid.Value.kReverse)
		{
			return Constants.GearPosition.kHigh;
		}
		else if (this.getPosition() == DoubleSolenoid.Value.kOff)
		{
			return Constants.GearPosition.kOff;
		}
		else return Constants.GearPosition.kError;
	}

	public static class Constants
	{
		public enum GearPosition {kLow, kHigh, kOff, kError};

		public class Ports
		{
			public static final int LOW_SPEED = 0;
			public static final int HIGH_SPEED = 1;
		}
	}
}
