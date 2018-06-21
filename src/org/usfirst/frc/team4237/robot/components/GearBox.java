package org.usfirst.frc.team4237.robot.components;

import org.usfirst.frc.team4237.robot.Util;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class GearBox extends EnhancedDoubleSolenoid
{
	public enum GearPosition {kLow, kHigh, kOff, kError};
	public class Ports
	{
		public static final int LOW_SPEED = 0;
		public static final int HIGH_SPEED = 1;
	}
	
	public GearBox(int extendPort, int retractPort)
	{
		super(extendPort, retractPort);
		Util.printObjectInfo(this);
	}
	
	public GearPosition getGearPosition()
	{
		if (this.getPosition() == DoubleSolenoid.Value.kForward)
		{
			return GearPosition.kLow;
		}
		else if (this.getPosition() == DoubleSolenoid.Value.kReverse)
		{
			return GearPosition.kHigh;
		}
		else if (this.getPosition() == DoubleSolenoid.Value.kOff)
		{
			return GearPosition.kOff;
		}
		else return GearPosition.kError;
	}
}
