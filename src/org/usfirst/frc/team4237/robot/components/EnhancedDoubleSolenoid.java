package org.usfirst.frc.team4237.robot.components;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class EnhancedDoubleSolenoid extends DoubleSolenoid
{
	public EnhancedDoubleSolenoid(int extendPort, int retractPort)
	{
		super(extendPort, retractPort);
	}
	
	public void extend()
	{
		this.set(DoubleSolenoid.Value.kForward);
	}
	
	public void retract()
	{
		this.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void off()
	{
		this.set(DoubleSolenoid.Value.kOff);
	}
	
	public DoubleSolenoid.Value getPosition()
	{
		System.out.println(this.get());
		return this.get();
	}
}
