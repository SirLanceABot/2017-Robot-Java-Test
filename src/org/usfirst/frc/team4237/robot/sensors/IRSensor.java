package org.usfirst.frc.team4237.robot.sensors;
import edu.wpi.first.wpilibj.AnalogInput;

public class IRSensor extends AnalogInput
{
	public IRSensor(int port)
	{
		super(port);
	}
	
	public double getDistance()
	{
		return 21521.2959 * Math.pow(this.getAverageValue(), -1.273);
	}
}
