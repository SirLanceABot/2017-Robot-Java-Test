package org.usfirst.frc.team4237.robot.components;

import org.usfirst.frc.team4237.robot.Util;

import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * Class to control the variable-intensity light ring
 * @author mark
 *
 */
public class LightRing
{
	private DigitalOutput onOffPin = new DigitalOutput(Constants.Ports.ON_OFF);
	private DigitalOutput onesPin = new DigitalOutput(Constants.Ports.ONES);
	private DigitalOutput twosPin = new DigitalOutput(Constants.Ports.TWOS);
	private DigitalOutput foursPin = new DigitalOutput(Constants.Ports.FOURS);

	private boolean[] pins = {false, false, false};

	private static LightRing instance = new LightRing();
	public static LightRing getInstance()
    {
        return instance;
    }

	public void setIntensity(Constants.Voltage level)
	{
		pins[0] = ((level.ordinal() / 4) % 2) == 1; //Value for pin 14
		pins[1] = ((level.ordinal() / 2) % 2) == 1; //Value for pin 13
		pins[2] = (level.ordinal() % 2) == 1;

		foursPin.set(pins[0]);
		twosPin.set(pins[1]);
		onesPin.set(pins[2]);
	}

	public void turnLightRingOn()
	{
		onOffPin.set(true);
	}

	public void turnLightsOff()
	{
		onOffPin.set(false);
	}

	public static class Constants
	{

		public enum Voltage {k068v, k084v, k116v, k132v, k135v};

		public class Ports
		{
			public static final int ON_OFF = 10;
			public static final int ONES = 12;
			public static final int TWOS = 13;
			public static final int FOURS = 14;
		}
	}
}