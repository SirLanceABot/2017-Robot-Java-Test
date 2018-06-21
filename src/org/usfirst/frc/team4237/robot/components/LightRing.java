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
	public enum Voltage {k068v, k084v, k116v, k132v, k135v};
	private DigitalOutput mOnOffPin;
	private DigitalOutput mOnes;
	private DigitalOutput mTwos;
	private DigitalOutput mFours;

	private boolean[] pins = {false, false, false};
	
	public class Ports
	{
		public static final int ON_OFF = 10;
		public static final int ONES = 12;
		public static final int TWOS = 13;
		public static final int FOURS = 14;
	}

	public LightRing(int onOffPort, int foursPort, int twosPort, int onesPort)
	{
		mOnOffPin = new DigitalOutput(onOffPort);
		mFours = new DigitalOutput(foursPort);
		mTwos = new DigitalOutput(twosPort);
		mOnes = new DigitalOutput(onesPort);
		Util.printObjectInfo(this);
	}

	public void setIntensity(Voltage level)
	{
		pins[0] = ((level.ordinal() / 4) % 2) == 1; //Value for pin 14
		pins[1] = ((level.ordinal() / 2) % 2) == 1; //Value for pin 13
		pins[2] = (level.ordinal() % 2) == 1;

		mFours.set(pins[0]);
		mTwos.set(pins[1]);
		mOnes.set(pins[2]);
	}

	public void turnLightRingOn()
	{
		mOnOffPin.set(true);
	}

	public void turnLightsOff()
	{
		mOnOffPin.set(false);
	}
}