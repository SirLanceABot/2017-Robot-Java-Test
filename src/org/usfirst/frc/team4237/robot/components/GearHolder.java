package org.usfirst.frc.team4237.robot.components;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team4237.robot.control.DriverXbox;
import org.usfirst.frc.team4237.robot.control.Xbox;

/**
 * Class to control the gear holder + kicker
 * @author Mark Washington
 *
 */
public class GearHolder
{
    private DriverXbox xbox = DriverXbox.getInstance();

	private EnhancedDoubleSolenoid holder;
	private EnhancedDoubleSolenoid kicker;
	private Timer timer;
	private boolean isCallingGearDrop;
	private Constants.DropStage dropStage;

	private static GearHolder instance = new GearHolder();
	public static GearHolder getInstance()
	{
		return instance;
	}

	private GearHolder()
	{
		holder = new EnhancedDoubleSolenoid(Constants.Ports.OPEN_GEAR, Constants.Ports.CLOSE_GEAR);
		kicker = new EnhancedDoubleSolenoid(Constants.Ports.KICKER_OUT, Constants.Ports.KICKER_IN);
		timer = new Timer();
		isCallingGearDrop = false;
		dropStage = Constants.DropStage.kOpenHolder;
	}

	public void teleop()
    {
        //A BUTTON - Controls gear dropper
        //Gear holder will stay open as long as button is held
        if (xbox.getRawButton(Xbox.Constants.A_BUTTON) || isCallingGearDrop)
        {
            System.out.println("Dropping gear");
            dropGearSequence();
        }
        else if (!xbox.getRawButton(Xbox.Constants.A_BUTTON))
        {
            close();
        }
    }

	public void open()
	{
		holder.extend();
	}

	public void close()
	{
		dropStage = Constants.DropStage.kOpenHolder;
		holder.retract();
	}

	public void kickerOut()
	{
		kicker.extend();
	}

	public void kickerIn()
	{
		kicker.retract();
	}

	public boolean dropGearSequence()
	{
		if (isCallingGearDrop == false)
		{
			isCallingGearDrop = true;
			timer.stop();
			timer.reset();
			timer.start();
		}

		switch(dropStage)
		{
		case kOpenHolder:
			this.open();
			if (timer.get() >= Constants.GEAR_HOLDER_TIME)
			{
				dropStage = Constants.DropStage.kKickOut;
			}
			break;

		case kKickOut:
			this.kickerOut();
			if(timer.get() >= Constants.GEAR_HOLDER_TIME + Constants.KICK_TIME)
			{
				dropStage = Constants.DropStage.kKickIn;
			}
			break;

		case kKickIn:
			this.kickerIn();
			dropStage = Constants.DropStage.kCloseHolder;
			isCallingGearDrop = false;
			break;
		case kCloseHolder:
			isCallingGearDrop = false;
			break;
		}

		return isCallingGearDrop;
	}

	public static class Constants
	{
		//Ports
		public static class Ports
		{
			public static final int CLOSE_GEAR = 2;
			public static final int OPEN_GEAR = 3;
			public static final int KICKER_OUT = 4;
			public static final int KICKER_IN = 5;
		}

		public enum DropStage {kOpenHolder, kKickOut, kKickIn, kCloseHolder};

		static final double GEAR_HOLDER_TIME = 0.3;
		static final double KICK_TIME = 0.4;
	}
}