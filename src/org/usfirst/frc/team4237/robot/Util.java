package org.usfirst.frc.team4237.robot;

public class Util
{
	/**
	 * Print information about passed object (used when everything loads for debugging)
	 * @param o Object to print info of
	 */
	public static void printObjectInfo(Object o)
	{
		System.out.println(o.getClass().getName() + ": " + o.toString());
	}
	
	/**
	 * Wait for an amount of time (in seconds)
	 * @param time Time to wait
	 */
	public static void wait(double time)
	{
		time = time * 1000; //Convert time from seconds to milliseconds
		try
		{
			Thread.sleep((int)time);
		} 
		catch (InterruptedException e)
		{
			e.printStackTrace();
		}
	}
}
