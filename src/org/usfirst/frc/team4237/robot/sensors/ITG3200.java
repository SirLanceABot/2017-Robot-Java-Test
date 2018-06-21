package org.usfirst.frc.team4237.robot.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

public class ITG3200 extends Thread
{
	private I2C gyro;
	private Timer timer;
	
	private float sensitivity;
	private double[] angle;
	private byte[] buffer = new byte[8];
	private float[] offset = new float[3];
	private float[] angleRate = new float[3];
	private float temperature;
	private float[] rotation = new float[3];
	boolean isWorking;
	private double integrationRate;
	private double previousTime;

	private static ITG3200 instance = new ITG3200(I2C.Port.kMXP, Constants.SENSITIVITY_SCALE_FACTOR, Constants.ITG3200_I2C_ADDR, 0.01, Constants.k1000Hz_10Hz, 0, 0, 0);
	public static ITG3200 getInstance()
	{
		return instance;
	}

	private ITG3200(
			I2C.Port port,
			float sensitivity,
			int deviceAddress,
			double integrationRate,
			byte sampleRate,
			double initialAngleX,
			double initialAngleY,
			double initialAngleZ
			)
	{
		gyro = new I2C(port, deviceAddress);
		this.start();
		this.timer = new Timer();
		this.sensitivity = sensitivity;
		this.isWorking = true;
		this.integrationRate = integrationRate;

		double[] initialAngle = {initialAngleX, initialAngleY, initialAngleZ};

		angle[0] = 0.0;
		angle[1] = 0.0;
		angle[2] = 0.0;

		timer.start();

		Timer.delay(0.1);

		if (isWorking() && gyro.write(Constants.ITG3200_PWR_MGM, Constants.ITG3200_RESET))
		{
			isWorking = false;
			System.out.println("[ITG-3200] Reset failed.");
		}

		Timer.delay(0.1);

		if (isWorking())
		{
			if (gyro.read(Constants.ITG3200_ID_REGISTER, 1, buffer))
			{
				isWorking = false;
				System.out.println("[ITG-3200] Read id failed.");
			}
			else
			{
				isWorking = (byte)(buffer[0] & Constants.ITG3200_ID_BITS) >> 1 == Constants.ITG3200_ID;
			}
		}

		if (isWorking())
		{
			System.out.printf("[ITG-3200] Gyro working on port %d, address $#.2x, device id register %#.2x.%n", port, deviceAddress, buffer[0]);

			setSampleRate(sampleRate);

			Timer.delay(1.0);
			System.out.println("[ITG-3200] Starting calibration - do not vibrate gyro until completed.");

			calibrate();

			setAngle(initialAngle);
		}
	}
	
	public boolean isWorking()
	{
		return isWorking;
	}
	
	public void calibrate()
	{
		int reads = 600;
		double delay = 0.004;
		int skip = 5;
		double[] temp = {0.0, 0.0, 0.0};

		this.interrupt();

		for (int i = 0; i < reads; i++)
		{
			getRaw();

			if (i >= skip)
			{
				for (int j = 0; j < 3; j++)
				{
					temp[j] += rotation[j];
				}
			}
			Timer.delay(delay);
		}

		for (int i = 0; i < 3; i++)
		{
			offset[i] = -(float)temp[i] / (float)(reads - skip);
			System.out.print(" " + offset[i] + ", ");
		}
		System.out.println();

		this.start();
	}

	@Override
	public synchronized void run()
	{
		double currentTime = timer.get();

		if (currentTime < previousTime)
		{
			previousTime = currentTime - integrationRate;
		}

		getRaw();

		for (int i = 0; i < 3; i++)
		{
			angleRate[i] = (rotation[i] + offset[i] / sensitivity);
			angle[i] += angleRate[i] * (currentTime - previousTime);
		}

		previousTime = currentTime;
	}
	
	public synchronized double[] getAngle()
	{
		return angle;
	}
	
	public synchronized float[] getAngleRate()
	{
		return angleRate;
	}
	
	public synchronized double getTemperature()
	{
		return 35.0 + (temperature + 13200.0) / 280;
	}
	
	public void setAngle(double[] angle)
	{
		this.angle = angle;
		previousTime = timer.get();
	}
	
	public void setSampleRate(byte sampleRate)
	{
		byte dlpffs;

		if (sampleRate < 0 || sampleRate > 7)
		{
			sampleRate = 9;
		}

		dlpffs = (byte)(Constants.ITG3200_FS_3 | sampleRate);

		if (gyro.write(Constants.ITG3200_DLPF_FS, dlpffs))
		{
			isWorking = false;
			System.out.println("[ITG-3200] Write configuration failed.");
		}
		else
		{
			System.out.printf("[ITG-3200] DLPF, Full Scale,  %#.2x%n", dlpffs);
		}

		Timer.delay(0.06);
	}
	
	public float[] getOffset()
	{
		return offset;
	}
	
	public double getX()
	{
		return getAngle()[0];
	}
	
	public double getY()
	{
		return getAngle()[1];
	}

	public double getZ()
	{
		return getAngle()[2];
	}

	public synchronized void getRaw()
	{
		if (gyro.read(Constants.ITG3200_TEMP_H, 8, buffer))
		{
			System.out.println("[ITG-3200] getRaw() read failed.");
			for (byte i = 0; i < buffer.length; i++)
			{
				buffer[i] = 0;
			}
		}

		rotation[0] = (float)(((buffer[2] << 8) | buffer[3])); //X
		rotation[1] = (float)(((buffer[4] << 8) | buffer[5])); //Y
		rotation[2] = (float)(((buffer[6] << 8) | buffer[7])); //Z

		temperature = (float)(((buffer[0] << 8) | buffer[1])); //Temperature
	}

	public static class Constants
	{
		// I2C addresses
		public static final int ITG3200_I2C_ADDR = 0x68;
		public static final int ITG3200_I2C_ADDR_ALT = 0x69;

		// default sensitivity
		public static final float SENSITIVITY_SCALE_FACTOR = 14.375f;

		// sample rate
		public static final byte k8000Hz_256Hz = 0; // 256Hz low pass filter bandwidth,	8,000Hz Internal Sample Rate
		public static final byte k1000Hz_188Hz = 1; // 188Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
		public static final byte k1000Hz_98Hz  = 2; //  98Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
		public static final byte k1000Hz_42Hz  = 3; //  42Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
		public static final byte k1000Hz_20Hz  = 4; //  20Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
		public static final byte k1000Hz_10Hz  = 5; //  10Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
		public static final byte k1000Hz_5Hz   = 6; //   5Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate

		private static final int ITG3200_ID_REGISTER = 0x00;
		private static final int ITG3200_ID = 0x34;
		private static final int ITG3200_ID_BITS = 0x7e;

		// configuration registers
		private static final int ITG3200_SMPLRT_DIV = 0x15;
		private static final int ITG3200_DLPF_FS = 0x16;

		// interrupt registers
		private static final int ITG3200_INT_CFG = 0x17;
		private static final int ITG3200_INT_STATUS = 0x1A;

		// data registers (read only)
		private static final int ITG3200_TEMP_H = 0x1B;
		private static final int ITG3200_TEMP_L = 0x1C;
		private static final int ITG3200_XOUT_H = 0x1D;
		private static final int ITG3200_XOUT_L = 0x1E;
		private static final int ITG3200_YOUT_H = 0x1F;
		private static final int ITG3200_YOUT_L = 0x20;
		private static final int ITG3200_ZOUT_H = 0x21;
		private static final int ITG3200_ZOUT_L = 0x22;
		private static final int DATA_REG_SIZE = 8;

		// power management
		private static final int ITG3200_PWR_MGM = 0x3E;

		// useful values
		private static final int ITG3200_RESET = 0x80;
		private static final int ITG3200_SLEEP = 0x40;
		private static final int ITG3200_WAKEUP = 0x00;
		// FS=11 DLPF=000 => 11000 => 0x18 => 0b11'000
		private static final int ITG3200_FS_3 = 0x18;
	}
}