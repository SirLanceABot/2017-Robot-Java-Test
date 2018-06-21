package org.usfirst.frc.team4237.robot;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Class to control and manage everything to do with robot vision
 * @author mark
 *
 */
public class Vision
{
	enum TurnDirection 
	{
		kLeft(-1), kStraight(0), kRight(1);
		private final int num;
		private TurnDirection(int num)
		{
			this.num = num;
		}
	}
	public static final int CENTER = 160;
	public static final int FINAL_ALIGNMENT_POINT = 250;
	public static final int CLOSE_TO_TARGET_HEIGHT = 55;
	public static final int CLOSE_TO_TARGET_DISTANCE_SONAR = 5;
	public static final int ACCEPTABLE_OFFSET = 6;

	private NetworkTable mNetworkTable;
	private Timer mUsePiTimer;

	private UsbCamera mAutoCamera;
	private VideoSink mVisionServer;

	private int mMidpointX;
	private int mNumTargets;
	private int mOffset;
	private int mLargestHeight;
	private int mCurrentAlignmentPoint;

	private ArrayList<Double> mHeightArrayList;
	private ArrayList<Double> mCenterXArrayList;
	private ArrayList<Double> mTargetInfo;
	private double[] mTempTargetInfo;

	private int mTargetInfoID;
	private int mPreviousTargetInfoID;

	boolean mIsPiWorking;

	public Vision()
	{
		Util.printObjectInfo(this);
		mUsePiTimer = new Timer();
		mNetworkTable = NetworkTable.getTable("GRIP/vision");
		mMidpointX = -9999;
		mNumTargets = -9999;
		mCurrentAlignmentPoint = -9999;
		mOffset = -9999;
		mLargestHeight = -9999;
		mHeightArrayList = new ArrayList<Double>();
		mCenterXArrayList = new ArrayList<Double>();
		mTargetInfo = new ArrayList<Double>();
		mPreviousTargetInfoID = -1;
		mTargetInfoID = 0;
		mIsPiWorking = true;
		mAutoCamera = CameraServer.getInstance().startAutomaticCapture("AutoCamera", "/dev/v4l/by-id/usb-HD_Camera_Manufacturer_HD_USB_Camera-video-index0");
		mVisionServer = CameraServer.getInstance().getServer();
		mVisionServer.setSource(mAutoCamera);
		//double temp = 0.0;
		mUsePiTimer.stop();
		mUsePiTimer.reset();
	}

	/**
	 * Verifies ArrayLists to make sure they are valid
	 * @return
	 */
	public boolean areArrayListsValid()
	{
		return mHeightArrayList.size() == mCenterXArrayList.size() && mCenterXArrayList.size() > 0;
	}

	/**
	 * Sets camera exposure to a low value for autonomous.
	 */
	public void setExposureLow()
	{
		mAutoCamera.setExposureManual(0);
	}

	/**
	 * Sets camera exposure to a high value for human operation.
	 */
	public void setExposureHigh()
	{
		mAutoCamera.setExposureManual(13);
	}

	public int getMidpoint()
	{
		return mMidpointX;

	}

	/**
	 * Returns the number of pieces of tape seen by the camera
	 * @return The number of pieces of tape seen by the camera
	 */
	public int getNumTargets()
	{
		return mNumTargets;
	}

	/**
	 * Returns the offset of the robot from the target
	 * @return The offset of the robot from the target
	 */
	public int getOffset()
	{
		return mOffset;
	}

	/**
	 * Returns the height of the largest piece of reflective tape in the camera's view
	 * @return The height of the largest piece of reflective tape in the camera's view
	 */
	public int getLargestHeight()
	{
		return mLargestHeight;
	}


	@Deprecated
	public boolean isCloseToTargetWithSensor()
	{
		return false;
	}

	/**
	 * Returns a boolean of whether the robot is close enough to the target or not.
	 * @return
	 */
	public boolean isCloseToTargetWithVision()
	{
		return mLargestHeight >= CLOSE_TO_TARGET_HEIGHT;
	}

	/**
	 * Returns boolean of whether the robot is sufficiently aligned with the target
	 * @return
	 */
	public boolean isAlignedWithTarget()
	{
		return (mCurrentAlignmentPoint - ACCEPTABLE_OFFSET) <= mMidpointX && mMidpointX <= (mCurrentAlignmentPoint + ACCEPTABLE_OFFSET);
	}

	/**
	 * Get speed to turn at depending on how far robot is offset from target.
	 * @return Speed to turn at
	 */
	public double getTurnSpeed()
	{
		double kP = -0.00069;
		double controllerMin = 0.22;
		double controllerMax = 0.7;

		double speed = 0.0;

		if (mMidpointX >= 0)
		{
			if (Math.abs(mOffset) <= ACCEPTABLE_OFFSET)
			{
				speed = 0.0;
			}
			else
			{
				speed = kP * mOffset;
				if (speed >= 0)
				{
					speed = Math.min(speed, controllerMax);
					speed = Math.max(speed, controllerMin);
				}
				else
				{
					speed = Math.max(speed, -controllerMax);
					speed = Math.min(speed, -controllerMin);
				}
			}
		}
		else
		{
			System.out.println("[Vision] No valid target to turn towards");
		}

		System.out.println("[Vision] Turning speed: " + speed);

		return speed;
	}

	/**
	 * Returns the speed the robot should turn at using the center of the target
	 * @return The speed the robot should turn at
	 */
	public double getTurnSpeedUsingCenter()
	{
		double controllerMin = 0.22;
		TurnDirection turnDirection;
		double speed;

		if (mMidpointX > CENTER)
		{
			turnDirection = TurnDirection.kRight;
		}
		else
		{
			turnDirection = TurnDirection.kLeft;
		}
		speed = controllerMin * turnDirection.num;
		System.out.println("[Vision] Turn speed using center: " + speed);
		return speed;
	}

	/**
	 * Returns an ArrayList containing the heights of every piece of tape in the camera's view
	 * @return
	 */
	public ArrayList<Double> getHeightVector()
	{
		return mHeightArrayList;
	}

	public boolean update()
	{
		boolean isDataGood;
		
		updateTargetInfoArrayList();
		isDataGood = areArrayListsValid();
		if (isDataGood)
		{
			sortHeightCenterX();
			updateNumTargets();
			updateMidpoint();
			updateLargestHeight();
			updateCurrentAlignmentPoint();
			updateOffset();
		}
		else
		{
			System.out.println("[Vision] The ArrayLists are either empty or not the same size");
			System.out.print("\t\tHeight ArrayList: ");
			for (double d : mHeightArrayList)
			{
				System.out.println(" " + d);
			}
			System.out.println();
			
			System.out.print("\t\tCenter X ArrayList: ");
			for (double d : mCenterXArrayList)
			{
				System.out.println(" " + d);
			}
		}
		return isDataGood;
	}

	void updateMidpoint()
	{
		if (mNumTargets >= 2)
		{
			if (mCenterXArrayList.get(0) > mCenterXArrayList.get(1))
			{
				mMidpointX = mCenterXArrayList.get(1).intValue();
			}
			else
			{
				mMidpointX = mCenterXArrayList.get(0).intValue();
			}
		}
		else if (mNumTargets == 1)
		{
			mMidpointX = mCenterXArrayList.get(0).intValue();
		}
		else
		{
			mMidpointX = -9999;
		}
		System.out.println("[Vision] MidpointX: " + mMidpointX);

		if (mNumTargets >= 2 && mCenterXArrayList.get(0) > mCenterXArrayList.get(1))
		{
			System.out.println("WARNING [Vision] Seeing two targets and left is smaller");
		}

	}

	void updateNumTargets()
	{
		mNumTargets = mCenterXArrayList.size();
		if (mNumTargets > 2)
		{
			System.out.println("[Vision] Number of contours found: " + mNumTargets);
			mNumTargets = 2;
		}
	}

	void updateOffset()
	{
		mOffset = mCurrentAlignmentPoint - mMidpointX;
		System.out.println("[Vision]: Offset: " + mOffset);
	}

	void updateLargestHeight()
	{
		if (mNumTargets > 0)
		{
			mLargestHeight = mHeightArrayList.get(0).intValue();
		}
		else
		{
			mLargestHeight = CLOSE_TO_TARGET_HEIGHT;
		}
		System.out.println("[Vision] Max height: " + mLargestHeight);
	}

	void updateTargetInfoArrayList()
	{
		if (mIsPiWorking)
		{
			mTempTargetInfo = mNetworkTable.getNumberArray("Pi TargetInfo", new double[] {-9999.0});
			for (double d : mTempTargetInfo)
			{
				mTargetInfo.add(d);
			}
		}
		else
		{
			//TODO: Add backup vision code for RoboRIO
			System.out.println("[Vision] Backup vision code has not been implemented yet");
		}
		System.out.print("[Vision] TargetInfo: ");
		for (double d : mTargetInfo)
		{
			System.out.print(d + " ");
		}
		System.out.println();

		mCenterXArrayList.clear();
		mHeightArrayList.clear();

		if (mTargetInfo.size() < 2 || mTargetInfo.size() % 2 != 0)
		{
			System.out.println("[Vision] Damaged camera frame, size: " + mTargetInfo.size());
		}
		else
		{
			double checksum = mTargetInfo.get(mTargetInfo.size() - 1);
			mTargetInfoID = mTargetInfo.get(mTargetInfo.size() - 2).intValue();
			checksum -= mTargetInfoID;

			for (int targetInfoIndex = 0; targetInfoIndex < mTargetInfo.size() - 2; targetInfoIndex += 2)
			{
				checksum -= mTargetInfo.get(targetInfoIndex);
				mCenterXArrayList.add(mTargetInfo.get(targetInfoIndex));

				checksum -= mTargetInfo.get(targetInfoIndex + 1);
				mHeightArrayList.add(mTargetInfo.get(targetInfoIndex + 1));
			}

			if (checksum != 0)
			{
				mCenterXArrayList.clear();
				mHeightArrayList.clear();
				System.out.println("[Vision] Camera frame error, checksum: " + checksum);
			}
			else
			{
				if (mPreviousTargetInfoID + 1 != mTargetInfoID)
				{
					System.out.println("Skipped processing camera frame " + mPreviousTargetInfoID + 1 + " to frame " + (mTargetInfoID - 1));
				}

				System.out.println("Processing frame ID " + mTargetInfoID);
			}
		}
		if (mIsPiWorking)
		{
			mIsPiWorking = checkPi();
		}
		mPreviousTargetInfoID = mTargetInfoID;
	}

	void updateCenterXArrayList()
	{
		double[] temp = mNetworkTable.getNumberArray("centerX", new double[] {-9999});
		mCenterXArrayList.clear();
		for (double d : temp)
		{
			mCenterXArrayList.add(d);
		}
	}

	void updateHeightVector()
	{
		double[] temp = mNetworkTable.getNumberArray("height", new double[] {-9999});
		mHeightArrayList.clear();
		for (double d : temp)
		{
			mHeightArrayList.add(d);
		}
	}

	void updateCurrentAlignmentPoint()
	{
		mCurrentAlignmentPoint = (int)((double)CENTER + (double)(110 - CENTER) * Math.min((double)mLargestHeight / (double)CLOSE_TO_TARGET_HEIGHT, 1.0));
		System.out.println("[Vision] Current alignment point:" + mCurrentAlignmentPoint);
	}

	void sortHeightCenterX()
	{
		for (int i = 0; i < mHeightArrayList.size(); i++)
		{
			for (int j = i + 1; j < mHeightArrayList.size(); j++)
			{
				if (mHeightArrayList.get(i) < mHeightArrayList.get(j))
				{
					Collections.swap(mHeightArrayList, i, j);
				}
			}
		}
		System.out.println("[Vision] Processing frame ID: " + mTargetInfoID);
		System.out.print("Height ArrayList: ");
		for (double d : mHeightArrayList)
		{
			System.out.print(d + " ");
		}
		System.out.println();

		System.out.println("Center X ArrayList: ");
		for (double d : mCenterXArrayList)
		{
			System.out.println(d + " ");
		}
		System.out.println();
	}

	public boolean checkPi()
	{
		double time = mUsePiTimer.get();
		System.out.println("[Vision] Pi Timer: " + time);
		if (!mIsPiWorking)
		{
			System.out.println("WARNING [Vision] Pi is not working");
			return false;
		}
		else if (mTargetInfo.size() < 2 || mTargetInfo.size() % 2 != 0)
		{
			if (time < 0.8)
			{
				System.out.println("WARNING [Vision] Pi might not be working");
				return true;
			}
			else
			{
				System.out.println("WARNING [Vision] Pi is not working, switching to RIO");
				return false;
			}
		}
		else if (mPreviousTargetInfoID == mTargetInfoID)
		{
			if (time < 0.8)
			{
				System.out.println("WARNING [Vision] Pi might not be working");
				return true;
			}
			else
			{
				System.out.println("WARNING [Vision] Pi is not working, switching to RIO");
				return false;
			}
		}
		else
		{
			mUsePiTimer.reset();
			return true;
		}
	}
	
	//TODO: Add vision thread, resetPiTimer(), startPiTimer(), startVisionThread(), stopVisionThread()
	public static void visionThread()
	{

	}
}