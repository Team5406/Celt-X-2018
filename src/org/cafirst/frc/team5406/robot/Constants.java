package org.cafirst.frc.team5406.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public final class Constants {
	public static int ARM_UP = 6850;
	public static int ARM_DOWN = 100;
	public static double ARM_RANGE = 135; //degrees
	public static double WRIST_RANGE = 135; //degrees
	public static int ELEVATOR_UP = 215000;
	public static int ELEVATOR_DOWN = 1000;
	public static int WRIST_UP = 0;
	public static int WRIST_DOWN = -2700;
	public static int ARM_DOWN_THRESHOLD = 500;
	public static int ELEVATOR_DOWN_THRESHOLD = 2500;
	public static int RAMP_SERVO_START = 75;
	public static int RAMP_SERVO_RELEASE = 180;

	public static double GYRO_PID_P = 0.022;
	public static double GYRO_PID_I = 0.000;
	public static double GYRO_PID_D = 0.125;
	public static double WHEEL_BASE = 35.5;
	
	public static String AXIS_IP = "10.54.6.11";
	
	public static int IMAGE_WIDTH = 480;
	public static int IMAGE_HEIGHT = 320;
	public static int OFF_CENTER = 0;
	public static int IMAGE_CENTER = IMAGE_WIDTH / 2 + OFF_CENTER;
	
	public static double CAMERA_FIELD = Math.PI/3;
	public static double CAMERA_FIELD_HALF = CAMERA_FIELD/2;
	
	public static AHRS navX = new AHRS(SPI.Port.kMXP);
	
	public static double xboxControllerDeadband = 0.2;
	
	public static double revPerInch = 0.06;
	public static double maxSpeed = 5 * 12 * revPerInch * 60;
	public static double minSpeed = 1 * 12 * revPerInch * 60;
	public static double driveGearRatio = 4.125; //66/16
	
	
	public static int[] pdpSlots = {
			4,  //left 1
			0,  //left 2
			2,  //left 3
			1,  //left 4
			11, //right 1
			14, //right 2
			15, //right 3
			13  //right 4
	};
	
	//PCM Ports
	public static int GRIP_HIGH = 3;
	public static int ELEVATOR_SHIFT = 1;
	public static int RAMP_RELEASE = 2;
	public static int GRIP_LOW = 0;

	public static int kTimeoutMs = 10;
	
	public boolean equalsDeadband(double value){
	return ((-1*xboxControllerDeadband) < value && value < xboxControllerDeadband);
	}
	
	
	public Constants() {
	}
}