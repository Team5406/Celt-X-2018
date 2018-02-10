package org.cafirst.frc.team5406.util;

public class XboxController extends ControllerBase {

	public static final int A_BUTTON = 1;
	public static final int B_BUTTON = 2;
	public static final int X_BUTTON = 3;
	public static final int Y_BUTTON = 4;
	public static final int LEFT_BUMPER = 5;
	public static final int RIGHT_BUMPER = 6;
	public static final int BACK_BUTTON = 7;
	public static final int START_BUTTON = 8;
	public static final int LEFT_STICK = 9;
	public static final int RIGHT_STICK = 10;

	public static final int LEFT_X_AXIS = 0;
	public static final int LEFT_Y_AXIS = 1;
	public static final int LEFT_TRIGGER_AXIS = 2;
	public static final int RIGHT_TRIGGER_AXIS = 3;
	public static final int RIGHT_X_AXIS = 4;
	public static final int RIGHT_Y_AXIS = 5;
	
	public static enum DirectionPad{
		UP,
		DOWN,
		LEFT,
		RIGHT,
		NONE
	}

	public XboxController(int port) {
		super(port);
	}
	
	public double getLeftX(){
		return super.getRawAxis(LEFT_X_AXIS);
	}
	
	public double getLeftY(){
		return -super.getRawAxis(LEFT_Y_AXIS);
	}
	
	public double getRightX(){
		return super.getRawAxis(RIGHT_X_AXIS);
	}
	
	public double getRightY(){
		return -super.getRawAxis(RIGHT_Y_AXIS);
	}
	
	public double getLeftTrigger(){
		return super.getRawAxis(LEFT_TRIGGER_AXIS);
	}
	
	public boolean getLeftTriggerPressed(){
		return Math.abs(super.getRawAxis(LEFT_TRIGGER_AXIS)) > 0.2;
	}
	
	public double getRightTrigger(){
		return super.getRawAxis(RIGHT_TRIGGER_AXIS);
	}
	
	public boolean getRightTriggerPressed(){
		return Math.abs(super.getRawAxis(RIGHT_TRIGGER_AXIS)) > 0.2;
	}
	
	public DirectionPad getDirectionPad(){
		int dir = super.getPOV();
		switch(dir){
			default:
				return DirectionPad.NONE;
			case 0:
				return DirectionPad.UP;
			case 90:
				return DirectionPad.RIGHT;
			case 180:
				return DirectionPad.DOWN;
			case 270:
				return DirectionPad.LEFT;
		}
	}
	
}
