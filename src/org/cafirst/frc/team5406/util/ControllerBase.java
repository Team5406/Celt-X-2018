package org.cafirst.frc.team5406.util;

import edu.wpi.first.wpilibj.Joystick;

public class ControllerBase extends Joystick {

	int numButtons;
	boolean[] previousButtonState;
		
	public ControllerBase(int port) {
		super(port);
		numButtons = super.getButtonCount();
		previousButtonState = new boolean[numButtons + 1];
	}

    public void updateButtons() {
		for(int i = 1; i <= numButtons; i++){
			previousButtonState[i] = getRawButton(i);
		}
	}
	
	public boolean getButtonHeld(int button){
		return super.getRawButton(button);
	}
	
	public boolean getButtonOnce(int button){
		return (super.getRawButton(button) && !previousButtonState[button]);
	}
	
	public boolean getButtonRelease(int button){
		return (!super.getRawButton(button) && previousButtonState[button]);
	}
	
}
