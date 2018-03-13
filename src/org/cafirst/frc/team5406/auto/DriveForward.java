package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveForward extends AutonomousRoutine{
	
	protected int autoLoop;
	protected int step;
	protected double speed;
	protected Robot robot;
	
	protected WPI_TalonSRX _frontLeftMotor;
	protected WPI_TalonSRX _frontRightMotor;
	
	protected WPI_TalonSRX _intakeMotor;
	
	public DriveForward(Robot _robot, WPI_TalonSRX _frontLeft, WPI_TalonSRX _frontRight, WPI_TalonSRX _intake) {
		robot = _robot;
		_frontLeftMotor = _frontLeft;
		_frontRightMotor = _frontRight;
		
		_intakeMotor = _intake;
		
		name = "Drive Forward";
	}

	@Override
	public void periodic() {
		double positionLeft = _frontLeftMotor.getSelectedSensorPosition(0);
    	double positionRight = _frontRightMotor.getSelectedSensorPosition(0);
    	
    	switch(step) {
    	case 0:
    		robot.wristDown();
    		robot.elevatorUp();
    		step = 1;
    	case 1:
    		speed+=0.015;
    		if (positionLeft>10000) {
    			step =2;
    		}
    		break;
    	case 2:
    		speed = 0.7;
    		if(positionLeft > 35000) {
    			step =3;
    		}
    		break;
    	case 3:
    		speed-=0.05;
    		if(positionLeft > 51000 || speed <=0.1) {
    			step =4;
    		}
    		break;
    	case 4:
    		if(positionLeft > 50000) {
    			speed = -0.1;
        		_intakeMotor.set(0.8);
    		}else {
    			speed=0;
    			step = 5;
    		}
    		break;   
    	case 5:
    		_intakeMotor.set(0.8);
    	}
    	System.out.println(positionRight);
    	robot._drive.tankDrive(speed,speed);
	}

	@Override
	public void init() {
		robot.setupMotors();
    	autoLoop=0;
    	//shiftSolenoid.set(true);
    	_frontLeftMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
    	_frontRightMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
		step = 0;
		speed = 0;
	}

}
