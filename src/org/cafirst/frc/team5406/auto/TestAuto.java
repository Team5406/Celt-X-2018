package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TestAuto extends AutonomousRoutine{

	public TestAuto(Robot _robot, WPI_TalonSRX _frontLeft, WPI_TalonSRX _frontRight, WPI_TalonSRX _intake) {
		super(_robot, _frontLeft, _frontRight, _intake);
		// TODO Auto-generated constructor stub
		name = "Test Auto";
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
