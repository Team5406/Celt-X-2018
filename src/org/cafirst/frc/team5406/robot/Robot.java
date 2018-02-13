/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.cafirst.frc.team5406.robot;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Solenoid;
import org.cafirst.frc.team5406.util.XboxController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Notifier;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(1);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(5);
	
	WPI_TalonSRX _intakeMotor= new WPI_TalonSRX(10);
	WPI_TalonSRX _intakeSlave1 = new WPI_TalonSRX(9);
	WPI_TalonSRX _wristMotor = new WPI_TalonSRX(13);
	WPI_TalonSRX _elevatorMotor= new WPI_TalonSRX(12);

	

	/* extra talons for six motor drives */
	WPI_VictorSPX _leftSlave1 = new WPI_VictorSPX(2);
	WPI_VictorSPX _leftSlave2 = new WPI_VictorSPX(3);
	WPI_VictorSPX _leftSlave3 = new WPI_VictorSPX(4);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(6);
	WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(7);
	WPI_VictorSPX _rightSlave3 = new WPI_VictorSPX(8);
	WPI_VictorSPX _elevatorSlave1 = new WPI_VictorSPX(11);


	double speed = 0;
	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);
    Solenoid elevatorSolenoid;
    Solenoid gripSolenoidLow;
    Solenoid gripSolenoidHigh;
    Solenoid wristSolenoid;
    int flipWrist = 0;
    int gripCounter = 0;
	int autoLoop = 0;
	int step =0;
	
    private XboxController driverGamepad;
    private XboxController operatorGamepad;

    class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {  
	    	double _pos = _wristMotor.getActiveTrajectoryPosition();
	    	if (_pos > 500 && flipWrist==1) {
	    		    	wristSolenoid.set(true);
	    		    	flipWrist = 2;
	    		    	
	    	}
	    	if (_pos < 5000 && flipWrist==3) {
		    	wristSolenoid.set(false);
		    	flipWrist = 4;
	    	}
	    	/*if (_pos > 6000 && flipWrist==2) {
		    	wristSolenoid.set(false);
		    	flipWrist = 0;
	    	}*/
	    	}
	}
	Notifier _notifier = new Notifier(new PeriodicRunnable());
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	gripSolenoidHigh = new Solenoid(3);
    	elevatorSolenoid = new Solenoid(0); //1
    	wristSolenoid = new Solenoid(2);
    	gripSolenoidLow = new Solenoid(1); //0

    	driverGamepad = new XboxController(0);
    	operatorGamepad = new XboxController(1);

    	/* take our extra talons and just have them follow the Talons updated in arcadeDrive */
    	_leftSlave1.follow(_frontLeftMotor);
    	_leftSlave2.follow(_frontLeftMotor);
    	_leftSlave3.follow(_frontLeftMotor);
    	_rightSlave1.follow(_frontRightMotor);
    	_rightSlave2.follow(_frontRightMotor);
    	_rightSlave3.follow(_frontRightMotor);
    	
    	_elevatorSlave1.follow(_elevatorMotor);
    	_intakeSlave1.follow(_intakeMotor);
    	_intakeSlave1.setInverted(true);
    	
    	_wristMotor.setSensorPhase(false);

    	autoLoop = 0;
    	/*_leftSlave1.setInverted(true);
    	_leftSlave2.setInverted(true);
    	_rightSlave1.setInverted(true);
    	_rightSlave2.setInverted(true);*/
    	_frontRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 50);
		_frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
		_frontLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 50);
		_frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
		_frontLeftMotor.configContinuousCurrentLimit(30, 0);
		_frontLeftMotor.configPeakCurrentLimit(30, 0);
		_frontLeftMotor.configPeakCurrentDuration(30, 0);
		_frontLeftMotor.enableCurrentLimit(true);
		_frontRightMotor.configContinuousCurrentLimit(30, 0);
		_frontRightMotor.configPeakCurrentLimit(30, 0);
		_frontRightMotor.configPeakCurrentDuration(30, 0);
		_frontRightMotor.enableCurrentLimit(true);
		_wristMotor.configContinuousCurrentLimit(40, 0);
		_wristMotor.configPeakCurrentLimit(40, 0);
		_wristMotor.configPeakCurrentDuration(40, 0);
		_wristMotor.enableCurrentLimit(true);

    	_elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 50);
		_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
		_wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 50);
		_wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);

    	int kTimeoutMs = 10;
    	_wristMotor.selectProfileSlot(0,0);
    	_wristMotor.config_kF(0, 0.2, kTimeoutMs);
    	_wristMotor.config_kP(0, 0.2, kTimeoutMs);
    	_wristMotor.config_kI(0, 0, kTimeoutMs);
    	_wristMotor.config_kD(0, 0, kTimeoutMs);
    	_wristMotor.selectProfileSlot(1,0);
    	_wristMotor.config_kF(1, 0.2, kTimeoutMs);
    	_wristMotor.config_kP(1, 0.15, kTimeoutMs);
    	_wristMotor.config_kI(1, 0, kTimeoutMs);
    	_wristMotor.config_kD(1, 0, kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
    	_wristMotor.configMotionCruiseVelocity(1500, kTimeoutMs);
    	_wristMotor.configMotionAcceleration(2500, kTimeoutMs);
		/* zero the sensor */
    	_wristMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
    	
    	_elevatorMotor.selectProfileSlot(0,0);
    	_elevatorMotor.config_kF(0, 0.2, kTimeoutMs);
    	_elevatorMotor.config_kP(0, 0.2, kTimeoutMs);
    	_elevatorMotor.config_kI(0, 0, kTimeoutMs);
    	_elevatorMotor.config_kD(0, 0.0, kTimeoutMs);
    	_elevatorMotor.selectProfileSlot(1,0);
    	_elevatorMotor.config_kF(1, 0.2, kTimeoutMs);
    	_elevatorMotor.config_kP(1, 0.2, kTimeoutMs);
    	_elevatorMotor.config_kI(1, 0, kTimeoutMs);
    	_elevatorMotor.config_kD(1, 0.0, kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
    	_elevatorMotor.configMotionCruiseVelocity(8000, kTimeoutMs);
    	_elevatorMotor.configMotionAcceleration(5000, kTimeoutMs);
		/* zero the sensor */
    	_elevatorMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);


    }

    /**
     * This function is called periodically during operator control
     */
    
    public void teleopPeriodic() {
        _drive.arcadeDrive(driverGamepad.getLeftY(), driverGamepad.getLeftX());

		if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
			elevatorDown();
			wristUp();
			gripSolenoidHigh.set(true);
			elevatorSolenoid.set(false);
			wristSolenoid.set(false);
        }
		
		if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
			wristDown();
			elevatorUp();
			gripSolenoidHigh.set(true);
			elevatorSolenoid.set(false);
			wristSolenoid.set(false);
		}
		
		
		if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
			wristDown();
			elevatorDown();
			gripSolenoidHigh.set(true);
			elevatorSolenoid.set(false);
			flipWrist =3;
			_notifier.startPeriodic(0.005);
			

		}
		
		if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
			wristUp();
			elevatorUp();
			gripSolenoidHigh.set(true);
			elevatorSolenoid.set(false);
			flipWrist =1;
			_notifier.startPeriodic(0.005);
			//wristSolenoid.set(true);
			//wristSolenoid.set(true);
		}
		
		
		if(operatorGamepad.getLeftTriggerPressed()){
			_intakeMotor.set(0.8);
        }else if(operatorGamepad.getRightTriggerPressed()) {
        	_intakeMotor.set(-1);
        }else {
        	_intakeMotor.set(0);
        }
	   
		
		switch(operatorGamepad.getDirectionPad()){
		case UP:
			gripSolenoidHigh.set(true);
			gripSolenoidLow.set(true);
			break;
		case DOWN:
			gripSolenoidHigh.set(false);
			gripSolenoidLow.set(false);
			break;
		case LEFT:
			gripSolenoidHigh.set(true);
			gripSolenoidLow.set(false);
			break;
		case RIGHT:
			gripSolenoidHigh.set(false);
			gripSolenoidLow.set(true);
			break;
		case NONE:
		}

		if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
			wristSolenoid.set(true);
		}
		if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
			wristSolenoid.set(false);
		}
		
    }
    
    public void autonomousInit() {
    	autoLoop=0;
    	//shiftSolenoid.set(true);
    	_frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
    	_frontRightMotor.setSelectedSensorPosition(0, 0, 10);
    	speed =0;
    	step =0;
    }
    
    public void wristUp() {
    	_wristMotor.selectProfileSlot(0,0);
    	_wristMotor.set(ControlMode.MotionMagic, 6900);
    }
    public void wristDown() {
    	_wristMotor.selectProfileSlot(1,0);
    	_wristMotor.set(ControlMode.MotionMagic, 100);
    }
    public void elevatorUp() {
    	_elevatorMotor.selectProfileSlot(0,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, 175000);
    }
    public void elevatorDown() {
    	_elevatorMotor.selectProfileSlot(1,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, 1000);
    }
    public void autonomousPeriodic() {
    	/*
    	double positionLeft = _frontLeftMotor.getSelectedSensorPosition(0);
    	double positionRight = _frontRightMotor.getSelectedSensorPosition(0);
    	
    	switch(step) {
    	case 0:
    		speed+=0.015;
    		if (positionRight>2000) {
    			step =1;
    		}
    		break;
    	case 1:
    		speed = 0.7;
    		if(positionRight > 28000) {
    			step =2;
    		}
    		break;
    	case 2:
    		speed-=0.015;
    		if(positionRight > 33800 || speed <=0.1) {
    			step =3;
    		}
    		break;
    	case 3:
    		
    		if(positionRight > 34200) {
    			speed = -0.1;
    		}else {
    			speed=0;
    		}
    		break;    	 	
    	}
    	System.out.println(positionRight);*/
    	//_drive.tankDrive(0.5,0.5);
    	
    }
}