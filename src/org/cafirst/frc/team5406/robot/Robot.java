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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(1); //drive
	WPI_VictorSPX _leftSlave1 = new WPI_VictorSPX(2); //left drive
	WPI_VictorSPX _leftSlave2 = new WPI_VictorSPX(3);
	WPI_VictorSPX _leftSlave3 = new WPI_VictorSPX(4);
	
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(5);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(6);
	WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(7);
	WPI_VictorSPX _rightSlave3 = new WPI_VictorSPX(8); //right drive
	
	WPI_TalonSRX _intakeMotor= new WPI_TalonSRX(10);
	WPI_VictorSPX _intakeSlave1 = new WPI_VictorSPX(9);
	WPI_TalonSRX _elevatorMotor= new WPI_TalonSRX(12); //
	WPI_VictorSPX _elevatorSlave1 = new WPI_VictorSPX(11);
	WPI_TalonSRX _armMotor = new WPI_TalonSRX(13); //arm motor
	WPI_TalonSRX _wristMotor= new WPI_TalonSRX(14);



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
	
	boolean armUp = false;
	boolean elevatorUp = false;
	boolean wristUp = false;
	int testCounter = 0;
	double motorCurrentSum = 0;
	int motorCurrentCount = 0;
	long motorEncoder = 0;
	int motorDirection = 1;
	boolean buttonPress = false;
	
    private XboxController driverGamepad;
    private XboxController operatorGamepad;

/*    class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {  
	    	double _pos = _armMotor.getActiveTrajectoryPosition();
	    	if (_pos > 500 && flipWrist==1) {
	    		    	wristUp()
	    		    	flipWrist = 2;
	    		    	
	    	}
	    	if (_pos < 5000 && flipWrist==3) {
		    	wristDown();
		    	flipWrist = 4;
	    	}
	    	if (_pos > 6000 && flipWrist==2) {
		    	wristSolenoid.set(false);
		    	flipWrist = 0;
	    	}
	    	}
	}
	Notifier _notifier = new Notifier(new PeriodicRunnable());*/
	public void setupMotors() {
		_leftSlave1.follow(_frontLeftMotor);
    	_leftSlave2.follow(_frontLeftMotor);
    	_leftSlave3.follow(_frontLeftMotor);
    	_rightSlave1.follow(_frontRightMotor);
    	_rightSlave2.follow(_frontRightMotor);
    	_rightSlave3.follow(_frontRightMotor);
    	
    	_elevatorSlave1.follow(_elevatorMotor);
    	_intakeSlave1.follow(_intakeMotor);
    	_intakeSlave1.setInverted(true);
    	
    	_armMotor.setSensorPhase(false);
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
		_armMotor.configContinuousCurrentLimit(40, 0);
		_armMotor.configPeakCurrentLimit(40, 0);
		_armMotor.configPeakCurrentDuration(40, 0);
		_armMotor.enableCurrentLimit(true);
		_intakeMotor.configContinuousCurrentLimit(40, 0);
		_intakeMotor.configPeakCurrentLimit(40, 0);
		_intakeMotor.configPeakCurrentDuration(40, 0);
		_intakeMotor.enableCurrentLimit(true);
		_wristMotor.configContinuousCurrentLimit(30, 0);
		_wristMotor.configPeakCurrentLimit(30, 0);
		_wristMotor.configPeakCurrentDuration(30, 0);
		_wristMotor.enableCurrentLimit(true);

    	_elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 50);
		_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
		_armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 50);
		_armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
		_wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 50);
		_wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);

    	int kTimeoutMs = 10;
    	_wristMotor.selectProfileSlot(0,0);
    	_wristMotor.config_kF(0, 1, kTimeoutMs);
    	_wristMotor.config_kP(0, 0.25, kTimeoutMs);
    	_wristMotor.config_kI(0, 0, kTimeoutMs);
    	_wristMotor.config_kD(0, 0, kTimeoutMs);
    	_wristMotor.selectProfileSlot(1,0);
    	_wristMotor.config_kF(1, .1, kTimeoutMs);
    	_wristMotor.config_kP(1, 1, kTimeoutMs);
    	_wristMotor.config_kI(1, 0, kTimeoutMs);
    	_wristMotor.config_kD(1, 1.5, kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
    	_wristMotor.configMotionCruiseVelocity(4000, kTimeoutMs);
    	_wristMotor.configMotionAcceleration(2000, kTimeoutMs);
		/* zero the sensor */
    	_wristMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);

    	
    	_armMotor.selectProfileSlot(0,0);
    	_armMotor.config_kF(0, 0.2, kTimeoutMs);
    	_armMotor.config_kP(0, 0.2, kTimeoutMs);
    	_armMotor.config_kI(0, 0, kTimeoutMs);
    	_armMotor.config_kD(0, 0, kTimeoutMs);
    	_armMotor.selectProfileSlot(1,0);
    	_armMotor.config_kF(1, 0.2, kTimeoutMs);
    	_armMotor.config_kP(1, 0.15, kTimeoutMs);
    	_armMotor.config_kI(1, 0, kTimeoutMs);
    	_armMotor.config_kD(1, 0, kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
    	_armMotor.configMotionCruiseVelocity(1500, kTimeoutMs);
    	_armMotor.configMotionAcceleration(2500, kTimeoutMs);
		/* zero the sensor */
    	_armMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
    	
    	//Elevator Up
    	_elevatorMotor.selectProfileSlot(0,0);
    	_elevatorMotor.config_kF(0, 0.0525, kTimeoutMs);
    	_elevatorMotor.config_kP(0, 0.08, kTimeoutMs);
    	_elevatorMotor.config_kI(0, 0, kTimeoutMs);
    	_elevatorMotor.config_kD(0, 0, kTimeoutMs);
    	
    	//Elevator Down
    	_elevatorMotor.selectProfileSlot(1,0);
    	_elevatorMotor.config_kF(1, 0.0525, kTimeoutMs);
    	_elevatorMotor.config_kP(1, 0.08, kTimeoutMs);
    	_elevatorMotor.config_kI(1, 0, kTimeoutMs);
    	_elevatorMotor.config_kD(1, 0, kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
    	_elevatorMotor.configMotionCruiseVelocity(27000, kTimeoutMs);
    	_elevatorMotor.configMotionAcceleration(125000, kTimeoutMs);
		/* zero the sensor */
    	_elevatorMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);

	}
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	gripSolenoidHigh = new Solenoid(3);
    	elevatorSolenoid = new Solenoid(1);
    	wristSolenoid = new Solenoid(2);
    	gripSolenoidLow = new Solenoid(0);

    	driverGamepad = new XboxController(0);
    	operatorGamepad = new XboxController(1);

    	/* take our extra talons and just have them follow the Talons updated in arcadeDrive */
   	
    	setupMotors();
    
    }
    
    public void teleopInit() {
    	setupMotors();
    }

    /**
     * This function is called periodically during operator control
     */
    
    public void teleopPeriodic() {
    	double precisionDriveX;
    	double precisionDriveY;
    	
		if(driverGamepad.getLeftTriggerPressed()){
        	precisionDriveX = 0.5;
        }else{
        	precisionDriveX = 1;
        }
		
		if(driverGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
        	precisionDriveY = 0.5;
        }else{
        	precisionDriveY = 1;
        }
		
        _drive.arcadeDrive(precisionDriveY*driverGamepad.getLeftY(), precisionDriveX*driverGamepad.getLeftX());

    	SmartDashboard.putNumber("Left Drive 1", _frontLeftMotor.getOutputCurrent());
    	SmartDashboard.putNumber("Left Drive 2", _leftSlave1.getOutputCurrent());
    	SmartDashboard.putNumber("Left Drive 3", _leftSlave2.getOutputCurrent());
    	SmartDashboard.putNumber("Left Drive 4", _leftSlave3.getOutputCurrent());
    	SmartDashboard.putNumber("Right Drive 1", _frontRightMotor.getOutputCurrent());
    	SmartDashboard.putNumber("Right Drive 2", _rightSlave1.getOutputCurrent());
    	SmartDashboard.putNumber("Right Drive 3", _rightSlave2.getOutputCurrent());
    	SmartDashboard.putNumber("Right Drive 4", _rightSlave3.getOutputCurrent());
 	
    	
		if(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
			elevatorSolenoid.set(true);
			_elevatorMotor.set(driverGamepad.getRightY());
        }

		
		if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
			/*gripFirm();
			elevatorDown();
			armUp();
			elevatorSolenoid.set(false);
			wristSolenoid.set(true);*/
			wristUp();
        }
		
		if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
			/*gripFirm();
			armDown();
			elevatorUp();
			elevatorSolenoid.set(false);
			wristSolenoid.set(true);*/
			wristDown();
		}
		
		
		if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
			gripFirm();
			armDown();
			elevatorDown();
			elevatorSolenoid.set(false);
			/*flipWrist =3;
			_notifier.startPeriodic(0.005);*/
			wristSolenoid.set(true);


		}
		
		if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
			gripFirm();
			armUp();
			elevatorUp();
			elevatorSolenoid.set(false);
			/*flipWrist =1;
			_notifier.startPeriodic(0.005);*/
			wristSolenoid.set(true);
			//wristSolenoid.set(true);
		}
		
		
		if(operatorGamepad.getLeftTriggerPressed()){
			_intakeMotor.set(Math.pow(operatorGamepad.getLeftTrigger(),2));
        }else if(operatorGamepad.getRightTriggerPressed()) {
        	_intakeMotor.set(-1*operatorGamepad.getRightTrigger());
        	if(armUp == false && elevatorUp == false) {
        		gripLight();
        	}
        }else {
        	_intakeMotor.set(-0.3);
        }
	   
		
		switch(operatorGamepad.getDirectionPad()){
		case UP:
			gripOpen();
			break;
		case DOWN:
			gripFirm();
			break;
		case LEFT:
			gripLight();
			break;
		case RIGHT:
			gripNeutral();
			break;
		case NONE:
		}

		if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
				elevatorSolenoid.set(true);
				elevatorDown();
		}
		if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
				armDown();
				elevatorDown();
				gripFirm();
				elevatorSolenoid.set(false);
				wristSolenoid.set(false);
		}
		
		
		
		//_wristMotor.set(operatorGamepad.getLeftY());
		
    }
    
    public void autonomousInit() {
    	setupMotors();
    	autoLoop=0;
    	//shiftSolenoid.set(true);
    	_frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
    	_frontRightMotor.setSelectedSensorPosition(0, 0, 10);
    	speed =0;
    	step =0;
    }
    public void gripLight() {
		gripSolenoidHigh.set(true);
		gripSolenoidLow.set(false);
    }
    public void gripFirm() {
    	gripSolenoidHigh.set(false);
		gripSolenoidLow.set(false);
		_intakeMotor.set(-0.3);
    }
    public void gripOpen() {
    	gripSolenoidHigh.set(true);
		gripSolenoidLow.set(true);
    }
    public void gripNeutral() {
    	gripSolenoidHigh.set(false);
		gripSolenoidLow.set(true);
        }
        
     	
    public void armUp() {
    	_armMotor.selectProfileSlot(0,0);
    	_armMotor.set(ControlMode.MotionMagic, 6900);
    	armUp = true;
    }
    public void armDown() {
    	_armMotor.selectProfileSlot(1,0);
    	_armMotor.set(ControlMode.MotionMagic, 100);
    	armUp = false;
    }
    
    public void wristUp() {
    	_wristMotor.selectProfileSlot(0,0);
    	_wristMotor.set(ControlMode.MotionMagic, 0);
    	wristUp = true;
    }
    public void wristDown() {
    	_wristMotor.selectProfileSlot(1,0);
    	_wristMotor.set(ControlMode.MotionMagic, 2500);
    	wristUp = false;
    }

    public void elevatorUp() {
    	_elevatorMotor.selectProfileSlot(0,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, 215000);
    	elevatorUp = true;
    }
    public void elevatorDown() {
    	_elevatorMotor.selectProfileSlot(1,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, 1000);
    	elevatorUp = false;
    	
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
    public void testInit() {
    	_leftSlave1.follow(_leftSlave1);
    	_leftSlave2.follow(_leftSlave2);
    	_leftSlave3.follow(_leftSlave3);
    	_rightSlave1.follow(_rightSlave1);
    	_rightSlave2.follow(_rightSlave2);
    	_rightSlave3.follow(_rightSlave3);
    	
    	_elevatorSlave1.follow(_elevatorSlave1);
    	_intakeSlave1.follow(_intakeSlave1);
    	System.out.println("Starting system test. Press A to begin.");
    	testCounter =0;
    }
    
    public void testPeriodic() {
    	if(operatorGamepad.getRawButtonPressed(XboxController.A_BUTTON)) {
    			testCounter++;

	    		if (testCounter >1) {
	    	    	System.out.println("Motor Current: " + (motorCurrentSum/motorCurrentCount) + "A");
	
	    	    	_frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	_frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    		}
	    		
	    		_frontLeftMotor.set(0);
	    		_leftSlave1.set(0);
	    		_leftSlave2.set(0);
	    		_leftSlave3.set(0);
	    		_frontRightMotor.set(0);
	    		_rightSlave1.set(0);
	    		_rightSlave2.set(0);
	    		_rightSlave3.set(0);
	    		_elevatorMotor.set(0);
	    		_elevatorSlave1.set(0);
	    		_armMotor.set(0);
	    		_intakeMotor.set(0);
	    		_intakeSlave1.set(0);
	    		motorCurrentSum = 0;
	    		motorCurrentCount = 0;
    		
    	}

    	
    	switch(testCounter) {
	    	case 1: //Left Drive Motor 1
	    		motorEncoder = _frontLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    	    	_frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering left drive 1. [Desc: " + _frontLeftMotor.getDescription() + ", Name: " +_frontLeftMotor.getName() + " ID: " +_frontLeftMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + _frontLeftMotor.getBusVoltage());	
	    		}
	    		_frontLeftMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _frontLeftMotor.getOutputCurrent();
	    		break;
	    	
	    	case 2: //Left Slave Motor 1
	    		motorEncoder = _frontLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			_frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering left slave 1. [Desc: " + _leftSlave1.getDescription() + ", Name: " +_leftSlave1.getName() + " ID: " +_leftSlave1.getDeviceID());	    		
		    		System.out.println("Voltage: " + _leftSlave1.getBusVoltage());	
	    		}
	    		_leftSlave1.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _leftSlave1.getOutputCurrent();
	    		break;
	    		
	    	case 3: //Left Slave Motor 2
	    		motorEncoder = _frontLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			_frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering left slave 2. [Desc: " + _leftSlave2.getDescription() + ", Name: " +_leftSlave2.getName() + " ID: " +_leftSlave2.getDeviceID());	    		
		    		System.out.println("Voltage: " + _leftSlave2.getBusVoltage());	
	    		}
	    		_leftSlave2.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _leftSlave2.getOutputCurrent();
	    		break;
	    		
	    	case 4: //Left Slave Motor 3
	    		motorEncoder = _frontLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			_frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering left slave 3. [Desc: " + _leftSlave3.getDescription() + ", Name: " +_leftSlave3.getName() + " ID: " +_leftSlave3.getDeviceID());	    		
		    		System.out.println("Voltage: " + _leftSlave3.getBusVoltage());	
	    		}
	    		_leftSlave3.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _leftSlave3.getOutputCurrent();
	    		break;
	    		
	    	
	    	case 5: //Right Drive Motor 1
	    		motorEncoder = _frontRightMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			_frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering front right motor. [Desc: " + _frontRightMotor.getDescription() + ", Name: " +_frontRightMotor.getName() + " ID: " +_frontRightMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + _frontRightMotor.getBusVoltage());	
	    		}
	    		_frontRightMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _frontRightMotor.getOutputCurrent();
	    		break;
	    		
	    	case 6: //Right Slave Motor 1
	    		motorEncoder = _frontRightMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			_frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering right slave 1. [Desc: " + _rightSlave1.getDescription() + ", Name: " +_rightSlave1.getName() + " ID: " +_rightSlave1.getDeviceID());	    		
		    		System.out.println("Voltage: " + _rightSlave1.getBusVoltage());	
	    		}
	    		_rightSlave1.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _rightSlave1.getOutputCurrent();
	    		break;
	    		
	    	case 7: //Right Slave Motor 2
	    		motorEncoder = _frontRightMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			_frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering right slave 2. [Desc: " + _rightSlave2.getDescription() + ", Name: " +_rightSlave2.getName() + " ID: " +_rightSlave2.getDeviceID());	    		
		    		System.out.println("Voltage: " + _rightSlave2.getBusVoltage());	
	    		}
	    		_rightSlave2.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _rightSlave2.getOutputCurrent();
	    		break;
	    		
	    	case 8: //Right Slave Motor 3
	    		motorEncoder = _frontRightMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			_frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering right slave 3. [Desc: " + _rightSlave3.getDescription() + ", Name: " +_rightSlave3.getName() + " ID: " +_rightSlave3.getDeviceID());	    		
		    		System.out.println("Voltage: " + _rightSlave3.getBusVoltage());	
	    		}
	    		_rightSlave3.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _rightSlave3.getOutputCurrent();
	    		break;
	    		
	    	case 9: //Elevator Motor
	    		motorEncoder = _elevatorMotor.getSelectedSensorPosition(0);
	    		if(motorEncoder > 150000) {
	    	    	motorDirection = -1;
	    	    	System.out.println("Reached 150000 ticks, switching direction");
	    		}else if(motorEncoder < 25000) {
	    	    	motorDirection = 1;
	    	    	System.out.println("Reached 25000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering elevator motor. [Desc: " + _elevatorMotor.getDescription() + ", Name: " +_elevatorMotor.getName() + " ID: " +_elevatorMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + _elevatorMotor.getBusVoltage());	
	    		}
	    		_elevatorMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _elevatorMotor.getOutputCurrent();
	    		break;
	    		
	    	case 10: //Elevator Slave 1
	    		motorEncoder = _elevatorMotor.getSelectedSensorPosition(0);
	    		if(motorEncoder > 150000) {
	    	    	motorDirection = -1;
	    	    	System.out.println("Reached 150000 ticks, switching direction");
	    		}else if(motorEncoder < 25000) {
	    	    	motorDirection = 1;
	    	    	System.out.println("Reached 25000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering elevator slave 1. [Desc: " + _elevatorSlave1.getDescription() + ", Name: " +_elevatorSlave1.getName() + " ID: " +_elevatorSlave1.getDeviceID());	    		
		    		System.out.println("Voltage: " + _elevatorSlave1.getBusVoltage());	
	    		}
	    		_elevatorSlave1.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _elevatorSlave1.getOutputCurrent();
	    		break;
	    		
	    	case 11: //Arm Motor
	    		motorEncoder = _armMotor.getSelectedSensorPosition(0);
				wristSolenoid.set(true);
	    		if(motorEncoder > 5000) {
	    			//_armMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection = -1;
	    	    	System.out.println("Reached 5000 ticks, switching direction");
	    		}else if(motorEncoder <1000){
	    	    	motorDirection =1;
	    	    	System.out.println("Reached 1000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering arm motor. [Desc: " + _armMotor.getDescription() + ", Name: " +_armMotor.getName() + " ID: " +_armMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + _armMotor.getBusVoltage());	
	    		}
	    		_armMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _armMotor.getOutputCurrent();
	    		break;
	    		
	    	case 12: //Intake Motor
	    		motorEncoder = _intakeMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			_intakeMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering intake motor. [Desc: " + _intakeMotor.getDescription() + ", Name: " +_intakeMotor.getName() + " ID: " +_intakeMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + _intakeMotor.getBusVoltage());	
	    		}
	    		_intakeMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _intakeMotor.getOutputCurrent();
	    		break;
	    		
	    	case 13: //Intake Slave 1
	    		motorEncoder = _intakeMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			_intakeMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering intake slave 1. [Desc: " + _intakeSlave1.getDescription() + ", Name: " +_intakeSlave1.getName() + " ID: " +_intakeSlave1.getDeviceID());	    		
		    		System.out.println("Voltage: " + _intakeSlave1.getBusVoltage());	
	    		}
	    		_intakeSlave1.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _intakeSlave1.getOutputCurrent();
	    		break;
	    	case 14:
	  	    		System.out.println("Testing done.");
	  	    		testCounter++;
	  	    		break;
	  	   default:
    	}
    
    }
    
}