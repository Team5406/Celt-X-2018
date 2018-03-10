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
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Solenoid;
import org.cafirst.frc.team5406.util.XboxController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;



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
	
	/*Comp Bot has a Victor for ID #9 - Intake, Practice Bot has a Talon*/
	//WPI_VictorSPX _intakeSlave1 = new WPI_VictorSPX(9);
	WPI_TalonSRX _intakeSlave1 = new WPI_TalonSRX(9);

	
	WPI_TalonSRX _elevatorMotor= new WPI_TalonSRX(12); //
	WPI_VictorSPX _elevatorSlave1 = new WPI_VictorSPX(11);
	WPI_TalonSRX _armMotor = new WPI_TalonSRX(13); //arm motor
	WPI_TalonSRX _wristMotor= new WPI_TalonSRX(14);
	
	PowerDistributionPanel pdp = new PowerDistributionPanel();
	public static int PRACTICE_BOT = 0; //jumper to short pins on practice bot
	public static DigitalInput practiceBot = new DigitalInput(PRACTICE_BOT);
		
	double speed = 0;
	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);
	public static AHRS navX = new AHRS(SPI.Port.kMXP);
	private Servo rampCatch = new Servo(11); //DIO0 on MXP
	

    Solenoid elevatorSolenoid;
    Solenoid gripSolenoidLow;
    Solenoid gripSolenoidHigh;
    Solenoid wristSolenoid;
    int flipWrist = 0;
    int gripCounter = 0;
	int autoLoop = 0;
	int step =0;
	int stepA = 0;
	boolean disabled = true;
	boolean elevTestShift = false;
	int[] pdpSlots = {
			4,  //left 1
			0,  //left 2
			2,  //left 3
			1,  //left 4
			11, //right 1
			14, //right 2
			15, //right 3
			13  //right 4
	};
	
	public enum GripState {
	    FIRM,
	    LIGHT,
	    OPEN
	}
	
	public enum ArmState {
	    UP,
	    DOWN
	}
	
	public enum WristState {
	    UP,
	    DOWN
	}
	
	public enum ElevatorState {
	    UP,
	    DOWN
	}
	
	GripState gripState = GripState.FIRM;
	ArmState armState = ArmState.DOWN;
	ElevatorState elevatorState = ElevatorState.DOWN;
	WristState wristState = WristState.UP;
	
	
	boolean armUp = false;
	boolean elevatorUp = false;
	boolean wristUp = false;
	int testCounter = 0;
	double motorCurrentSum = 0;
	int motorCurrentCount = 0;
	long testPeriodicCounter = 0;
	long motorEncoder = 0;
	int motorDirection = 1;
	boolean buttonPress = false;
	int kTimeoutMs = 10;
	boolean isPracticeBot = false;
	
    private XboxController driverGamepad;
    private XboxController operatorGamepad;
    
    boolean needsWristUp = false;
    boolean manualElevator = false;
    boolean manualWrist = false;
    boolean manualArm = false;
	boolean elevatorOverride = false;
	boolean wristOverride = false;
	boolean armOverride = false;
	boolean wristZeroed = false;
	boolean armZeroed = false;
	boolean elevatorZeroed = false;
    
    boolean gripSpin = false;

    class PeriodicRunnable implements java.lang.Runnable {
	    public void run() { 
	    	//shoulder = 155 deg
	    	//wrist = 90 deg
	    /*wrist up
	     * arm to 105
	     * wrist down
	     * arm all the way up
	     */
	    	switch (flipWrist) {
	    	case 1:
		    	if(_armMotor.getSelectedSensorPosition(0) > 5500) {
		    		
		    		wristDown();
		    	}
		    	flipWrist = 2;
		    	break;
	    	case 2:
		    	if(_armMotor.getSelectedSensorPosition(0) < 5500) {
		    		flipWrist = 5;
		    		wristUp();
		    	}
		    	
		    	break;
	    	
	    	case 3:
		    	if(_armMotor.getSelectedSensorPosition(0) < 500) {
		    		
		    		wristUp();
		    	}
		    	flipWrist = 4;
		    	break;
	    	case 4:
		    	if(_armMotor.getSelectedSensorPosition(0) > 500) {
		    		flipWrist = 5;
		    		wristDown();
		    	}
		    	break;
	    	
		    	
	    	}
	    }
	}
	Notifier _notifier = new Notifier(new PeriodicRunnable());
	public void setupMotors() {
		if(disabled) {
			disabled = false;	
			_leftSlave1.follow(_frontLeftMotor);
	    	_leftSlave2.follow(_frontLeftMotor);
	    	_leftSlave3.follow(_frontLeftMotor);
	    	_rightSlave1.follow(_frontRightMotor);
	    	_rightSlave2.follow(_frontRightMotor);
	    	_rightSlave3.follow(_frontRightMotor);
	    	
	    	_frontLeftMotor.configOpenloopRamp(0.25, 10);
	    	_frontRightMotor.configOpenloopRamp(0.25, 10);
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
			_frontLeftMotor.configContinuousCurrentLimit(20, 0);
			//_frontLeftMotor.configPeakCurrentLimit(30, 0);
			//_frontLeftMotor.configPeakCurrentDuration(30, 0);
			_frontLeftMotor.enableCurrentLimit(true);
			_frontRightMotor.configContinuousCurrentLimit(20, 0);
			//_frontRightMotor.configPeakCurrentLimit(30, 0);
			//_frontRightMotor.configPeakCurrentDuration(30, 0);
			_frontRightMotor.enableCurrentLimit(true);
			_armMotor.configContinuousCurrentLimit(40, 0);
			_armMotor.configPeakCurrentLimit(60, 0);
			_armMotor.configPeakCurrentDuration(30, 0);
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
	
	    	_wristMotor.selectProfileSlot(0,0);
	    	_wristMotor.config_kF(0, 1, kTimeoutMs);
	    	_wristMotor.config_kP(0, 0.5, kTimeoutMs);
	    	_wristMotor.config_kI(0, 0, kTimeoutMs);
	    	_wristMotor.config_kD(0, 0, kTimeoutMs);
	    	_wristMotor.selectProfileSlot(1,0);
	    	_wristMotor.config_kF(1, .2, kTimeoutMs);
	    	_wristMotor.config_kP(1, 1, kTimeoutMs);
	    	_wristMotor.config_kI(1, 0.001, kTimeoutMs);
	    	_wristMotor.config_kD(1, 1.5, kTimeoutMs);
			/* set acceleration and vcruise velocity - see documentation */
	    	_wristMotor.configMotionCruiseVelocity(4000, kTimeoutMs);
	    	_wristMotor.configMotionAcceleration(2000, kTimeoutMs);
			/* zero the sensor */
	    	_wristMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
	    	_wristMotor.set(ControlMode.PercentOutput, 0);
	
	    	
	    	_armMotor.selectProfileSlot(0,0);
	    	_armMotor.config_kF(0, 0.4, kTimeoutMs);
	    	_armMotor.config_kP(0, 0.4, kTimeoutMs);
	    	_armMotor.config_kI(0, 0, kTimeoutMs);
	    	_armMotor.config_kD(0, 0, kTimeoutMs);
	    	_armMotor.selectProfileSlot(1,0);
	    	_armMotor.config_kF(1, 0.2, kTimeoutMs);
	    	_armMotor.config_kP(1, 0.15, kTimeoutMs);
	    	_armMotor.config_kI(1, 0, kTimeoutMs);
	    	_armMotor.config_kD(1, 0, kTimeoutMs);
			/* set acceleration and vcruise velocity - see documentation */
	    	_armMotor.configMotionCruiseVelocity(2000, kTimeoutMs);
	    	_armMotor.configMotionAcceleration(2000, kTimeoutMs);
			/* zero the sensor */
	    	_armMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
	    	_armMotor.set(ControlMode.PercentOutput, 0);
	
	    	
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
	    	_elevatorMotor.set(ControlMode.PercentOutput, 0);
	    	
	    	
	    	
	    	_elevatorMotor.configForwardSoftLimitThreshold(216000, 10);
	    	_elevatorMotor.configReverseSoftLimitThreshold(0, 10);
	    	_wristMotor.configForwardSoftLimitThreshold(0, 10);
	    	_wristMotor.configReverseSoftLimitThreshold(-2700, 10);
	    	_armMotor.configForwardSoftLimitThreshold(6900, 10);
	    	_armMotor.configReverseSoftLimitThreshold(0, 10);
	    	_armMotor.configForwardSoftLimitEnable(true, 10);
	    	_armMotor.configReverseSoftLimitEnable(true, 10);
	    	_wristMotor.configForwardSoftLimitEnable(true, 10);
	    	_wristMotor.configReverseSoftLimitEnable(true, 10);
	    	_elevatorMotor.configForwardSoftLimitEnable(true, 10);
	    	_elevatorMotor.configReverseSoftLimitEnable(true, 10);
    	

		}
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

    	driverGamepad = new XboxController(1);
    	operatorGamepad = new XboxController(0);

    	/* take our extra talons and just have them follow the Talons updated in arcadeDrive */
   	

    	isPracticeBot = !practiceBot.get();
    	    	
    	if(isPracticeBot) {
    	System.out.println("I am a practice bot.");  		
    	}
    	else {
    		System.out.println("Identity crisis. I am not a practice bot.");
    	}
    	
    	setupMotors();
    
    }
    public void disabledInit() {
    	disabled = true;
    	_elevatorMotor.set(ControlMode.PercentOutput, 0);
    	_wristMotor.set(ControlMode.PercentOutput, 0);
    	_armMotor.set(ControlMode.PercentOutput, 0);
    }
    
    public void teleopInit() {
    	setupMotors();
		_wristMotor.set(ControlMode.MotionMagic, _wristMotor.getSelectedSensorPosition(0));

    }

    /**
     * This function is called periodically during operator control
     */
    
    public void teleopPeriodic() {
    	gripState = GripState.FIRM;
    	boolean armSet = false;
		boolean elevatorOverrideNew = false;
		boolean wristOverrideNew = false;
		boolean armOverrideNew = false;
		boolean wristSet = false;
		
		if(!wristZeroed) {
			wristOverrideNew = true;
			_wristMotor.set(0.5);
			System.out.print(_wristMotor.getOutputCurrent());
			if(_wristMotor.getOutputCurrent()>15) {
				_wristMotor.setSelectedSensorPosition(150, 0, kTimeoutMs);
				wristUp();
				wristZeroed = true;
				wristSet = true;
				wristOverrideNew = false;
			}
		}
		
		if(!armZeroed) {
			_armMotor.set(-0.5);
			armOverrideNew = true;

			if(_armMotor.getOutputCurrent()>5) {
				_armMotor.setSelectedSensorPosition(-10, 0, kTimeoutMs);
				armDown();
				armZeroed = true;
				armOverrideNew = false;
			}
		}
		
		if(!elevatorZeroed) {
			_elevatorMotor.set(-0.5);
			elevatorOverrideNew = true;
			if(_elevatorMotor.getOutputCurrent()>5) {
				_elevatorMotor.setSelectedSensorPosition(-500, 0, kTimeoutMs);
				elevatorDown();
				elevatorZeroed = true;
				elevatorOverrideNew = false;
			}
		}



    	
		
    	/*******************************
    	 * Driver Controls
    	 ******************************/
    	
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
		
		if(driverGamepad.getButtonHeld(XboxController.BACK_BUTTON) && driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)) {
			rampCatch.setAngle(90);
		}
		
        _drive.arcadeDrive(precisionDriveY*driverGamepad.getLeftY(), precisionDriveX*driverGamepad.getLeftX());

        displayCurrent();
		
		if(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
			if(Math.abs(driverGamepad.getRightY())>0.2 ) {
				elevatorSlow();
				_elevatorMotor.set(driverGamepad.getRightY());
				manualElevator = true;
			}
			
		/***************************
		 * Operator Controls
		 **************************/
			
		}else if(Math.abs(operatorGamepad.getLeftY())>0.2 ) {
				elevatorSlow();
				_elevatorMotor.set(operatorGamepad.getLeftY());
				manualElevator = true;
				if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
					elevatorOverrideNew = true;
				}
		} else if (manualElevator) {
			manualElevator = false;
	    	_elevatorMotor.selectProfileSlot(0,0);
	    	_elevatorMotor.set(ControlMode.MotionMagic, _elevatorMotor.getSelectedSensorPosition(0));
		}


		if(Math.abs(operatorGamepad.getRightY())>0.2 ) {
			_wristMotor.set(operatorGamepad.getRightY());
			manualWrist = true;
			needsWristUp = false;
			wristSet = true;
			if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
				wristOverrideNew = true;
			}
		} else if (manualWrist) {
			manualWrist = false;
			needsWristUp = false;
			wristSet = true;
			_wristMotor.selectProfileSlot(0,0);
			System.out.println("Hold at: " + _wristMotor.getSelectedSensorPosition(0));
			_wristMotor.set(ControlMode.MotionMagic, _wristMotor.getSelectedSensorPosition(0));
		}
		

    	
        if(operatorGamepad.getButtonHeld(XboxController.LEFT_STICK)&&operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
        	_elevatorMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
         }
        
        if(operatorGamepad.getButtonHeld(XboxController.RIGHT_STICK)&&operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
        	_wristMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
         }
		
		if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
			elevatorFast();
			elevatorDown();
			armUp();
			wristSet = true;
			flipWrist =3;
			_notifier.startPeriodic(0.005);
			needsWristUp = false;
        }
		
		if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
			armDown();
			elevatorFast();
			elevatorUp();
			wristDown();
			wristSet = true;
			needsWristUp = false;
		}
		
		
		if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
			armDown();
			elevatorFast();
			elevatorDown();
			//wristDown();
			flipWrist =1;
			_notifier.startPeriodic(0.005);
			wristSet = true;
			//manualWrist = true;
			needsWristUp = false;
		}
		
		if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
			wristUp();
			armUp();
			elevatorFast();
			elevatorSwitchMid();
			flipWrist =3;
			_notifier.startPeriodic(0.005);
			//wristDown();
			wristSet = true;
			needsWristUp = false;
		}
		
		
		if(operatorGamepad.getLeftTriggerPressed() && operatorGamepad.getButtonHeld(XboxController.START_BUTTON)){
			_intakeMotor.set(Math.pow(operatorGamepad.getLeftTrigger(),2));
        }else if(operatorGamepad.getLeftTriggerPressed()){
        	if(armUp == false) {
        		wristSlightDown();
        		wristSet = true;
        		needsWristUp = true;
        		if(_wristMotor.getSelectedSensorPosition(0) < -350) {
        			_intakeMotor.set(0.7);
        		}
        	}else {
        		_intakeMotor.set(0.5);
        	}
        }else if(operatorGamepad.getRightTriggerPressed()) {
        	_intakeMotor.set(-1*operatorGamepad.getRightTrigger());
        }else {
        	if(!gripSpin) {
        		_intakeMotor.set(0);
        	}
        }
	   
		if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
			if(elevatorUp == false && armUp == false) {
				gripState = GripState.LIGHT;
			}
			wristDown();
			needsWristUp = true;
			wristSet = true;
		}
		
		if (operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)) {
			if(_armMotor.getSelectedSensorPosition(0) < 500) {
				wristSlightUp();
				wristSet = true;
				needsWristUp = true;
			}
		}
		
		if(operatorGamepad.getButtonHeld(XboxController.BACK_BUTTON)){
			gripState = GripState.OPEN;
		}
		
		if(!wristSet && needsWristUp) {
			if(_armMotor.getSelectedSensorPosition(0) < 500) {
				needsWristUp = false;
				wristUp();
			}
		}
		
		
		switch(operatorGamepad.getDirectionPad()){
		case UP:
			_armMotor.set(0.6);
			manualArm = true;
			break;
		case DOWN:
			_armMotor.set(-0.4);
			manualArm = true;
			break;
		case LEFT:
			if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
				
			}
			break;
		case RIGHT:
		case NONE:
			if(manualArm) {
				manualArm = false;
				_armMotor.selectProfileSlot(0,0);
				_armMotor.set(ControlMode.MotionMagic, _armMotor.getSelectedSensorPosition(0));
			}
		}


		switch(gripState) {
		case FIRM:
			gripFirm();
			break;
		case LIGHT:
			gripLight();
			break;
		case OPEN:
			gripOpen();
			break;
		}
		
		if (armOverrideNew != armOverride) {
			_armMotor.configForwardSoftLimitEnable(!armOverrideNew, 10);
			_armMotor.configReverseSoftLimitEnable(!armOverrideNew, 10);
			armOverride = armOverrideNew;
		}
		
		if (elevatorOverrideNew != elevatorOverride) {
			_elevatorMotor.configForwardSoftLimitEnable(!elevatorOverrideNew, 10);
			_elevatorMotor.configReverseSoftLimitEnable(!elevatorOverrideNew, 10);
			elevatorOverride = elevatorOverrideNew;
		}
		
		if (wristOverrideNew != wristOverride) {
			_wristMotor.configForwardSoftLimitEnable(!wristOverrideNew, 10);
			_wristMotor.configReverseSoftLimitEnable(!wristOverrideNew, 10);
			wristOverride = wristOverrideNew;
		}
		
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
    
    
    public void elevatorFast() {
    	elevatorSolenoid.set(false);
    }
    
    public void elevatorSlow() {
    	elevatorSolenoid.set(true);
    }
    public void gripLight() {
		gripSolenoidHigh.set(true);
		gripSolenoidLow.set(false);
		gripSpin = false;
    }
    public void gripFirm() {
    	gripSolenoidHigh.set(false);
		gripSolenoidLow.set(false);
    	/*if(armUp == false && wristUp == false && elevatorUp == false) {
    		wristSlightUp();
    	}*/
		gripSpin = false;
    }
    public void gripOpen() {
    	gripSolenoidHigh.set(true);
		gripSolenoidLow.set(true);
		//_intakeMotor.set(-0.7);
		gripSpin = true;
    }
    public void gripNeutral() {
    	gripSolenoidHigh.set(false);
		gripSolenoidLow.set(true);
		gripSpin = false;
        }
        
     	
    public void armUp() {
    	_armMotor.selectProfileSlot(0,0);
    	_armMotor.set(ControlMode.MotionMagic, 6850);
    	armUp = true;
    }
    public void armDown() {
    	_armMotor.selectProfileSlot(1,0);
    	_armMotor.set(ControlMode.MotionMagic, 100);
    	armUp = false;
    }
    
    public void wristSlightUp() {
    	_wristMotor.selectProfileSlot(0,0);
    	_wristMotor.set(ControlMode.MotionMagic, -2000);
    	wristUp = false;
    	manualWrist = false;
    }
    public void wristSlightDown() {
    	_wristMotor.selectProfileSlot(1,0);
    	_wristMotor.set(ControlMode.MotionMagic, -475);
    	wristUp = true;
    	manualWrist = false;
    }
    public void wristUp() {
    	_wristMotor.selectProfileSlot(0,0);
    	_wristMotor.set(ControlMode.MotionMagic, 0);
    	wristUp = true;
    	manualWrist = false;
    }
    public void wristDown() {
    	_wristMotor.selectProfileSlot(0,0);
    	_wristMotor.set(ControlMode.MotionMagic, -2700);
    	wristUp = false;
    	manualWrist = false;
    }

    public void elevatorUp() {
    	_elevatorMotor.selectProfileSlot(0,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, 215000);
    	elevatorUp = true;
    }
    public void elevatorSwitchMid() {
    	_elevatorMotor.selectProfileSlot(0,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, 150000);
    	elevatorUp = true;
    }

    public void elevatorDown() {
    	_elevatorMotor.selectProfileSlot(1,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, 1000);
    	elevatorUp = false;
    	
    }
    
    public void displayCurrent() {
    	   	SmartDashboard.putNumber("Left Drive 1", pdp.getCurrent(pdpSlots[0]));
        	SmartDashboard.putNumber("Left Drive 2", pdp.getCurrent(pdpSlots[1]));
        	SmartDashboard.putNumber("Left Drive 3", pdp.getCurrent(pdpSlots[2]));
        	SmartDashboard.putNumber("Left Drive 4", pdp.getCurrent(pdpSlots[3]));
        	SmartDashboard.putNumber("Right Drive 1", pdp.getCurrent(pdpSlots[4]));
        	SmartDashboard.putNumber("Right Drive 2", pdp.getCurrent(pdpSlots[5]));
        	SmartDashboard.putNumber("Right Drive 3", pdp.getCurrent(pdpSlots[6]));
        	SmartDashboard.putNumber("Right Drive 4", pdp.getCurrent(pdpSlots[7]));
    }
   public void autonomousPeriodic() {
    	
    	double positionLeft = _frontLeftMotor.getSelectedSensorPosition(0);
    	double positionRight = _frontRightMotor.getSelectedSensorPosition(0);
    	
    	switch(step) {
    	case 0:
    		wristDown();
    		elevatorUp();
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
    	_drive.tankDrive(speed,speed);
    	/*switch(stepA) {
    	case 0:
	    	switch(step) {
	    	case 0:
	    		wristUp();
	    		elevatorUp();
	    		step = 1;
	    	case 1:
	    		speed+=0.015;
	    		if (positionLeft>16133) {
	    			step =1;
	    		}
	    		break;
	    	case 2:
	    		speed = 0.7;
	    		if(positionLeft > 190000) {
	    			step =2;
	    		}
	    		break;
	    	case 3:
	    		speed-=0.015;
	    		if(positionLeft > 207000 || speed <=0.1) {
	    			step =3;
	    		}
	    		break;
	    	case 4:
	    		
	    		if(positionLeft > 208000) {
	    			speed = -0.1;
	    		}else {
	    			speed=0;
	    			step = 5;
	    			stepA = 1;
	    		}
	    		break;   
	    	}
	    	_drive.tankDrive(speed,speed);
    	break;
    	case 1:
    		if(positionLeft < 250000) {
    			_drive.tankDrive(0.3,0.5);
    		}else {
            	_drive.tankDrive(0,0);
            	stepA = 2;
    		}
    		break;
    	case 2:
    		_intakeMotor.set(0.5);
    		break;
    	}*/
    	
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
    	testPeriodicCounter++;
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
	    		_wristMotor.set(0);
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
	    		motorCurrentSum += pdp.getCurrent(pdpSlots[0]);
	    		break;
	    	
	    	case 2: //Left Slave Motor 1
	    		motorEncoder = _frontLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			elevatorSolenoid.set(false);
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
	    		motorCurrentSum += pdp.getCurrent(pdpSlots[1]);
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
	    		motorCurrentSum += pdp.getCurrent(pdpSlots[2]);
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
	    		motorCurrentSum += pdp.getCurrent(pdpSlots[3]);
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
	    		motorCurrentSum += pdp.getCurrent(pdpSlots[4]);
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
	    		motorCurrentSum += pdp.getCurrent(pdpSlots[5]);
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
	    		motorCurrentSum += pdp.getCurrent(pdpSlots[6]);
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
	    		motorCurrentSum += pdp.getCurrent(pdpSlots[7]);
	    		break;
	    	case 9: //Wrist Motor
	    		motorEncoder = _wristMotor.getSelectedSensorPosition(0);
	    		if(motorEncoder > 2000) {
	    	    	motorDirection = -1;
	    	    	System.out.println("Reached 2500 ticks, switching direction");
	    		}else if(motorEncoder < 1000) {
	    	    	motorDirection = 1;
	    	    	System.out.println("Reached 100 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering wrist motor. [Desc: " + _wristMotor.getDescription() + ", Name: " +_wristMotor.getName() + " ID: " +_wristMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + _wristMotor.getBusVoltage());	
	    		}
	    		_wristMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _wristMotor.getOutputCurrent();
	    		break;
	    	case 10: //Elevator Motor
	    		elevatorSolenoid.set(false);
	    		motorEncoder = _elevatorMotor.getSelectedSensorPosition(0);
	    		if(motorEncoder > 150000) {
	    			elevatorSolenoid.set(!elevTestShift);
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
	    		_elevatorMotor.set(motorDirection*.6); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _elevatorMotor.getOutputCurrent();
	    		break;
	    		
	    	case 11: //Elevator Slave 1
	    		elevatorSolenoid.set(false);
	    		motorEncoder = _elevatorMotor.getSelectedSensorPosition(0);
	    		if(motorEncoder > 150000) {
	    			elevatorSolenoid.set(!elevTestShift);
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
	    		_elevatorSlave1.set(motorDirection*.6); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _elevatorSlave1.getOutputCurrent();
	    		break;
	    		
	    	case 12: //Arm Motor
	    		motorEncoder = _armMotor.getSelectedSensorPosition(0);
				wristSolenoid.set(true);
	    		if(motorEncoder > 3000) {
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
	    		_armMotor.set(motorDirection*.4); 
	    		motorCurrentCount++;
	    		motorCurrentSum += _armMotor.getOutputCurrent();
	    		break;
	    		
	    	case 13: //Intake Motor
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
	    		
	    	case 14: //Intake Slave 1
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
	    	case 15:
	    		//open, wait 1s, close firm, wait 1s, close light, wait 1s, repeat
	    		if(testPeriodicCounter % 150 ==100) {
	    			gripLight();
	    		}else if(testPeriodicCounter % 150 ==50) {
	    			gripFirm();
	    		}else if(testPeriodicCounter % 150 ==0) {
	    			gripOpen();
	    			testPeriodicCounter=0;
	    		}
	    		break;
	    	case 16:
	  	    		System.out.println("Testing done.");
	  	    		testCounter++;
	  	    		break;
	  	   default:
    	}
    
    }
   
}