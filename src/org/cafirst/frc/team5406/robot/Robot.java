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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Solenoid;



import org.cafirst.frc.team5406.util.XboxController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import org.cafirst.frc.team5406.subsystems.*;
import org.cafirst.frc.team5406.auto.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 * 
 */
public class Robot extends IterativeRobot {

	private Intake robotIntake = new Intake();
	private Drive robotDrive = new Drive();
	
	
	//PowerDistributionPanel pdp = new PowerDistributionPanel();
	public static int PRACTICE_BOT = 0; //jumper to short pins on practice bot
	public static DigitalInput practiceBot = new DigitalInput(PRACTICE_BOT);
		
	double speed = 0;
	DifferentialDrive _drive = new DifferentialDrive(robotDrive._frontLeftMotor, robotDrive._frontRightMotor);


    int flipWrist = 0;
    int gripCounter = 0;
	int autoLoop = 0;
	int step =0;
	int stepA = 0;
	boolean holdPosition = false;
	boolean elevTestShift = false;
	
	
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
	
	
	
	int testCounter = 0;
	double motorCurrentSum = 0;
	int motorCurrentCount = 0;
	long testPeriodicCounter = 0;
	long motorEncoder = 0;
	int motorDirection = 1;
	boolean buttonPress = false;
	boolean isPracticeBot = false;

	
    private XboxController driverGamepad;
    private XboxController operatorGamepad;
    
   
	boolean elevatorOverride = false;
	boolean wristOverride = false;
	boolean armOverride = false;

    
	private AutonomousRoutine selectedRoutine;
	private SendableChooser<Object> autonomousSelector = new SendableChooser<>();

    
    int rumbleTime =0 ; 
	
    class PeriodicRunnable implements java.lang.Runnable {
	    public void run() { 

	    	int armPos;
	    	
	    	switch (flipWrist) {
	    	case 1:
	    		robotIntake._wristMotor.selectProfileSlot(0,0);
	    		armPos = robotIntake.getArmPosition();
	    		robotIntake._wristMotor.set(ControlMode.MotionMagic, robotIntake.wristFlipPosDown(armPos));
	    		if(armPos < 100) {
	    			flipWrist = 5;
	    		}
	    		break;
	    	case 3:
	    		robotIntake._wristMotor.selectProfileSlot(0,0);
	    		armPos = robotIntake.getArmPosition();
	    		robotIntake._wristMotor.set(ControlMode.MotionMagic, robotIntake.wristFlipPosUp(armPos));
	    		if(armPos > Constants.ARM_UP - 100) {
	    			flipWrist = 5;
	    		}
	    		break;
	    		
	    	}

	    }
	}
	Notifier _notifier = new Notifier(new PeriodicRunnable());

	
	
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	


    	driverGamepad = new XboxController(1);
    	operatorGamepad = new XboxController(0);

    	/* take our extra talons and just have them follow the Talons updated in arcadeDrive */
    	selectedRoutine = new DriveStraight(robotDrive, robotIntake);
    	autonomousSelector.addObject("0 - Do Nothing", new DoNothing());
    	autonomousSelector.addObject("1 - Drive Straight", new DriveStraight(robotDrive, robotIntake));
    	autonomousSelector.addObject("2 - Front Switch", new AutoSwitchFront(robotDrive, robotIntake));
    	autonomousSelector.addObject("3 - Scale Auto Right", new AutoScaleRight(robotDrive, robotIntake));
    	//autonomousSelector.addDefault("4 - Scale/Switch Auto Right", new AutoScaleSwitchRight(robotDrive, robotIntake));
    	//autonomousSelector.addDefault("5 - Turn to Angle", new TurnToAngle(robotDrive, robotIntake));
    	autonomousSelector.addObject("6 - Switch with Scale Prep", new AutoSwitchScalePrep(robotDrive, robotIntake));
    	autonomousSelector.addObject("7 - Near Scale OR Switch", new AutoNearScaleOrSwitch(robotDrive, robotIntake));
    	autonomousSelector.addDefault("8 - Side Scale", new AutoSideRight(robotDrive, robotIntake));
    	SmartDashboard.putData("Autonomous", autonomousSelector);

    	
    	
    	robotIntake.setupMotors();
    	//robotIntake.zeroMotors();
    	robotDrive.setupMotors();
    }
	@Override
    public void disabledPeriodic(){  
    	SmartDashboard.putNumber("cubeDistance", robotIntake.cubeDistance());
		selectedRoutine = (AutonomousRoutine) autonomousSelector.getSelected();
		SmartDashboard.putString("Selected Autonomous", selectedRoutine.getName());
		//SmartDashboard.putNumber("Heading", Constants.navX.getYaw());
		
    }
	
    public void disabledInit() {
    	selectedRoutine.end();
    	robotDrive.disabled = true;
    	robotIntake.disabled = true;
    	robotIntake.rampHold();
    	robotIntake._elevatorMotor.set(ControlMode.PercentOutput, 0);
    	robotIntake._wristMotor.set(ControlMode.PercentOutput, 0);
    	robotIntake._armMotor.set(ControlMode.PercentOutput, 0);
    	
    }
    
    public void teleopInit() {
    	Constants.navX.zeroYaw();
    	robotIntake.setupMotors();
    	robotDrive.setupMotors();
    	robotIntake._wristMotor.set(ControlMode.MotionMagic, robotIntake._wristMotor.getSelectedSensorPosition(0));
    	selectedRoutine.end();
    }

    /**
     * This function is called periodically during operator control
     */
    
    public void teleopPeriodic() {
    	
    	gripState = GripState.FIRM;
    	//boolean armSet = false;
		boolean elevatorOverrideNew = false;
		boolean wristOverrideNew = false;
		boolean armOverrideNew = false;
		boolean wristSet = false;
		
		long elevatorPos = robotIntake.getElevatorPosition();
		long armPos = robotIntake.getArmPosition();
		// long wristPos = robotIntake._wristMotor.getSelectedSensorPosition(0);
		
		
		double cubeDistance = robotIntake.cubeDistance();
		
    	SmartDashboard.putNumber("cubeDistance", cubeDistance);
    	rumbleTime++;
    	if(cubeDistance < 15 && rumbleTime < 40) {
    		operatorGamepad.setRumble(RumbleType.kLeftRumble, 1);
    		operatorGamepad.setRumble(RumbleType.kRightRumble, 1);
    		driverGamepad.setRumble(RumbleType.kLeftRumble, 1);
    		driverGamepad.setRumble(RumbleType.kRightRumble, 1);
    	}else if(cubeDistance >= 15) {
    		operatorGamepad.setRumble(RumbleType.kLeftRumble, 0);
    		operatorGamepad.setRumble(RumbleType.kRightRumble, 0);
    		driverGamepad.setRumble(RumbleType.kLeftRumble, 0);
    		driverGamepad.setRumble(RumbleType.kRightRumble, 0);
    		rumbleTime = 0;
    	}else {
    		operatorGamepad.setRumble(RumbleType.kLeftRumble, 0);
    		operatorGamepad.setRumble(RumbleType.kRightRumble, 0);
    		driverGamepad.setRumble(RumbleType.kLeftRumble, 0);
    		driverGamepad.setRumble(RumbleType.kRightRumble, 0);
    	}
    	
		
    	/*******************************
    	 * Driver Controls
    	 ******************************/
    	
    	double precisionDriveX;
    	double precisionDriveY;

		
		if(driverGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
        	precisionDriveY = 0.5;
        	precisionDriveX = 0.5;
        }else{
        	precisionDriveY = 1;
        	precisionDriveX = 1;
        }
		
		if(driverGamepad.getButtonHeld(XboxController.START_BUTTON) && (driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) || driverGamepad.getRightTriggerPressed() )) {
			robotIntake.rampRelease();
		}
		

			_drive.arcadeDrive(precisionDriveX*driverGamepad.getLeftX(), precisionDriveY*driverGamepad.getLeftY());
        //displayCurrent();
		
		if(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) || driverGamepad.getRightTriggerPressed()){
			robotIntake.compressor.stop();
			if(Math.abs(driverGamepad.getRightY())>0.2 ) {
				robotIntake.elevatorSlow();
				robotIntake._elevatorMotor.set(driverGamepad.getRightY());
				robotIntake.manualElevator = true;
			}
		} else if(driverGamepad.getLeftTriggerPressed()){
				robotIntake.compressor.stop();
				if(Math.abs(driverGamepad.getRightY())>0.2 ) {
					robotIntake.elevatorFast();
					robotIntake._elevatorMotor.set(driverGamepad.getRightY());
					robotIntake.manualElevator = true;
				}
			
		/***************************
		 * Operator Controls
		 **************************/
			
		}else if(Math.abs(operatorGamepad.getLeftY())>0.2 ) {
			robotIntake.elevatorSlow();
			robotIntake._elevatorMotor.set(operatorGamepad.getLeftY());
				robotIntake.manualElevator = true;
				if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
					elevatorOverrideNew = true;
				}
		} else if (robotIntake.manualElevator) {
			robotIntake.manualElevator = false;
			robotIntake._elevatorMotor.selectProfileSlot(0,0);
			robotIntake._elevatorMotor.set(ControlMode.MotionMagic, robotIntake.getElevatorPosition());
		} else {
			robotIntake.compressor.start();
		}


		if(Math.abs(operatorGamepad.getRightY())>0.2 ) {
			robotIntake._wristMotor.set(operatorGamepad.getRightY());
			robotIntake.manualWrist = true;
			robotIntake.needsWristUp = false;
			wristSet = true;
			if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
				wristOverrideNew = true;
			}
		} else if (robotIntake.manualWrist) {
			robotIntake.manualWrist = false;
			robotIntake.needsWristUp = false;
			wristSet = true;
			robotIntake._wristMotor.selectProfileSlot(0,0);
			System.out.println("Hold at: " + robotIntake.getWristPosition());
			robotIntake._wristMotor.set(ControlMode.MotionMagic, robotIntake.getWristPosition());
		}
		

    	
        if(operatorGamepad.getButtonHeld(XboxController.LEFT_STICK)&&operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
        	robotIntake.setElevatorPosition(0);
         }
        
        if(operatorGamepad.getButtonHeld(XboxController.RIGHT_STICK)&&operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
        	robotIntake.setWristPosition(0);
        }
		
		if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
			robotIntake.elevatorFast();
			robotIntake.elevatorDown();
			robotIntake.armUp();
			wristSet = true;
			flipWrist =3;
			_notifier.startPeriodic(0.005);
			robotIntake.needsWristUp = false;
        }
		
		if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
			robotIntake.armDown();
			robotIntake.elevatorFast();
			robotIntake.elevatorUp();
			//wristDown();
			flipWrist =1;
			_notifier.startPeriodic(0.005);
			wristSet = true;
			//manualWrist = true;
			robotIntake.needsWristUp = false;
		}
		
		
		if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
			robotIntake.armDown();
			robotIntake.elevatorFast();
			robotIntake.elevatorDown();
			//wristDown();
			flipWrist =1;
			_notifier.startPeriodic(0.005);
			wristSet = true;
		
			//manualWrist = true;
			robotIntake.needsWristUp = false;
		}
		
		if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
			robotIntake.wristUp();
			robotIntake.armUp();
			robotIntake.elevatorFast();
			robotIntake.elevatorSwitchMid();
			flipWrist =3;
			_notifier.startPeriodic(0.005);
			//wristDown();
			wristSet = true;
			robotIntake.needsWristUp = false;
		}
		
		
		if(operatorGamepad.getLeftTriggerPressed() && operatorGamepad.getButtonHeld(XboxController.START_BUTTON)){
			robotIntake.spinIntake(Math.pow(operatorGamepad.getLeftTrigger(),2)*660);
        }else if(operatorGamepad.getLeftTriggerPressed()){
        	if(armPos < Constants.ARM_DOWN_THRESHOLD) {
        		robotIntake.wristSlightDown();
        		wristSet = true;
        		robotIntake.needsWristUp = true;
        		if(robotIntake.getWristPosition() < -350) {
        			robotIntake.spinIntake(500);
        		}
        	}else if (armPos > Constants.ARM_UP_THRESHOLD && robotIntake.getWristPosition() < Constants.WRIST_UP_SHOT_THRESHOLD) {
        		robotIntake.spinIntake(1000);
        	}else {
        		robotIntake.spinIntake(operatorGamepad.getLeftTrigger()*175);
        	}
        }else if(operatorGamepad.getRightTriggerPressed()) {
        	robotIntake.spinIntake(-1*operatorGamepad.getRightTrigger()*1000);
        }else {
        	if(!robotIntake.gripSpin) {
        		robotIntake.spinIntake(0);
        	}
        }
	   
		if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
			if(elevatorPos < Constants.ELEVATOR_DOWN_THRESHOLD && armPos < Constants.ARM_DOWN_THRESHOLD) {
				gripState = GripState.LIGHT;
			}
			if(armPos < Constants.ARM_DOWN_THRESHOLD) {
				robotIntake.wristDown();
				robotIntake.needsWristUp = true;
			}else if(armPos > Constants.ARM_UP_THRESHOLD) {
				robotIntake.wristUpShot();
				robotIntake.needsWristUp = false;
			}
			wristSet = true;
			robotIntake.wristOut = true;
		}else {
			robotIntake.wristOut = false;
		}
		
		if (operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)) {
			if(armPos < Constants.ARM_DOWN_THRESHOLD) {
				robotIntake.wristSlightUp();
				wristSet = true;
				robotIntake.needsWristUp = true;
			}
		}
		
		if(operatorGamepad.getButtonHeld(XboxController.BACK_BUTTON)){
			gripState = GripState.OPEN;
		}
		
		if(!wristSet && robotIntake.needsWristUp) {
			flipWrist = 5;
			if(armPos < Constants.ARM_DOWN_THRESHOLD) {
				robotIntake.needsWristUp = false;
				robotIntake.wristUp();
			}
		}
		
		
		switch(operatorGamepad.getDirectionPad()){
		case UP:
			robotIntake._armMotor.set(0.6);
			robotIntake.manualArm = true;
			if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
				armOverrideNew = true;
			}
			break;
		case DOWN:
			robotIntake._armMotor.set(-0.4);
			robotIntake.manualArm = true;
			if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
				armOverrideNew = true;
			}
			break;
		case LEFT:
			if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
				robotIntake.setArmPosition(0);
			}
			break;
		case RIGHT:
		case NONE:
			if(robotIntake.manualArm) {
				robotIntake.manualArm = false;
				robotIntake._armMotor.selectProfileSlot(0,0);
				robotIntake._armMotor.set(ControlMode.MotionMagic, robotIntake.getArmPosition());
			}
		}


		switch(gripState) {
		case FIRM:
			robotIntake.gripFirm();
			break;
		case LIGHT:
			robotIntake.gripLight();
			break;
		case OPEN:
			robotIntake.gripOpen();
			break;
		}
		
		if (armOverrideNew != armOverride) {
			robotIntake._armMotor.configForwardSoftLimitEnable(!armOverrideNew, Constants.kTimeoutMs);
			robotIntake._armMotor.configReverseSoftLimitEnable(!armOverrideNew, Constants.kTimeoutMs);
			armOverride = armOverrideNew;
		}
		
		if (elevatorOverrideNew != elevatorOverride) {
			robotIntake._elevatorMotor.configForwardSoftLimitEnable(!elevatorOverrideNew, Constants.kTimeoutMs);
			robotIntake._elevatorMotor.configReverseSoftLimitEnable(!elevatorOverrideNew, Constants.kTimeoutMs);
			elevatorOverride = elevatorOverrideNew;
		}
		
		if (wristOverrideNew != wristOverride) {
			robotIntake._wristMotor.configForwardSoftLimitEnable(!wristOverrideNew, Constants.kTimeoutMs);
			robotIntake._wristMotor.configReverseSoftLimitEnable(!wristOverrideNew, Constants.kTimeoutMs);
			wristOverride = wristOverrideNew;
		}
		
		if(armPos > Constants.ARM_UP_THRESHOLD) {
			robotIntake._wristMotor.configReverseSoftLimitThreshold(Constants.WRIST_THRESHOLD_UP_REVERSE, Constants.kTimeoutMs);
			robotIntake._wristMotor.configForwardSoftLimitThreshold(Constants.WRIST_THRESHOLD_UP_FORWARD, Constants.kTimeoutMs);
		} else {
			robotIntake._wristMotor.configReverseSoftLimitThreshold(Constants.WRIST_THRESHOLD_DOWN_REVERSE, Constants.kTimeoutMs);
			robotIntake._wristMotor.configForwardSoftLimitThreshold(Constants.WRIST_THRESHOLD_DOWN_FORWARD, Constants.kTimeoutMs);
		}
		
    }
    
   
   
    
    public void displayCurrent() {
    	   /*	SmartDashboard.putNumber("Left Drive 1", pdp.getCurrent(Constants.pdpSlots[0]));
        	SmartDashboard.putNumber("Left Drive 2", pdp.getCurrent(Constants.pdpSlots[1]));
        	SmartDashboard.putNumber("Left Drive 3", pdp.getCurrent(Constants.pdpSlots[2]));
        	SmartDashboard.putNumber("Left Drive 4", pdp.getCurrent(Constants.pdpSlots[3]));
        	SmartDashboard.putNumber("Right Drive 1", pdp.getCurrent(Constants.pdpSlots[4]));
        	SmartDashboard.putNumber("Right Drive 2", pdp.getCurrent(Constants.pdpSlots[5]));
        	SmartDashboard.putNumber("Right Drive 3", pdp.getCurrent(Constants.pdpSlots[6]));
        	SmartDashboard.putNumber("Right Drive 4", pdp.getCurrent(Constants.pdpSlots[7]));*/
    }
    public void autonomousInit() {
		SmartDashboard.putString("Robot Status", "Auto");
		selectedRoutine.init();
    }
    
   public void autonomousPeriodic() {
		//SmartDashboard.putNumber("Heading", Constants.navX.getYaw());

	   selectedRoutine.periodic();
    }
   
   
	
    public void testInit() {
    	selectedRoutine.end();
    	robotDrive._leftSlave1.follow(robotDrive._leftSlave1);
    	robotDrive._leftSlave2.follow(robotDrive._leftSlave2);
    	robotDrive._leftSlave3.follow(robotDrive._leftSlave3);
    	robotDrive._rightSlave1.follow(robotDrive._rightSlave1);
    	robotDrive._rightSlave2.follow(robotDrive._rightSlave2);
    	robotDrive._rightSlave3.follow(robotDrive._rightSlave3);
    	
    	System.out.println("Starting system test. Press A to begin.");
    	testCounter =0;
    }
    
    public void testPeriodic() {
    	testPeriodicCounter++;
    	if(operatorGamepad.getRawButtonPressed(XboxController.A_BUTTON)) {
    			testCounter++;

	    		if (testCounter >1) {
	    	    	System.out.println("Motor Current: " + (motorCurrentSum/motorCurrentCount) + "A");
	
	    	    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    		}
	    		
	    		robotDrive._frontLeftMotor.set(0);
	    		robotDrive._leftSlave1.set(0);
	    		robotDrive._leftSlave2.set(0);
	    		robotDrive._leftSlave3.set(0);
	    		robotDrive._frontRightMotor.set(0);
	    		robotDrive._rightSlave1.set(0);
	    		robotDrive._rightSlave2.set(0);
	    		robotDrive._rightSlave3.set(0);
	    		robotIntake._elevatorMotor.set(0);
	    		robotIntake._elevatorSlave1.set(0);
	    		robotIntake._armMotor.set(0);
	    		robotIntake._intakeLeftMotor.set(0);
	    		robotIntake._intakeRightMotor.set(0);
	    		robotIntake._wristMotor.set(0);
	    		motorCurrentSum = 0;
	    		motorCurrentCount = 0;
    		
    	}

    	
    	switch(testCounter) {
	    	case 1: //Left Drive Motor 1
	    		motorEncoder = robotDrive._frontLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	//motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering left drive 1. [Desc: " + robotDrive._frontLeftMotor.getDescription() + ", Name: " +robotDrive._frontLeftMotor.getName() + " ID: " +robotDrive._frontLeftMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotDrive._frontLeftMotor.getBusVoltage());	
	    		}
	    		robotDrive._frontLeftMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		//motorCurrentSum += pdp.getCurrent(Constants.pdpSlots[0]);
	    		break;
	    	
	    	case 2: //Left Slave Motor 1
	    		motorEncoder = robotDrive._frontLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotIntake.shiftElevator(false);
	    			robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	//motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering left slave 1. [Desc: " + robotDrive._leftSlave1.getDescription() + ", Name: " +robotDrive._leftSlave1.getName() + " ID: " +robotDrive._leftSlave1.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotDrive._leftSlave1.getBusVoltage());	
	    		}
	    		robotDrive._leftSlave1.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		//motorCurrentSum += pdp.getCurrent(Constants.pdpSlots[1]);
	    		break;
	    		
	    	case 3: //Left Slave Motor 2
	    		motorEncoder = robotDrive._frontLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	//motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering left slave 2. [Desc: " + robotDrive._leftSlave2.getDescription() + ", Name: " +robotDrive._leftSlave2.getName() + " ID: " +robotDrive._leftSlave2.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotDrive._leftSlave2.getBusVoltage());	
	    		}
	    		robotDrive._leftSlave2.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		//motorCurrentSum += pdp.getCurrent(Constants.pdpSlots[2]);
	    		break;
	    		
	    	case 4: //Left Slave Motor 3
	    		motorEncoder = robotDrive._frontLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	//motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering left slave 3. [Desc: " + robotDrive._leftSlave3.getDescription() + ", Name: " +robotDrive._leftSlave3.getName() + " ID: " +robotDrive._leftSlave3.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotDrive._leftSlave3.getBusVoltage());	
	    		}
	    		robotDrive._leftSlave3.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		//motorCurrentSum += pdp.getCurrent(Constants.pdpSlots[3]);
	    		break;
	    		
	    	
	    	case 5: //Right Drive Motor 1
	    		motorEncoder = robotDrive._frontRightMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	//motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering front right motor. [Desc: " + robotDrive._frontRightMotor.getDescription() + ", Name: " +robotDrive._frontRightMotor.getName() + " ID: " +robotDrive._frontRightMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotDrive._frontRightMotor.getBusVoltage());	
	    		}
	    		robotDrive._frontRightMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		//motorCurrentSum += pdp.getCurrent(Constants.pdpSlots[4]);
	    		break;
	    		
	    	case 6: //Right Slave Motor 1
	    		motorEncoder = robotDrive._frontRightMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	//motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering right slave 1. [Desc: " + robotDrive._rightSlave1.getDescription() + ", Name: " +robotDrive._rightSlave1.getName() + " ID: " +robotDrive._rightSlave1.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotDrive._rightSlave1.getBusVoltage());	
	    		}
	    		robotDrive._rightSlave1.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		//motorCurrentSum += pdp.getCurrent(Constants.pdpSlots[5]);
	    		break;
	    		
	    	case 7: //Right Slave Motor 2
	    		motorEncoder = robotDrive._frontRightMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	//motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering right slave 2. [Desc: " + robotDrive._rightSlave2.getDescription() + ", Name: " +robotDrive._rightSlave2.getName() + " ID: " +robotDrive._rightSlave2.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotDrive._rightSlave2.getBusVoltage());	
	    		}
	    		robotDrive._rightSlave2.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		//motorCurrentSum += pdp.getCurrent(Constants.pdpSlots[6]);
	    		break;
	    		
	    	case 8: //Right Slave Motor 3
	    		motorEncoder = robotDrive._frontRightMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	//motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering right slave 3. [Desc: " + robotDrive._rightSlave3.getDescription() + ", Name: " +robotDrive._rightSlave3.getName() + " ID: " +robotDrive._rightSlave3.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotDrive._rightSlave3.getBusVoltage());	
	    		}
	    		robotDrive._rightSlave3.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		//motorCurrentSum += pdp.getCurrent(Constants.pdpSlots[7]);
	    		break;
	    	case 9: //Wrist MoterobotIntake._wristMotor.getSelectedSensorPosition(0);
	    		if(motorEncoder > 2000) {
	    	    	motorDirection = -1;
	    	    	System.out.println("Reached 2500 ticks, switching direction");
	    		}else if(motorEncoder < 1000) {
	    	    	motorDirection = 1;
	    	    	System.out.println("Reached 100 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering wrist motor. [Desc: " + robotIntake._wristMotor.getDescription() + ", Name: " +robotIntake._wristMotor.getName() + " ID: " +robotIntake._wristMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotIntake._wristMotor.getBusVoltage());	
	    		}
	    		robotIntake._wristMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += robotIntake._wristMotor.getOutputCurrent();
	    		break;
	    	case 10: //Elevator Motor
	    		robotIntake.shiftElevator(false);
	    		motorEncoder = robotIntake.
	    				_elevatorMotor.getSelectedSensorPosition(0);
	    		if(motorEncoder > 150000) {
	    			robotIntake.shiftElevator(!elevTestShift);
	    	    	motorDirection = -1;
	    	    	System.out.println("Reached 150000 ticks, switching direction");
	    		}else if(motorEncoder < 25000) {
	    	    	motorDirection = 1;
	    	    	System.out.println("Reached 25000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering elevator motor. [Desc: " + robotIntake._elevatorMotor.getDescription() + ", Name: " +robotIntake._elevatorMotor.getName() + " ID: " +robotIntake._elevatorMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotIntake._elevatorMotor.getBusVoltage());	
	    		}
	    		robotIntake._elevatorMotor.set(motorDirection*.6); 
	    		motorCurrentCount++;
	    		motorCurrentSum += robotIntake._elevatorMotor.getOutputCurrent();
	    		break;
	    		
	    	case 11: //Elevator Slave 1
	    		robotIntake.shiftElevator(false);
	    		motorEncoder = robotIntake._elevatorMotor.getSelectedSensorPosition(0);
	    		if(motorEncoder > 150000) {
	    			robotIntake.shiftElevator(!elevTestShift);
	    	    	motorDirection = -1;
	    	    	System.out.println("Reached 150000 ticks, switching direction");
	    		}else if(motorEncoder < 25000) {
	    	    	motorDirection = 1;
	    	    	System.out.println("Reached 25000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering elevator slave 1. [Desc: " + robotIntake._elevatorSlave1.getDescription() + ", Name: " +robotIntake._elevatorSlave1.getName() + " ID: " +robotIntake._elevatorSlave1.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotIntake._elevatorSlave1.getBusVoltage());	
	    		}
	    		robotIntake._elevatorSlave1.set(motorDirection*.6); 
	    		motorCurrentCount++;
	    		motorCurrentSum += robotIntake._elevatorSlave1.getOutputCurrent();
	    		break;
	    		
	    	case 12: //Arm Motor
	    		motorEncoder = robotIntake.getArmPosition();
	    		if(motorEncoder > 3000) {
	    			//_armMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection = -1;
	    	    	System.out.println("Reached 5000 ticks, switching direction");
	    		}else if(motorEncoder <1000){
	    	    	motorDirection =1;
	    	    	System.out.println("Reached 1000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering arm motor. [Desc: " + robotIntake._armMotor.getDescription() + ", Name: " +robotIntake._armMotor.getName() + " ID: " +robotIntake._armMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotIntake._armMotor.getBusVoltage());	
	    		}
	    		robotIntake._armMotor.set(motorDirection*.4); 
	    		motorCurrentCount++;
	    		motorCurrentSum += robotIntake._armMotor.getOutputCurrent();
	    		break;
	    		
	    	case 13: //Intake Motor
	    		motorEncoder = robotIntake._intakeLeftMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotIntake._intakeLeftMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering intake motor. [Desc: " + robotIntake._intakeLeftMotor.getDescription() + ", Name: " +robotIntake._intakeLeftMotor.getName() + " ID: " +robotIntake._intakeLeftMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotIntake._intakeLeftMotor.getBusVoltage());	
	    		}
	    		robotIntake._intakeLeftMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += robotIntake._intakeLeftMotor.getOutputCurrent();
	    		break;
	    		
	    	case 14: //Intake Slave 1
	    		motorEncoder = robotIntake._intakeRightMotor.getSelectedSensorPosition(0);
	    		if(Math.abs(motorEncoder) > 40000) {
	    			robotIntake._intakeRightMotor.setSelectedSensorPosition(0, 0, 10);
	    	    	motorDirection *= -1;
	    	    	System.out.println("Reached 40000 ticks, switching direction");
	    		}
	    		if(motorCurrentCount==0) {
		    		System.out.println("Powering intake slave 1. [Desc: " + robotIntake._intakeRightMotor.getDescription() + ", Name: " +robotIntake._intakeRightMotor.getName() + " ID: " +robotIntake._intakeRightMotor.getDeviceID());	    		
		    		System.out.println("Voltage: " + robotIntake._intakeRightMotor.getBusVoltage());	
	    		}
	    		robotIntake._intakeRightMotor.set(motorDirection*.5); 
	    		motorCurrentCount++;
	    		motorCurrentSum += robotIntake._intakeRightMotor.getOutputCurrent();
	    		break;
	    	case 15:
	    		//open, wait 1s, close firm, wait 1s, close light, wait 1s, repeat
	    		if(testPeriodicCounter % 150 ==100) {
	    			robotIntake.gripLight();
	    		}else if(testPeriodicCounter % 150 ==50) {
	    			robotIntake.gripFirm();
	    		}else if(testPeriodicCounter % 150 ==0) {
	    			robotIntake.gripOpen();
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