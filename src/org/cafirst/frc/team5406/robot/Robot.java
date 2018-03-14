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

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.cafirst.frc.team5406.util.XboxController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 * 
 */
public class Robot extends IterativeRobot {

	int ARM_UP = 6850;
	int ARM_DOWN = 100;
	double ARM_RANGE = 135; //degrees
	double WRIST_RANGE = 135; //degrees
	int ELEVATOR_UP = 215000;
	int ELEVATOR_DOWN = 1000;
	int WRIST_UP = 0;
	int WRIST_DOWN = -2700;
	int ARM_DOWN_THRESHOLD = 500;
	int ELEVATOR_DOWN_THRESHOLD = 2500;
	int RAMP_SERVO_START = 75;
	int RAMP_SERVO_RELEASE = 180;
	
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

	Compressor compressor = new Compressor();
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
	
	   ArrayList<double[]> motionProfile = new ArrayList<double[]>();
		ArrayList<Double> pathLeft = new ArrayList<Double>();
		ArrayList<Double> pathRight = new ArrayList<Double>();
		ArrayList<Double> pathAngles = new ArrayList<Double>();
		double totalDistanceLeft = 0;
		double totalDistanceRight = 0;
		double revPerInch = 0.075;
		double maxSpeed = 5 * 12 * revPerInch * 60;
		double minSpeed = 1 * 12 * revPerInch * 60;
		double driveGearRatio = 4.125; //66/16
		

    Solenoid elevatorSolenoid;
    Solenoid gripSolenoidLow;
    Solenoid gripSolenoidHigh;
    Solenoid rampSolenoid;
    int flipWrist = 0;
    int gripCounter = 0;
	int autoLoop = 0;
	int step =0;
	int stepA = 0;
	boolean holdPosition = false;
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
	double WHEEL_BASE = 35.5;
	
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
    double startTime = 0;
    int lastPoint = 0;
    double targetTime = 0;
    boolean drivePathDone = false;
    int autoStep = 0;
	double GYRO_PID_P = 0.022;
	double GYRO_PID_I = 0.000;
	double GYRO_PID_D = 0.125;
	int wristUpDelay = 0;
    
    class AutoRunnable implements java.lang.Runnable {
    	private double targetAngle;
    	private double accumI = 0.0;
    	public double lastAngle = 0;
    	private double previousError = 0.0;
    	
	    public void run() {
	    	
	    	double leftSpeed = 0;
			double rightSpeed = 0;
			double dSpeed = 0;
			double speedChangeMultiplier = 0;
			double targetSpeedLeft = 0;
			double targetSpeedRight = 0;
			double currentAngle = navX.getYaw();
 	
	    	
	    	if(!drivePathDone) {
	    		System.out.println("Cur:" +  currentAngle + "Tar:"+ motionProfile.get(lastPoint)[5]);
				speedChangeMultiplier = calcSpeed(motionProfile.get(lastPoint)[5] - currentAngle);
	    		double elapsedTime = Timer.getFPGATimestamp() - startTime; 	
		    	int numPoints = motionProfile.size();
		    	if (elapsedTime > targetTime) {
		    		if(lastPoint < numPoints -2) {
		    			//System.out.println(lastPoint + ", " + motionProfile.get(lastPoint)[2]/1000 + ", " + motionProfile.get(lastPoint)[1]/1000);
			    		targetTime += motionProfile.get(lastPoint)[0]/1000;
			    		targetSpeedLeft = motionProfile.get(lastPoint)[2]*(4096/600);
						leftSpeed =targetSpeedLeft+Math.signum(targetSpeedLeft)*targetSpeedLeft*speedChangeMultiplier; //-1*400-1200 = -1800
			    		targetSpeedRight = motionProfile.get(lastPoint)[4]*(4096/600);
						rightSpeed = targetSpeedRight-Math.signum(targetSpeedRight)*targetSpeedRight*speedChangeMultiplier; //400-1200 = -800
						System.out.println("LS: "+ leftSpeed + ", LT: " + targetSpeedLeft + ", LA:" + _frontLeftMotor.getSelectedSensorVelocity(0) + ", RS: "+ rightSpeed + ", RT: " + targetSpeedRight + ", RA:" + _frontRightMotor.getSelectedSensorVelocity(0));
			    		_frontRightMotor.set(ControlMode.Velocity,rightSpeed);
						_frontLeftMotor.set(ControlMode.Velocity, leftSpeed);
			    		lastPoint++;
		    		}else {
		    			_frontLeftMotor.set(ControlMode.Velocity,0);
		    			_frontRightMotor.set(ControlMode.Velocity,0);
		    			drivePathDone = true;
		    		}
		    	}
	    	}

	    }
	    
	    public double calcSpeed(double currentError){
			
	 		double valP = GYRO_PID_P * currentError;
	 		double valI = accumI;
	 		double valD = GYRO_PID_D * (previousError - currentError);
	 		if(Math.abs(valD) > Math.abs(valP)) valD = valP; // Limit so that D isn't the driving number
	 		accumI += GYRO_PID_I;
	 		
	 		//If we overshoot, reset the I
	 		if(Math.signum(previousError) != Math.signum(currentError)){ 
	 			accumI = 0; 
	 			valI = 0;
	 		}
	 		double speed = valP + (valI * (currentError > 0 ? 1.0 : -1.0)) - valD;
	 		previousError = currentError;
	 		return speed;
	 	}
	}
	Notifier _autoLoop = new Notifier(new AutoRunnable());
	
    class PeriodicRunnable implements java.lang.Runnable {
	    public void run() { 

	    	int armPos;
	    	
	    	switch (flipWrist) {
	    	case 1:
	    		_wristMotor.selectProfileSlot(0,0);
	    		armPos = _armMotor.getSelectedSensorPosition(0);
	    		_wristMotor.set(ControlMode.MotionMagic, wristFlipPos(armPos));
	    		if(armPos < 200) {
	    			flipWrist = 5;
	    		}
	    		break;
	    	case 3:
	    		_wristMotor.selectProfileSlot(0,0);
	    		armPos = _armMotor.getSelectedSensorPosition(0);
	    		_wristMotor.set(ControlMode.MotionMagic, wristFlipPos(armPos));
	    		if(armPos > ARM_UP - 200) {
	    			flipWrist = 5;
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
	    	
	    	_frontLeftMotor.configOpenloopRamp(0.25, kTimeoutMs);
	    	_frontRightMotor.configOpenloopRamp(0.25, kTimeoutMs);
	    	_frontLeftMotor.configClosedloopRamp(0.08, kTimeoutMs);
	    	_frontRightMotor.configClosedloopRamp(0.08, kTimeoutMs);
	    	_frontLeftMotor.selectProfileSlot(0,0);
	    	_frontLeftMotor.config_kF(0, .8, kTimeoutMs);
	    	_frontLeftMotor.config_kP(0, 0.2, kTimeoutMs);
	    	_frontLeftMotor.config_kI(0, 0.0001, kTimeoutMs);
	    	_frontLeftMotor.config_kD(0, 0.001, kTimeoutMs);
	    	_frontRightMotor.selectProfileSlot(0,0);
	    	_frontRightMotor.config_kF(0, .8, kTimeoutMs);
	    	_frontRightMotor.config_kP(0, 0.2, kTimeoutMs);
	    	_frontRightMotor.config_kI(0, 0.0001, kTimeoutMs);
	    	_frontRightMotor.config_kD(0, 0.001, kTimeoutMs);
	    	_frontRightMotor.setSensorPhase(false);
	    	_frontRightMotor.setInverted(true);
	    	_rightSlave1.setInverted(true);
	    	_rightSlave2.setInverted(true);
	    	_rightSlave3.setInverted(true);
	    	
	    	
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
	    	_frontRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, kTimeoutMs);
			_frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
			_frontLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, kTimeoutMs);
			_frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
			_frontLeftMotor.configContinuousCurrentLimit(20, kTimeoutMs);
			//_frontLeftMotor.configPeakCurrentLimit(30, 0);
			//_frontLeftMotor.configPeakCurrentDuration(30, 0);
			_frontLeftMotor.enableCurrentLimit(true);
			_frontRightMotor.configContinuousCurrentLimit(20, kTimeoutMs);
			//_frontRightMotor.configPeakCurrentLimit(30, 0);
			//_frontRightMotor.configPeakCurrentDuration(30, 0);
			_frontRightMotor.enableCurrentLimit(true);
			_armMotor.configContinuousCurrentLimit(40, kTimeoutMs);
			_armMotor.configPeakCurrentLimit(60, kTimeoutMs);
			_armMotor.configPeakCurrentDuration(30, kTimeoutMs);
			_armMotor.enableCurrentLimit(true);
			_intakeMotor.configContinuousCurrentLimit(40, kTimeoutMs);
			_intakeMotor.configPeakCurrentLimit(40, kTimeoutMs);
			_intakeMotor.configPeakCurrentDuration(40, kTimeoutMs);
			_intakeMotor.enableCurrentLimit(true);
			_wristMotor.configContinuousCurrentLimit(30, kTimeoutMs);
			_wristMotor.configPeakCurrentLimit(30, kTimeoutMs);
			_wristMotor.configPeakCurrentDuration(30, kTimeoutMs);
			_wristMotor.enableCurrentLimit(true);
	
	    	_elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, kTimeoutMs);
			_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
			_armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, kTimeoutMs);
			_armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
			_wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, kTimeoutMs);
			_wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	
	    	_wristMotor.selectProfileSlot(0,0);
	    	_wristMotor.config_kF(0, 1, kTimeoutMs);
	    	_wristMotor.config_kP(0, 0.5, kTimeoutMs);
	    	_wristMotor.config_kI(0, 0, kTimeoutMs);
	    	_wristMotor.config_kD(0, 0, kTimeoutMs);
	    	_wristMotor.selectProfileSlot(1,0);
	    	_wristMotor.config_kF(1, 1, kTimeoutMs);
	    	_wristMotor.config_kP(1, 1, kTimeoutMs);
	    	_wristMotor.config_kI(1, 0.001, kTimeoutMs);
	    	_wristMotor.config_kD(1, 0, kTimeoutMs);
			/* set acceleration and vcruise velocity - see documentation */
	    	_wristMotor.configMotionCruiseVelocity(4000, kTimeoutMs);
	    	_wristMotor.configMotionAcceleration(8000, kTimeoutMs);
			
	
	    	
	    	_armMotor.selectProfileSlot(0,0);
	    	_armMotor.config_kF(0, 0.4, kTimeoutMs);
	    	_armMotor.config_kP(0, 0.4, kTimeoutMs);
	    	_armMotor.config_kI(0, 0, kTimeoutMs);
	    	_armMotor.config_kD(0, 0, kTimeoutMs);
	    	_armMotor.selectProfileSlot(1,0);
	    	_armMotor.config_kF(1, 0.1, kTimeoutMs);
	    	_armMotor.config_kP(1, 0.15, kTimeoutMs);
	    	_armMotor.config_kI(1, 0, kTimeoutMs);
	    	_armMotor.config_kD(1, 0, kTimeoutMs);
			/* set acceleration and vcruise velocity - see documentation */
	    	_armMotor.configMotionCruiseVelocity(2000, kTimeoutMs);
	    	_armMotor.configMotionAcceleration(4000, kTimeoutMs);
			
	
	    	
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
			
	    	
	    	
	    	
	    	_elevatorMotor.configForwardSoftLimitThreshold(216000, kTimeoutMs);
	    	_elevatorMotor.configReverseSoftLimitThreshold(0, kTimeoutMs);
	    	_wristMotor.configForwardSoftLimitThreshold(0, kTimeoutMs);
	    	_wristMotor.configReverseSoftLimitThreshold(-2700, kTimeoutMs);
	    	_armMotor.configForwardSoftLimitThreshold(6900, kTimeoutMs);
	    	_armMotor.configReverseSoftLimitThreshold(0, kTimeoutMs);
	    	_armMotor.configForwardSoftLimitEnable(true, kTimeoutMs);
	    	_armMotor.configReverseSoftLimitEnable(true, kTimeoutMs);
	    	_wristMotor.configForwardSoftLimitEnable(true, kTimeoutMs);
	    	_wristMotor.configReverseSoftLimitEnable(true, kTimeoutMs);
	    	_elevatorMotor.configForwardSoftLimitEnable(true, kTimeoutMs);
	    	_elevatorMotor.configReverseSoftLimitEnable(true, kTimeoutMs);
    	

		}
	}
	
	public void zeroMotors() {
		/* zero the sensor */
    	_elevatorMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
    	_elevatorMotor.set(ControlMode.PercentOutput, 0);
    	/* zero the sensor */
    	_armMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
    	_armMotor.set(ControlMode.PercentOutput, 0);
    	/* zero the sensor */
    	_wristMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
    	_wristMotor.set(ControlMode.PercentOutput, 0);
	}
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	gripSolenoidHigh = new Solenoid(3);
    	elevatorSolenoid = new Solenoid(1);
    	rampSolenoid = new Solenoid(2);
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
    	zeroMotors();
    }
    public void disabledInit() {
    	disabled = true;
    	rampHold();
    	_elevatorMotor.set(ControlMode.PercentOutput, 0);
    	_wristMotor.set(ControlMode.PercentOutput, 0);
    	_armMotor.set(ControlMode.PercentOutput, 0);
    	_autoLoop.stop();
    }
    
    public void teleopInit() {
    	_autoLoop.stop();
    	setupMotors();
		_wristMotor.set(ControlMode.MotionMagic, _wristMotor.getSelectedSensorPosition(0));
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
		
		long elevatorPos = _elevatorMotor.getSelectedSensorPosition(0);
		long armPos = _armMotor.getSelectedSensorPosition(0);
		//long wristPos = _wristMotor.getSelectedSensorPosition(0);
		
		
		



    	
		
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
		
		if(driverGamepad.getButtonHeld(XboxController.START_BUTTON) && driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)) {
			rampRelease();
		}
		

			_drive.arcadeDrive(precisionDriveX*driverGamepad.getLeftX(), precisionDriveY*driverGamepad.getLeftY());
        //displayCurrent();
		
		if(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
			compressor.stop();
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
		} else {
			compressor.start();
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
			//wristDown();
			flipWrist =1;
			_notifier.startPeriodic(0.005);
			wristSet = true;
			//manualWrist = true;
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
        	if(armPos < ARM_DOWN_THRESHOLD) {
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
			if(elevatorPos < ELEVATOR_DOWN_THRESHOLD && armPos < ARM_DOWN_THRESHOLD) {
				gripState = GripState.LIGHT;
			}
			wristDown();
			needsWristUp = true;
			wristSet = true;
		}
		
		if (operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)) {
			if(armPos < ARM_DOWN_THRESHOLD) {
				wristSlightUp();
				wristSet = true;
				needsWristUp = true;
			}
		}
		
		if(operatorGamepad.getButtonHeld(XboxController.BACK_BUTTON)){
			gripState = GripState.OPEN;
		}
		
		if(!wristSet && needsWristUp) {
			flipWrist = 5;
			if(armPos < ARM_DOWN_THRESHOLD) {
				needsWristUp = false;
				wristUp();
			}
		}
		
		
		switch(operatorGamepad.getDirectionPad()){
		case UP:
			_armMotor.set(0.6);
			manualArm = true;
			if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
				armOverrideNew = true;
			}
			break;
		case DOWN:
			_armMotor.set(-0.4);
			manualArm = true;
			if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
				armOverrideNew = true;
			}
			break;
		case LEFT:
			if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON)) {
				_armMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
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
			_armMotor.configForwardSoftLimitEnable(!armOverrideNew, kTimeoutMs);
			_armMotor.configReverseSoftLimitEnable(!armOverrideNew, kTimeoutMs);
			armOverride = armOverrideNew;
		}
		
		if (elevatorOverrideNew != elevatorOverride) {
			_elevatorMotor.configForwardSoftLimitEnable(!elevatorOverrideNew, kTimeoutMs);
			_elevatorMotor.configReverseSoftLimitEnable(!elevatorOverrideNew, kTimeoutMs);
			elevatorOverride = elevatorOverrideNew;
		}
		
		if (wristOverrideNew != wristOverride) {
			_wristMotor.configForwardSoftLimitEnable(!wristOverrideNew, kTimeoutMs);
			_wristMotor.configReverseSoftLimitEnable(!wristOverrideNew, kTimeoutMs);
			wristOverride = wristOverrideNew;
		}
		
    }
    
   
    public void rampRelease() {
    	rampSolenoid.set(true);
    }
    
    public void rampHold() {
    	rampSolenoid.set(false);
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
    	_armMotor.set(ControlMode.MotionMagic, ARM_UP);
    	armUp = true;
    }
    public void armDown() {
    	_armMotor.selectProfileSlot(1,0);
    	_armMotor.set(ControlMode.MotionMagic, ARM_DOWN);
    	armUp = false;
    }
    
    public double wristFlipPos(int armPos) {
    	return 1E-05*armPos*armPos - 0.4935*armPos - 5E-11;
    	//return -0.0002*armPos*armPos + 1.0437*armPos - 2800;


    }
    
    public void wristSlightUp() {
    	_wristMotor.selectProfileSlot(0,0);
    	_wristMotor.set(ControlMode.MotionMagic, -2100);
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
    	_wristMotor.set(ControlMode.MotionMagic, WRIST_UP);
    	wristUp = true;
    	manualWrist = false;
    }
    public void wristDown() {
    	_wristMotor.selectProfileSlot(1,0);
    	_wristMotor.set(ControlMode.MotionMagic, WRIST_DOWN);
    	wristUp = false;
    	manualWrist = false;
    }

    public void elevatorUp() {
    	_elevatorMotor.selectProfileSlot(0,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, ELEVATOR_UP);
    	elevatorUp = true;
    }
    public void elevatorSwitchMid() {
    	_elevatorMotor.selectProfileSlot(0,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, 150000);
    	elevatorUp = true;
    }

    public void elevatorDown() {
    	_elevatorMotor.selectProfileSlot(1,0);
    	_elevatorMotor.set(ControlMode.MotionMagic, ELEVATOR_DOWN);
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
    public void autonomousInit() {
    	setupMotors();
		_wristMotor.set(ControlMode.MotionMagic, _wristMotor.getSelectedSensorPosition(0));

    	_frontLeftMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
    	_frontRightMotor.setSelectedSensorPosition(0, 0, kTimeoutMs);
    	startTime = Timer.getFPGATimestamp();
         lastPoint = 0;
        targetTime = 0;
		pathLeft = new ArrayList<Double>();
		pathRight = new ArrayList<Double>();
		pathAngles = new ArrayList<Double>();
         motionProfile = new ArrayList<double[]>();
 		drivePathDone = false;
         autoStep = 0;
         navX.zeroYaw();
         wristUpDelay = 0;
    }
    
   public void autonomousPeriodic() {
	   	switch(autoStep) {
	   	case 0:

			if(!wristZeroed) {
				_wristMotor.configForwardSoftLimitEnable(false, kTimeoutMs);
				_wristMotor.configReverseSoftLimitEnable(false, kTimeoutMs);
				_wristMotor.set(0.5);
				System.out.print(_wristMotor.getOutputCurrent());
				if(_wristMotor.getOutputCurrent()>15) {
					_wristMotor.setSelectedSensorPosition(150, 0, kTimeoutMs);
					_wristMotor.configForwardSoftLimitEnable(false, kTimeoutMs);
					_wristMotor.configReverseSoftLimitEnable(false, kTimeoutMs);
					wristUp();
					wristZeroed = true;
				}
			}
			
			if(!armZeroed) {
				_armMotor.set(-0.5);
				_armMotor.configForwardSoftLimitEnable(false, kTimeoutMs);
				_armMotor.configReverseSoftLimitEnable(false, kTimeoutMs);

				if(_armMotor.getOutputCurrent()>5) {
					_armMotor.setSelectedSensorPosition(-10, 0, kTimeoutMs);
					_armMotor.configForwardSoftLimitEnable(false, kTimeoutMs);
					_armMotor.configReverseSoftLimitEnable(false, kTimeoutMs);
					armDown();
					armZeroed = true;
				}
			}
			
			if(!elevatorZeroed) {
				_elevatorMotor.set(-0.5);
				_elevatorMotor.configForwardSoftLimitEnable(false, kTimeoutMs);
				_elevatorMotor.configReverseSoftLimitEnable(false, kTimeoutMs);
				if(_elevatorMotor.getOutputCurrent()>5) {
					_elevatorMotor.setSelectedSensorPosition(-500, 0, kTimeoutMs);
					_elevatorMotor.configForwardSoftLimitEnable(true, kTimeoutMs);
					_elevatorMotor.configReverseSoftLimitEnable(true, kTimeoutMs);
					elevatorDown();
					elevatorZeroed = true;
				}
			}
			
			autoStep++;
			break;
	   	case 1:
	   	   String gameData;
	   	   gameData = DriverStation.getInstance().getGameSpecificMessage();
           if(gameData.length() > 0)
           {
        	   ArrayList<Point2D> left = new ArrayList<Point2D>();
        	   left.add(new Point2D.Double(0, 0));
				
			  if(gameData.charAt(0) == 'L')
			  {
				  left.add(new Point2D.Double(-60, 107));
				  bezierPoints(left, 0, 5);
			  } else {
				  left.add(new Point2D.Double(40, 107));
				  bezierPoints(left, 0, 355);
			  }
				
			  _autoLoop.startPeriodic(0.005);
			  autoStep++;
           }
           break;
	   	case 2:
	   		if(drivePathDone) {
	   			autoStep++;
	   		}
	   		break;
	   	case 3:
   	   		wristSlightDown();
   	   		needsWristUp = true;
   	   		if(_wristMotor.getSelectedSensorPosition(0) < -350) {
   	   			_intakeMotor.set(0.7);
   	   		}
	   		break;
	   	case 4:
	   		wristUpDelay++;
	   		if(wristUpDelay > 200) {
	   			_intakeMotor.set(0);
	   			wristUp();
	   		}
	   	}
               
   		
    }
   
   public Point2D[] findControlPoints(Point2D s1, Point2D s2, Point2D s3) {
		Point2D[] controlPoints = new Point2D[2];
		double l1 = s1.distance(s2);
		double l2 = s2.distance(s3);
		Point2D m1 = new Point2D.Double((s1.getX() + s2.getX()) / 2.0, (s1.getY() + s2.getY()) / 2.0);
		Point2D m2 = new Point2D.Double((s2.getX() + s3.getX()) / 2.0, (s2.getY() + s3.getY()) / 2.0);
		double dxm = m1.getX() - m2.getX();
		double dym = m1.getY() - m2.getY();
		double k = l2 / (l1 + l2);
		Point2D cm = new Point2D.Double(m2.getX() + dxm * k, m2.getY() + dym * k);
		double tx = s2.getX() - cm.getX();
		double ty = s2.getY() - cm.getY();
		controlPoints[0] = new Point2D.Double(m1.getX() + tx, m1.getY() + ty);
		controlPoints[1] = new Point2D.Double(m2.getX() + tx, m2.getY() + ty);
		return controlPoints;
	}

	public ArrayList<Point2D> plotBezierQuad(Point2D s1, Point2D s2, Point2D s3, Point2D s4) {

		ArrayList<Point2D> leftWheel = new ArrayList<Point2D>();
		ArrayList<Point2D> rightWheel = new ArrayList<Point2D>();

		Point2D[] S1 = findControlPoints(s1, s2, s3);
		Point2D[] S2 = findControlPoints(s2, s3, s4);
		double l1 = Math.floor(s1.distance(s2));
		double l2 = Math.floor(s2.distance(s3));
		double l3 = Math.floor(s3.distance(s4));

		double step = 1;

		int ctr = (int) Math.floor(((l1 > 0 ? l1 : l2) + (l3 > 0 ? l3 : l2)) / step); // scale by step to change number
																						// of points interpolated
		Point2D p1 = (Point2D) s2.clone();
		Point2D p2 = S1[1];
		Point2D p3 = S2[0];
		Point2D p4 = (Point2D) s3.clone();

		// Now do actual bezier math
		Point2D pf = p1;
		double ss = 1.0 / (ctr + 1), ss2 = ss * ss, ss3 = ss2 * ss, pre1 = 3.0 * ss, pre2 = 3.0 * ss2, pre4 = 6.0 * ss2,
				pre5 = 6.0 * ss3, tmp1x = p1.getX() - p2.getX() * 2.0 + p3.getX(),
				tmp1y = p1.getY() - p2.getY() * 2.0 + p3.getY(),
				tmp2x = (p2.getX() - p3.getX()) * 3.0 - p1.getX() + p4.getX(),
				tmp2y = (p2.getY() - p3.getY()) * 3.0 - p1.getY() + p4.getY(),
				dfx = (p2.getX() - p1.getX()) * pre1 + tmp1x * pre2 + tmp2x * ss3,
				dfy = (p2.getY() - p1.getY()) * pre1 + tmp1y * pre2 + tmp2y * ss3, ddfx = tmp1x * pre4 + tmp2x * pre5,
				ddfy = tmp1y * pre4 + tmp2y * pre5, dddfx = tmp2x * pre5, dddfy = tmp2y * pre5;
		// System.out.println(ctr);
		double m_right;
		double dy_right;
		double dx_right;
		double distanceLeft = 0;
		double distanceRight = 0;
		double angle = 0;
		boolean firstRight = true;
		Point2D pt2 = (Point2D) pf.clone();
		Point2D last_pt = (Point2D) pt2.clone();
		while (ctr > 0) {
			ctr--;
			pf.setLocation(pf.getX() + dfx, pf.getY() + dfy);
			m_right = -1 * dfx / dfy;
			dx_right = WHEEL_BASE / Math.sqrt(m_right * m_right + 1);
			dy_right = m_right * dx_right;
			distanceLeft = Math.sqrt(dfx * dfx + dfy * dfy) * revPerInch;

			if (dfy < 0) {
				pt2.setLocation(pf.getX() - dx_right, pf.getY() - dy_right);
			} else {
				pt2.setLocation(pf.getX() + dx_right, pf.getY() + dy_right);
			}
			if(firstRight) {
				firstRight = false;
				last_pt.setLocation(pt2.getX(), pt2.getY());
			}
			//System.out.println("Rpt: " + pt2.getX() + ", " + pt2.getY() + ", " + last_pt.getX() + ", " + last_pt.getY());
			distanceRight = pt2.distance(last_pt)*revPerInch;
			last_pt.setLocation(pt2.getX(), pt2.getY());
			
			totalDistanceLeft += distanceLeft;
			totalDistanceRight += distanceRight;
			pathLeft.add(distanceLeft);
			pathRight.add(distanceRight);
			rightWheel.add(pt2);
			dfx += ddfx;
			dfy += ddfy;
			ddfx += dddfx;
			ddfy += dddfy;
			pathAngles.add(Math.toDegrees(Math.atan2(dfx, dfy)));
			leftWheel.add(pf);

			//System.out.println(pf.getX() + ", " + pf.getY() + ", " + pt2.getX() + ", " + pt2.getY());
		}
		pf.setLocation(p4.getX(), p4.getY());
		leftWheel.add(pf);
		m_right = -1 * dfx / dfy;
		dx_right = WHEEL_BASE / Math.sqrt(m_right * m_right + 1);
		dy_right = m_right * dx_right;
		if (dfy < 0) {
			pt2.setLocation(pf.getX() - dx_right, pf.getY() - dy_right);
		} else {
			pt2.setLocation(pf.getX() + dx_right, pf.getY() + dy_right);
		}
		rightWheel.add(pt2);
		//System.out.println(pf.getX() + ", " + pf.getY() + ", " + pt2.getX() + ", " + pt2.getY());

		return leftWheel;
	}


	public void bezierPoints(ArrayList<Point2D> pts, double angleIn, double angleOut) {

		/*
		 * duplicate first and last points loop over sets of 4 points 0...3; 1...4, etc.
		 */
		// System.out.println("BezierPoints Function");
		if (pts.size() >= 2) {
			Point2D startControl = new Point2D.Double(pts.get(1).getX() + (2*(pts.get(0).getY() - pts.get(1).getY())*(Math.sin(angleIn*Math.PI/180))),
					pts.get(1).getY() + (2*(pts.get(0).getY() - pts.get(1).getY())*(Math.cos(angleIn*Math.PI/180))));
			Point2D endControl  = new Point2D.Double(pts.get(pts.size()-2).getX() + (2*(pts.get(pts.size()-1).getY() - pts.get(pts.size()-2).getY())*(Math.sin(angleOut*Math.PI/180))),
					pts.get(pts.size()-2).getY() + (2*(pts.get(pts.size()-1).getY() - pts.get(pts.size()-2).getY())*(Math.cos(angleOut*Math.PI/180))));


			
			pts.add(0, startControl);
			pts.add(endControl);
			int pt_i = 0;
			do {
				plotBezierQuad(pts.get(pt_i), pts.get(pt_i + 1), pts.get(pt_i + 2), pts.get(pt_i + 3));
				pt_i++;
			} while ((pt_i + 3) < pts.size());
			
			double distanceLeft = 0;
			double distanceRight = 0;
			double accDistance = 0;
			double decDistance = 0;
			double targetSpeed = maxSpeed;
			double timeLeft = 0;
			double maxDistance =0;
			double totalDistance =0;
			double speedLeft=0, speedRight = 0;
			double cDistanceLeft=0, cDistanceRight = 0;
			double timeRight = 0;
			double accRevs=2;
			double angle = 0;
			double[] motionProfileData = new double[6];

			totalDistance = Math.max(totalDistanceLeft, totalDistanceRight);

			for (int i = 0; i < pathLeft.size(); i++) {

				distanceLeft = pathLeft.get(i);
				distanceRight = pathRight.get(i);
				cDistanceLeft+=distanceLeft;
				cDistanceRight+=distanceRight;

				if(totalDistance < 2*accRevs) {
					accDistance = totalDistance/2;
					decDistance = accDistance;
				}else {
					accDistance = accRevs;
					decDistance = totalDistance - accRevs;
				}
				
				maxDistance = Math.max(cDistanceLeft, cDistanceRight);
				if(maxDistance < accDistance) {
					targetSpeed = minSpeed+(maxSpeed-minSpeed)*maxDistance/accRevs;
				}else if (maxDistance>=accDistance && maxDistance<=decDistance) {
					targetSpeed=maxSpeed;
				}else {
					targetSpeed = minSpeed+(maxSpeed-minSpeed)*(totalDistance-maxDistance)/accRevs;
				}
				//targetSpeed = maxSpeed;
				
				if(distanceLeft > distanceRight) {
					speedLeft = targetSpeed;
					timeLeft = 60000 * distanceLeft / speedLeft;
					timeRight = timeLeft;
					speedRight = 60000*distanceRight/timeRight;
				}else {
					speedRight = targetSpeed;
					timeRight = 60000 * distanceRight / speedRight;
					timeLeft = timeRight;
					speedLeft = 60000*distanceLeft/timeLeft;
				}
				
				motionProfileData = new double[6];
				motionProfileData[0] = timeLeft;
				motionProfileData[1] = cDistanceLeft;
				motionProfileData[2] = speedLeft;
				motionProfileData[3] = cDistanceRight;
				motionProfileData[4] = speedRight;
				motionProfileData[5] = pathAngles.get(i) - angleIn;

				motionProfile.add(motionProfileData);

			}
			

			
			/*System.out.println("Left motion profile");
			for (int i = 0; i < motionProfileLeft.size(); i++) {
				System.out.println(motionProfileLeft.get(i)[0] + ", " + motionProfileLeft.get(i)[1] + ", "
						+ motionProfileLeft.get(i)[2]);
			}
			System.out.println("Right motion profile");
			for (int i = 0; i < motionProfileRight.size(); i++) {
				System.out.println(motionProfileRight.get(i)[0] + ", " + motionProfileRight.get(i)[1] + ", "
						+ motionProfileRight.get(i)[2]);
			}*/
			//System.out.println(motionProfileLeft.get(motionProfileLeft.size()-1)[0]);
			//System.out.println(motionProfileRight.get(motionProfileRight.size()-1)[0]);


		}
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
	    	    	//motorDirection *= -1;
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
	    	    	//motorDirection *= -1;
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
	    	    	//motorDirection *= -1;
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
	    	    	//motorDirection *= -1;
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
	    	    	//motorDirection *= -1;
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
	    	    	//motorDirection *= -1;
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
	    	    	//motorDirection *= -1;
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
	    	    	//motorDirection *= -1;
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