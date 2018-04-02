package org.cafirst.frc.team5406.subsystems;


import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.util.PolynomialRegression;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Intake extends Subsystems{
    Solenoid elevatorSolenoid;
    Solenoid gripSolenoidLow;
    Solenoid gripSolenoidHigh;
    Solenoid rampSolenoid;
    	
	public WPI_TalonSRX _intakeRightMotor= new WPI_TalonSRX(10);
	
	/*Comp Bot has a Victor for ID #9 - Intake, Practice Bot has a Talon*/
	//public WPI_VictorSPX _intakeSlave1 = new WPI_VictorSPX(9);
	public WPI_TalonSRX _intakeLeftMotor = new WPI_TalonSRX(9);

	public Compressor compressor = new Compressor();
	public WPI_TalonSRX _elevatorMotor= new WPI_TalonSRX(12); //
	public WPI_VictorSPX _elevatorSlave1 = new WPI_VictorSPX(11);
	public WPI_TalonSRX _armMotor = new WPI_TalonSRX(13); //arm motor
	public WPI_TalonSRX _wristMotor= new WPI_TalonSRX(14);
	
	public boolean armUp = false;
	public boolean elevatorUp = false;
	public boolean wristUp = false;
    public boolean gripSpin = false;

    public boolean manualElevator = false;
    public boolean manualWrist = false;
    public boolean manualArm = false;
    public boolean disabled = false;
    
	public boolean wristZeroed = false;
	public boolean armZeroed = false;
	public boolean elevatorZeroed = false;
	public boolean needsWristUp = false;
	public boolean wristOut = false;
    PolynomialRegression profileUpIn;
    PolynomialRegression profileUpOut;
    PolynomialRegression profileDownIn;
    PolynomialRegression profileDownOut;

	public Intake(){
    	gripSolenoidHigh = new Solenoid(Constants.GRIP_HIGH);
    	elevatorSolenoid = new Solenoid(Constants.ELEVATOR_SHIFT);
    	rampSolenoid = new Solenoid(Constants.RAMP_RELEASE);
    	gripSolenoidLow = new Solenoid(Constants.GRIP_LOW);
    	
        profileUpIn = new PolynomialRegression(Constants.PROFILE_UP_IN_ARM, Constants.PROFILE_UP_IN_WRIST, 3);
        profileUpOut = new PolynomialRegression(Constants.PROFILE_UP_OUT_ARM, Constants.PROFILE_UP_OUT_WRIST, 3);
        profileDownIn = new PolynomialRegression(Constants.PROFILE_DOWN_IN_ARM, Constants.PROFILE_DOWN_IN_WRIST, 3);
        profileDownOut = new PolynomialRegression(Constants.PROFILE_DOWN_OUT_ARM, Constants.PROFILE_DOWN_OUT_WRIST, 3);

	
	}
	public void setupMotors() {
		if(disabled) {
			disabled = false;	
	    	
			_elevatorSlave1.follow(_elevatorMotor);
	    	//_intakeSlave1.follow(_intakeMotor);
	    	_intakeLeftMotor.setInverted(true);
	    	
	    	_armMotor.setSensorPhase(false);
	    	_wristMotor.setSensorPhase(false);
	    	_intakeRightMotor.setSensorPhase(false);
	    	_intakeLeftMotor.setSensorPhase(false);
			_armMotor.configContinuousCurrentLimit(40, Constants.kTimeoutMs);
			_armMotor.configPeakCurrentLimit(60, Constants.kTimeoutMs);
			_armMotor.configPeakCurrentDuration(30, Constants.kTimeoutMs);
			_armMotor.enableCurrentLimit(true);
			_intakeRightMotor.configContinuousCurrentLimit(40, Constants.kTimeoutMs);
			_intakeRightMotor.configPeakCurrentLimit(40, Constants.kTimeoutMs);
			_intakeRightMotor.configPeakCurrentDuration(40, Constants.kTimeoutMs);
			_intakeRightMotor.enableCurrentLimit(true);
			_intakeLeftMotor.configContinuousCurrentLimit(40, Constants.kTimeoutMs);
			_intakeLeftMotor.configPeakCurrentLimit(40, Constants.kTimeoutMs);
			_intakeLeftMotor.configPeakCurrentDuration(40, Constants.kTimeoutMs);
			_intakeLeftMotor.enableCurrentLimit(true);
			_wristMotor.configContinuousCurrentLimit(30, Constants.kTimeoutMs);
			_wristMotor.configPeakCurrentLimit(30, Constants.kTimeoutMs);
			_wristMotor.configPeakCurrentDuration(30, Constants.kTimeoutMs);
			_wristMotor.enableCurrentLimit(true);
	
	    	_elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
			_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
			_armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
			_armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
			_wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
			_wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
			_intakeRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
			_intakeRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
			_intakeLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
			_intakeLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
	
	    	_wristMotor.selectProfileSlot(0,0);
	    	_wristMotor.config_kF(0, 1, Constants.kTimeoutMs);
	    	_wristMotor.config_kP(0, 0.5, Constants.kTimeoutMs);
	    	_wristMotor.config_kI(0, 0, Constants.kTimeoutMs);
	    	_wristMotor.config_kD(0, 0, Constants.kTimeoutMs);
	    	_wristMotor.selectProfileSlot(1,0);
	    	_wristMotor.config_kF(1, 1, Constants.kTimeoutMs);
	    	_wristMotor.config_kP(1, 1.2, Constants.kTimeoutMs);
	    	_wristMotor.config_kI(1, 0, Constants.kTimeoutMs);
	    	_wristMotor.config_kD(1, 0, Constants.kTimeoutMs);
			/* set acceleration and vcruise velocity - see documentation */
	    	_wristMotor.configMotionCruiseVelocity(4000, Constants.kTimeoutMs);
	    	_wristMotor.configMotionAcceleration(8000, Constants.kTimeoutMs);
			
	
	    	
	    	_armMotor.selectProfileSlot(0,0);
	    	_armMotor.config_kF(0, 0.4, Constants.kTimeoutMs);
	    	_armMotor.config_kP(0, 0.4, Constants.kTimeoutMs);
	    	_armMotor.config_kI(0, 0, Constants.kTimeoutMs);
	    	_armMotor.config_kD(0, 0, Constants.kTimeoutMs);
	    	_armMotor.selectProfileSlot(1,0);
	    	_armMotor.config_kF(1, 0.4, Constants.kTimeoutMs);
	    	_armMotor.config_kP(1, 0.4, Constants.kTimeoutMs);
	    	_armMotor.config_kI(1, 0, Constants.kTimeoutMs);
	    	_armMotor.config_kD(1, 0, Constants.kTimeoutMs);
			/* set acceleration and vcruise velocity - see documentation */
	    	_armMotor.configMotionCruiseVelocity(6000, Constants.kTimeoutMs);
	    	_armMotor.configMotionAcceleration(12000, Constants.kTimeoutMs);
			
	
	    	
	    	//Elevator Up
	    	_elevatorMotor.selectProfileSlot(0,0);
	    	_elevatorMotor.config_kF(0, 0.0525, Constants.kTimeoutMs);
	    	_elevatorMotor.config_kP(0, 0.1, Constants.kTimeoutMs);
	    	_elevatorMotor.config_kI(0, 0, Constants.kTimeoutMs);
	    	_elevatorMotor.config_kD(0, 0, Constants.kTimeoutMs);
	    	
	    	//Elevator Down
	    	_elevatorMotor.selectProfileSlot(1,0);
	    	_elevatorMotor.config_kF(1, 0.0525, Constants.kTimeoutMs);
	    	_elevatorMotor.config_kP(1, 0.08, Constants.kTimeoutMs);
	    	_elevatorMotor.config_kI(1, 0, Constants.kTimeoutMs);
	    	_elevatorMotor.config_kD(1, 0, Constants.kTimeoutMs);
			/* set acceleration and vcruise velocity - see documentation */
	    	_elevatorMotor.configMotionCruiseVelocity(30000, Constants.kTimeoutMs);
	    	_elevatorMotor.configMotionAcceleration(100000, Constants.kTimeoutMs);
			
	    	//Intakes
	    	_intakeLeftMotor.selectProfileSlot(0,0);
	    	_intakeLeftMotor.config_kF(0, 0.128, Constants.kTimeoutMs);
	    	_intakeLeftMotor.config_kP(0, 0.2, Constants.kTimeoutMs);
	    	_intakeLeftMotor.config_kI(0, 0.0001, Constants.kTimeoutMs);
	    	_intakeLeftMotor.config_kD(0, 0, Constants.kTimeoutMs);

	    	_intakeRightMotor.selectProfileSlot(0,0);
	    	_intakeRightMotor.config_kF(0, 0.128, Constants.kTimeoutMs);
	    	_intakeRightMotor.config_kP(0, 0.2, Constants.kTimeoutMs);
	    	_intakeRightMotor.config_kI(0, 0.0001, Constants.kTimeoutMs);
	    	_intakeRightMotor.config_kD(0, 0, Constants.kTimeoutMs);

			
	    	
	    	_elevatorMotor.configForwardSoftLimitThreshold(216000, Constants.kTimeoutMs);
	    	_elevatorMotor.configReverseSoftLimitThreshold(0, Constants.kTimeoutMs);
	    	_wristMotor.configForwardSoftLimitThreshold(0, Constants.kTimeoutMs);
	    	_wristMotor.configReverseSoftLimitThreshold(-3100, Constants.kTimeoutMs);
	    	_armMotor.configForwardSoftLimitThreshold(6900, Constants.kTimeoutMs);
	    	_armMotor.configReverseSoftLimitThreshold(0, Constants.kTimeoutMs);
	    	_armMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
	    	_armMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);
	    	_wristMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
	    	_wristMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);
	    	_elevatorMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
	    	_elevatorMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);
    	

		}
	}
	public void zeroMotors() {
		/* zero the sensor */
    	_elevatorMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    	_elevatorMotor.set(ControlMode.PercentOutput, 0);
    	/* zero the sensor */
    	_armMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    	_armMotor.set(ControlMode.PercentOutput, 0);
    	/* zero the sensor */
    	_wristMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    	_wristMotor.set(ControlMode.PercentOutput, 0);
	}
	
	public void spinIntake(double speed) {
		if(speed ==0) {
			_intakeLeftMotor.set(ControlMode.PercentOutput,0);
			_intakeRightMotor.set(ControlMode.PercentOutput,0);

		}else {
		_intakeLeftMotor.set(ControlMode.Velocity,4096*speed/600);
		_intakeRightMotor.set(ControlMode.Velocity,4096*speed/600);
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
	    	_armMotor.set(ControlMode.MotionMagic, Constants.ARM_UP);
	    	armUp = true;
	    }
	    public void armDown() {
	    	_armMotor.selectProfileSlot(1,0);
	    	_armMotor.set(ControlMode.MotionMagic, Constants.ARM_DOWN);
	    	armUp = false;
	    }
	    
	    public double wristFlipPosUp(int armPos) {
	    	if(wristOut) {
	    		return profileUpOut.predict(armPos);
	    	}else {
	    		return profileUpIn.predict(armPos);
	    	}
	    }
	    
	    public double wristFlipPosDown(int armPos) {
	    	double wristPos;
	    	if(wristOut) {
	    		wristPos = profileDownOut.predict(armPos);
	    	}else {
	    		wristPos = profileDownIn.predict(armPos);
	    	}
	    	System.out.println(wristOut + ", " +armPos + ", " + wristPos);
	    	return wristPos;
	    }
	    
	    public void wristSlightUp() {
	    	_wristMotor.selectProfileSlot(0,0);
	    	_wristMotor.set(ControlMode.MotionMagic, -2400);
	    	wristUp = false;
	    	manualWrist = false;
	    }
	    public void wristSlightDown() {
	    	_wristMotor.selectProfileSlot(1,0);
	    	_wristMotor.set(ControlMode.MotionMagic, Constants.WRIST_PUNT);
	    	wristUp = true;
	    	manualWrist = false;
	    }	    
	    public void wristPuntMore() {
	    	_wristMotor.selectProfileSlot(1,0);
	    	_wristMotor.set(ControlMode.MotionMagic, Constants.WRIST_PUNT_MORE);
	    	wristUp = true;
	    	manualWrist = false;
	    }
	    public void wristUp() {
	    	_wristMotor.selectProfileSlot(0,0);
	    	_wristMotor.set(ControlMode.MotionMagic, Constants.WRIST_UP);
	    	wristUp = true;
	    	manualWrist = false;
	    }
	    public void wristDown() {
	    	_wristMotor.selectProfileSlot(1,0);
	    	_wristMotor.set(ControlMode.MotionMagic, Constants.WRIST_DOWN);
	    	wristUp = false;
	    	manualWrist = false;
	    }
	    public void wristDownMore() {
	    	_wristMotor.selectProfileSlot(1,0);
	    	_wristMotor.set(ControlMode.MotionMagic, Constants.WRIST_DOWN_MORE);
	    	wristUp = false;
	    	manualWrist = false;
	    }
	    public void wristUpShot() {
	    	_wristMotor.selectProfileSlot(0,0);
	    	_wristMotor.set(ControlMode.MotionMagic, Constants.WRIST_UP_SHOT);
	    	wristUp = false;
	    	manualWrist = false;
	    }

	    public void elevatorUp() {
	    	_elevatorMotor.selectProfileSlot(0,0);
	    	_elevatorMotor.set(ControlMode.MotionMagic, Constants.ELEVATOR_UP);
	    	elevatorUp = true;
	    }
	    public void elevatorSwitchMid() {
	    	_elevatorMotor.selectProfileSlot(0,0);
	    	_elevatorMotor.set(ControlMode.MotionMagic, 190000);
	    	elevatorUp = true;
	    }

	    public void elevatorDown() {
	    	_elevatorMotor.selectProfileSlot(1,0);
	    	_elevatorMotor.set(ControlMode.MotionMagic, Constants.ELEVATOR_DOWN);
	    	elevatorUp = false;
	    	
	    }
	    
	    public int getElevatorPosition() {
	    	return _elevatorMotor.getSelectedSensorPosition(0);
	    }
	    
	    public void setElevatorPosition(int pos) {
	    	_elevatorMotor.setSelectedSensorPosition(pos, 0, Constants.kTimeoutMs);
	    }
	    
	    public int getArmPosition() {
	    	return _armMotor.getSelectedSensorPosition(0);
	    }
	    
	    public void setArmPosition(int pos) {
	    	_armMotor.setSelectedSensorPosition(pos, 0, Constants.kTimeoutMs);
	    }
	    
	    public int getWristPosition() {
	    	return _wristMotor.getSelectedSensorPosition(0);
	    }
	    
	    public void setWristPosition(int pos) {
	    	_wristMotor.setSelectedSensorPosition(pos, 0, Constants.kTimeoutMs);
	    }
	    
	    public void shiftElevator(boolean state) {
	    	elevatorSolenoid.set(state);
	    }
}
