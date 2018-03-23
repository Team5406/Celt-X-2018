package org.cafirst.frc.team5406.subsystems;


import org.cafirst.frc.team5406.robot.Constants;
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
    	
	public WPI_TalonSRX _intakeMotor= new WPI_TalonSRX(10);
	
	/*Comp Bot has a Victor for ID #9 - Intake, Practice Bot has a Talon*/
	//WPI_VictorSPX _intakeSlave1 = new WPI_VictorSPX(9);
	public WPI_VictorSPX _intakeSlave1 = new WPI_VictorSPX(9);

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
    
	public Intake(){
    	gripSolenoidHigh = new Solenoid(Constants.GRIP_HIGH);
    	elevatorSolenoid = new Solenoid(Constants.ELEVATOR_SHIFT);
    	rampSolenoid = new Solenoid(Constants.RAMP_RELEASE);
    	gripSolenoidLow = new Solenoid(Constants.GRIP_LOW);

	
	}
	public void setupMotors() {
		if(disabled) {
			disabled = false;	
	    	
	    	_elevatorSlave1.follow(_elevatorMotor);
	    	_intakeSlave1.follow(_intakeMotor);
	    	_intakeSlave1.setInverted(true);
	    	
	    	_armMotor.setSensorPhase(false);
	    	_wristMotor.setSensorPhase(false);
	
			_armMotor.configContinuousCurrentLimit(40, Constants.kTimeoutMs);
			_armMotor.configPeakCurrentLimit(60, Constants.kTimeoutMs);
			_armMotor.configPeakCurrentDuration(30, Constants.kTimeoutMs);
			_armMotor.enableCurrentLimit(true);
			_intakeMotor.configContinuousCurrentLimit(40, Constants.kTimeoutMs);
			_intakeMotor.configPeakCurrentLimit(40, Constants.kTimeoutMs);
			_intakeMotor.configPeakCurrentDuration(40, Constants.kTimeoutMs);
			_intakeMotor.enableCurrentLimit(true);
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
	
	    	_wristMotor.selectProfileSlot(0,0);
	    	_wristMotor.config_kF(0, 1, Constants.kTimeoutMs);
	    	_wristMotor.config_kP(0, 0.5, Constants.kTimeoutMs);
	    	_wristMotor.config_kI(0, 0, Constants.kTimeoutMs);
	    	_wristMotor.config_kD(0, 0, Constants.kTimeoutMs);
	    	_wristMotor.selectProfileSlot(1,0);
	    	_wristMotor.config_kF(1, 1, Constants.kTimeoutMs);
	    	_wristMotor.config_kP(1, 1, Constants.kTimeoutMs);
	    	_wristMotor.config_kI(1, 0.001, Constants.kTimeoutMs);
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
	    	_armMotor.config_kF(1, 0.2, Constants.kTimeoutMs);
	    	_armMotor.config_kP(1, 0.3, Constants.kTimeoutMs);
	    	_armMotor.config_kI(1, 0, Constants.kTimeoutMs);
	    	_armMotor.config_kD(1, 0, Constants.kTimeoutMs);
			/* set acceleration and vcruise velocity - see documentation */
	    	_armMotor.configMotionCruiseVelocity(6000, Constants.kTimeoutMs);
	    	_armMotor.configMotionAcceleration(6000, Constants.kTimeoutMs);
			
	
	    	
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
	    	_elevatorMotor.configMotionAcceleration(250000, Constants.kTimeoutMs);
			
	    	
	    	
	    	
	    	_elevatorMotor.configForwardSoftLimitThreshold(216000, Constants.kTimeoutMs);
	    	_elevatorMotor.configReverseSoftLimitThreshold(0, Constants.kTimeoutMs);
	    	_wristMotor.configForwardSoftLimitThreshold(0, Constants.kTimeoutMs);
	    	_wristMotor.configReverseSoftLimitThreshold(-2700, Constants.kTimeoutMs);
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
	    	//return 7E-05*armPos*armPos - 0.8862*armPos - 2E-11;
	    	//return  8E-05*armPos*armPos - 0.9379*armPos + 4E-11;
	    	//return 5E-05*armPos*armPos - 0.7135*armPos + 3E-11;
	    	//return 1E-08*armPos*armPos*armPos - 0.0001*armPos*armPos - 0.1143*armPos - 7E-10;
	    	return 2E-08*armPos*armPos*armPos - 0.0002*armPos*armPos - 0.0434*armPos - 8E-10;
	    	//return 8E-05*armPos*armPos - 0.965*armPos - 2E-11;



	    	//return 3E-05*armPos*armPos - 0.5823*armPos - 2E-11;
	    	//return 1E-05*armPos*armPos - 0.4935*armPos - 5E-11;
	    	//return -0.0002*armPos*armPos + 1.0437*armPos - 2800;


	    }
	    
	    public double wristFlipPosDown(int armPos) {
	    	//return 7E-05*armPos*armPos - 0.8862*armPos - 2E-11;
	    	//return  8E-05*armPos*armPos - 0.9379*armPos + 4E-11;
	    	//return 5E-05*armPos*armPos - 0.7135*armPos + 3E-11;
	    	return 6E-08*armPos*armPos*armPos - 0.0007*armPos*armPos + 1.5625*armPos - 1E-09;
	    	//return 3E-08*armPos*armPos*armPos - 0.0003*armPos*armPos + 0.212*armPos - 1E-09;
	    	//return 8E-05*armPos*armPos - 0.965*armPos - 2E-11;



	    	//return 3E-05*armPos*armPos - 0.5823*armPos - 2E-11;
	    	//return 1E-05*armPos*armPos - 0.4935*armPos - 5E-11;
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
