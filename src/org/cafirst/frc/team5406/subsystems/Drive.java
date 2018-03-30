package org.cafirst.frc.team5406.subsystems;

import org.cafirst.frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Victor;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.cafirst.frc.team5406.subsystems.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Drive extends Subsystems{
	public WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(1); //drive
	public WPI_VictorSPX _leftSlave1 = new WPI_VictorSPX(2); //left drive
	public WPI_VictorSPX _leftSlave2 = new WPI_VictorSPX(3);
	public WPI_VictorSPX _leftSlave3 = new WPI_VictorSPX(4);
	
	public WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(5);
	public WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(6);
	public WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(7);
	public WPI_VictorSPX _rightSlave3 = new WPI_VictorSPX(8); //right drive
	
	int maxVelocity = 11600; //sensor units/100ms

	public boolean disabled = false;
	
	public Drive(){

	}
	
	public void setupMotors() {
		if(disabled) {
			disabled = false;	
			_leftSlave1.follow(_frontLeftMotor);
	    	_leftSlave2.follow(_frontLeftMotor);
	    	_leftSlave3.follow(_frontLeftMotor);
	    	_rightSlave1.follow(_frontRightMotor);
	    	_rightSlave2.follow(_frontRightMotor);
	    	_rightSlave3.follow(_frontRightMotor);
	    	
	    	_frontLeftMotor.configOpenloopRamp(0.25, Constants.kTimeoutMs);
	    	_frontRightMotor.configOpenloopRamp(0.25, Constants.kTimeoutMs);
	    	_frontLeftMotor.configClosedloopRamp(0.08, Constants.kTimeoutMs);
	    	_frontRightMotor.configClosedloopRamp(0.08, Constants.kTimeoutMs);
	    	_frontLeftMotor.selectProfileSlot(0,0);
	    	_frontLeftMotor.config_kF(0, 0.09, Constants.kTimeoutMs);
	    	_frontLeftMotor.config_kP(0, 0.2, Constants.kTimeoutMs);
	    	_frontLeftMotor.config_kI(0, 0.0001, Constants.kTimeoutMs);
	    	_frontLeftMotor.config_kD(0, 0.001, Constants.kTimeoutMs);
	    	_frontRightMotor.selectProfileSlot(0,0);
	    	_frontRightMotor.config_kF(0, .09, Constants.kTimeoutMs);
	    	_frontRightMotor.config_kP(0, 0.2, Constants.kTimeoutMs);
	    	_frontRightMotor.config_kI(0, 0.0001, Constants.kTimeoutMs);
	    	_frontRightMotor.config_kD(0, 0.001, Constants.kTimeoutMs);
	    	_frontRightMotor.setSensorPhase(false);
	    	_frontRightMotor.setInverted(true);
	    	_rightSlave1.setInverted(true);
	    	_rightSlave2.setInverted(true);
	    	_rightSlave3.setInverted(true);
	    	
	    	
	    	
	    	/*_leftSlave1.setInverted(true);
	    	_leftSlave2.setInverted(true);
	    	_rightSlave1.setInverted(true);
	    	_rightSlave2.setInverted(true);*/
	    	_frontRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
			_frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
			_frontLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
			_frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
			_frontLeftMotor.configContinuousCurrentLimit(20, Constants.kTimeoutMs);
			//_frontLeftMotor.configPeakCurrentLimit(30, 0);
			//_frontLeftMotor.configPeakCurrentDuration(30, 0);
			_frontLeftMotor.enableCurrentLimit(true);
			_frontRightMotor.configContinuousCurrentLimit(20, Constants.kTimeoutMs);
			//_frontRightMotor.configPeakCurrentLimit(30, 0);
			//_frontRightMotor.configPeakCurrentDuration(30, 0);
			_frontRightMotor.enableCurrentLimit(true);
			

		}
	}
	
	
}
