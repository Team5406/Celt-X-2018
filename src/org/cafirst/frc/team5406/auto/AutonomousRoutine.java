package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public abstract class AutonomousRoutine implements Cloneable{
	
	protected final int kTimeoutMs = 10;
	
	protected int autoLoop;
	protected int step;
	protected double speed;
	protected Robot robot;
	
	protected WPI_TalonSRX _frontLeftMotor;
	protected WPI_TalonSRX _frontRightMotor;
	
	protected WPI_TalonSRX _intakeMotor;
	
	public AutonomousRoutine(Robot _robot, WPI_TalonSRX _frontLeft, WPI_TalonSRX _frontRight, WPI_TalonSRX _intake)
	{
		robot = _robot;
		_frontLeftMotor = _frontLeft;
		_frontRightMotor = _frontRight;
		
		_intakeMotor = _intake;
	}
	
	public abstract void init();
	public abstract void periodic();
	
	@Override
	public Object clone() throws CloneNotSupportedException
	{
		return super.clone();
		
	}
}
