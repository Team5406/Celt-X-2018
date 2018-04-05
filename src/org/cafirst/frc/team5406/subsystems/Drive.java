package org.cafirst.frc.team5406.subsystems;

import org.cafirst.frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Victor;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.cafirst.frc.team5406.subsystems.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
	public boolean angleDriveDone = false;
	
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
	
	public void turnToAngle(double _targetAngle, boolean _isClockwise)
	{
		AngleRunner runner = new AngleRunner(_targetAngle, _isClockwise);
		Notifier not = new Notifier(runner);
		
		angleDriveDone = false;
		
		runner.SetNotifier(not);
		not.startPeriodic(0.005);
	}
	
	class AngleRunner implements java.lang.Runnable
	{
		double targetAngle;
		double startAngle =0;
		boolean isClockwise;
		
		double startTime = 0;
		double dTime = 0;
		double startSpeed = 5000; //native units
		
		boolean isDrivePathDone = false;
		
		double lStart, rStart;
		double dPos;
		
		double targetTime;
		
		double accumI = 0;
		double previousError = 0;
		
		Notifier notifier;
		
		public AngleRunner(double _targetAngle, boolean _isClockwise)
		{
			targetAngle = degTo360(_targetAngle);
		    double curYaw = degTo360(Constants.navX.getYaw());
		    startAngle = curYaw;
		    // Note:  The navx outputs values from 0 to 180 degrees and -180 to 0 degrees as the
		    // robot spins clockwise. Convert this to a 0 to 360 degrees scale.
		    
		    
			double d = Math.abs(targetAngle - curYaw) % 360; 
			double r = d > 180 ? 360 - d : d;

			//calculate sign 
			int sign = ( targetAngle- curYaw >= 0 &&  targetAngle - curYaw <= 180) || (targetAngle - curYaw <=-180 && targetAngle- curYaw>= -360) ? 1 : -1; 
			
			
			isClockwise = (sign > 0);
			
			startTime = Timer.getFPGATimestamp();
		}

		@Override
		public void run() {
			
			double currentAngle = degTo360(Constants.navX.getYaw());

			if(Math.abs(currentAngle - targetAngle) < 5)
				isDrivePathDone = true;
			System.out.println("isDrivePathDone: " + isDrivePathDone);
			System.out.println("tAngle: " + targetAngle + " cAngle: " + currentAngle);

			if(!isDrivePathDone)
			{
				double leftSpeed = 0;
				double rightSpeed = 0;
				double dSpeed = 0;
				double speedChangeMultiplier = 0;
				double targetSpeedLeft = 0;
				double targetSpeedRight = 0;
				
				double targetSpeed = startSpeed;
				double accAngle = 0;
				
				if(subtractAngles(targetAngle,startAngle) < 100) {
					accAngle = targetAngle/2;
				}else {
					accAngle = 50;
				}
				
				
				//double lError = currentAngle - ((_frontLeftMotor.getSelectedSensorPosition(0) - lStart) / Constants.TICKS_PER_CIRCLE * 360);
				
				double d = Math.abs(subtractAngles(targetAngle,startAngle) - currentAngle) % 360; 
				int sign = ( targetAngle- currentAngle >= 0 &&  subtractAngles(targetAngle,startAngle) - currentAngle <= 180) || (targetAngle - currentAngle <=-180 && targetAngle- currentAngle>= -360) ? 1 : -1; 

				double lError = sign*(d > 180 ? 360 - d : d);
				
				double m = subtractAngles(startAngle,currentAngle);
				
				if(m < accAngle) {
					targetSpeed = 1000+startSpeed*m/accAngle;
				}else if (m >= accAngle && m <= subtractAngles(subtractAngles(targetAngle,startAngle),accAngle)) {
					targetSpeed = 1000+startSpeed;
				} else if(m > subtractAngles(targetAngle,startAngle) - accAngle) {
					targetSpeed = (startSpeed+1000)*(subtractAngles(subtractAngles(targetAngle,startAngle),m))/accAngle;
				}
				
				speedChangeMultiplier = calcSpeed(lError);

				targetSpeedLeft = (isClockwise?1:-1)*targetSpeed;
				targetSpeedRight =(isClockwise?-1:1)* targetSpeed;
				
				

				leftSpeed = targetSpeedLeft
						+ Math.signum(targetSpeedLeft) * targetSpeedLeft * speedChangeMultiplier*0.01;
				
				rightSpeed = targetSpeedRight
						- Math.signum(targetSpeedRight) * targetSpeedRight * speedChangeMultiplier*0.01;
				System.out.println("accAngle: " +accAngle + "m: " + m + "err: " + lError + "; mult: " + speedChangeMultiplier +"; lSpeed: " + leftSpeed + "; rSpeed: " + rightSpeed);

				_frontLeftMotor.set(ControlMode.Velocity, targetSpeedLeft);
				_frontRightMotor.set(ControlMode.Velocity, targetSpeedRight);
				
			}
			else
			{
				angleDriveDone = true;
				_frontLeftMotor.set(ControlMode.Velocity, 0);
				_frontRightMotor.set(ControlMode.Velocity, 0);
				
				notifier.stop();
			}
		}
		
		public double subtractAngles (double angle1, double angle2) {
			double m = Math.abs(angle1 - angle2) % 360; 
			return (m > 180 ? 360 - m : m);
		}
		
		public double degTo360(double deg) {
			if (deg > -180 && deg < 0) {
		        return 360 + deg;
		    } else {
		        return deg;
		    }
		}
		
		public double calcSpeed(double currentError)
		{
			double valP = Constants.GYRO_PID_P * currentError;
	 		double valI = accumI;
	 		double valD = Constants.GYRO_PID_D * (previousError - currentError);
	 		if(Math.abs(valD) > Math.abs(valP)) valD = valP; // Limit so that D isn't the driving number
	 		accumI += Constants.GYRO_PID_I;
	 		
	 		//If we overshoot, reset the I
	 		if(Math.signum(previousError) != Math.signum(currentError)){ 
	 			accumI = 0; 
	 			valI = 0;
	 		}
	 		double speed = valP + (valI * (currentError > 0 ? 1.0 : -1.0)) - valD;
	 		previousError = currentError;
	 		return speed;
		}
		
		public void SetNotifier(Notifier _not)
		{
			notifier = _not;			
		}
		
	}
	
	
}
