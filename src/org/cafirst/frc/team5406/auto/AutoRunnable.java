package org.cafirst.frc.team5406.auto;

import java.util.ArrayList;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

class AutoRunnable implements java.lang.Runnable
{
	private double targetAngle;
	private double accumI = 0.0;
	public double lastAngle = 0;
	private double previousError = 0.0;
	private int lastPoint = 0;
	private double targetTime = 0;
	private boolean isBackwards = false;
	private boolean drivePathDone = false;
	
	private Drive robotDrive;
	private ArrayList<double[]> motionProfile;
	
	public AutoRunnable(Drive _robotDrive, ArrayList<double[]> drivePath, boolean _isBackwards)
	{
		robotDrive = _robotDrive;
		motionProfile = drivePath;
		isBackwards = _isBackwards;
	}
	
    public void run() {
    	
    	double leftSpeed = 0;
		double rightSpeed = 0;
		double dSpeed = 0;
		double speedChangeMultiplier = 0;
		double targetSpeedLeft = 0;
		double targetSpeedRight = 0;
		double currentAngle = Constants.navX.getYaw();

	    
    	
    	if(!drivePathDone) {
    		double elapsedTime = Timer.getFPGATimestamp() - Constants.AutoStartTime; 	
    		System.out.println("eTime: " + elapsedTime + "tTime: " + targetTime + "Cur:" +  currentAngle + "Tar:"+ motionProfile.get(lastPoint)[5]);
			speedChangeMultiplier = calcSpeed(motionProfile.get(lastPoint)[5] - currentAngle);
	    	int numPoints = motionProfile.size();
	    	if (elapsedTime > targetTime) {
	    		if(lastPoint < numPoints -2) {
	    			System.out.println(lastPoint + " ("+ (numPoints-2) + "), "+ motionProfile.get(lastPoint)[0]/1000 + + motionProfile.get(lastPoint)[2] + ", " + motionProfile.get(lastPoint)[1]);
		    		targetTime += motionProfile.get(lastPoint)[0]/1000;
		    		targetSpeedLeft = motionProfile.get(lastPoint)[2]*(4096/600)*Constants.driveGearRatio;
					leftSpeed =targetSpeedLeft+Math.signum(targetSpeedLeft)*targetSpeedLeft*speedChangeMultiplier; //-1*400-1200 = -1800
		    		targetSpeedRight = motionProfile.get(lastPoint)[4]*(4096/600)*Constants.driveGearRatio;
					rightSpeed = targetSpeedRight-Math.signum(targetSpeedRight)*targetSpeedRight*speedChangeMultiplier; //400-1200 = -800
					System.out.println("LS: "+ leftSpeed + ", LT: " + targetSpeedLeft + ", LA:" + robotDrive._frontLeftMotor.getSelectedSensorVelocity(0) + ", RS: "+ rightSpeed + ", RT: " + targetSpeedRight + ", RA:" + robotDrive._frontRightMotor.getSelectedSensorVelocity(0));
		    		robotDrive._frontRightMotor.set(ControlMode.Velocity,-1*(isBackwards ? leftSpeed : rightSpeed));
		    		robotDrive._frontLeftMotor.set(ControlMode.Velocity, -1*(isBackwards ? rightSpeed : leftSpeed));
		    		lastPoint++;
	    		}else {
	    			robotDrive._frontLeftMotor.set(ControlMode.Velocity,0);
	    			robotDrive._frontRightMotor.set(ControlMode.Velocity,0);
	    			drivePathDone = true;
	    		}
	    	}
    	}
    }
    
    public boolean IsDrivePathDone() { return drivePathDone; }
    
    public void AlterPath(ArrayList<double[]> _motionProfile, boolean _isBackwards)
    {
    	isBackwards = _isBackwards;
    	
    	motionProfile = _motionProfile;
    	lastPoint = 0;
    	
    	drivePathDone = false;
    }
    
    public double calcSpeed(double currentError){
		
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
}
