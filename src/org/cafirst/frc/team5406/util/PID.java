package org.cafirst.frc.team5406.util;

public class PID {
	
	private double kP = 0.0;
	private double kI = 0.0;
	private double kD = 0.0;
	private double accumI = 0.0;
	
	private double previousError = 0.0;
	
	private double desiredPosition = 0.0;
	
	public PID(double kP, double kI, double kD){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}
	
	public PID(){}
	
	public void initPID(double desiredPosition, double currentPosition){
		previousError = desiredPosition - currentPosition;
		this.desiredPosition = desiredPosition;
		accumI = 0.0;
	}
	
	public double getError(double currentPosition){
		return desiredPosition - currentPosition;
	}
	
	public double calcSpeed(double currentPosition){
		
		double currentError = desiredPosition - currentPosition;
		
		double valP = kP * currentError;
		double valI = accumI;
		double valD = kD * (previousError - currentError);
		if(Math.abs(valD) > Math.abs(valP)) valD = valP; // Limit so that D isn't the driving number
		accumI += kI;
		
		//If we overshoot, reset the I
		if(Math.signum(previousError) != Math.signum(currentError)){ 
			accumI = 0; 
			valI = 0;
		}
		
		double speed = valP + (valI * (desiredPosition > currentPosition ? 1.0 : -1.0)) - valD;

		previousError = currentError;
		
		return speed;
	}
	
	private int timesTrue = 0;
	public boolean isDone(double currentPosition, double deadband){
		if(Math.abs(desiredPosition - currentPosition) <= deadband){
			if(timesTrue >= 3){
				return true;
			}
			timesTrue++;
		}
		else timesTrue = 0;
		
		return false;
	}
	
	public void setDesiredPosition(double desiredPosition){
		this.desiredPosition = desiredPosition;
	}
	
	public double getDesiredPosition(){
		return desiredPosition;
	}
	
	public double getP(){
		return this.kP;
	}
	
	public double getI(){
		return this.kI;
	}
	
	public double getD(){
		return this.kD;
	}
	
	public void resetAccumI(){
		accumI = 0.0;
	}
	
	public void setConstants(double kP, double kI, double kD){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}
	
	
	
}