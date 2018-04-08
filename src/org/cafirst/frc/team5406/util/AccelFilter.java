package org.cafirst.frc.team5406.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AccelFilter {
	
	private double accelRate;
	
	private double currentValue;
	private double targetValue;
	
	public AccelFilter(double accelRate){
		this.accelRate = accelRate;
	}
	
	public void set(double value){
		this.targetValue = value;
	}
	
	public void reset(){
		targetValue = 0.0;
		currentValue = 0.0;
	}
	
	public double get(){
		if(targetValue > currentValue){
			currentValue += accelRate;
		}
		else if(targetValue < currentValue){
			currentValue -= accelRate;
		}
//		currentValue = Util.limitValue(currentValue, targetValue);
		SmartDashboard.putNumber("current", currentValue);
		SmartDashboard.putNumber("target", targetValue);
		
		return currentValue;
	}
	
	public double getTarget(){
		return targetValue;
	}
	
}