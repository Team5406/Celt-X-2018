package org.cafirst.frc.team5406.util;

public class Util {
	public Util(){
		
	}
	
	public double applyDeadband(double value){
		return applyDeadband(value, 0.2);
	}
	
	public double applyDeadband(double value, double deadband){
		if ((-1*deadband) < value && value < deadband){
			return 0;
		}else{
			return value;
		}
	}
}
