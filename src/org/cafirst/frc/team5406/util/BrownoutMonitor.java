package org.cafirst.frc.team5406.util;

import java.util.ArrayList;
import java.util.List;

import org.cafirst.frc.team5406.robot.Robot;

/**
 * Created based off the paper found here:
 * https://www.chiefdelphi.com/media/papers/3364
 */
public class BrownoutMonitor {
  
  private static double BATTERY_VOLTAGE = 12.5;
  private static final double BATTERY_RESISTANCE = 0.02; // 0.012
  
  private Motors motorType;
  private int motorsPerSide;
  private double min_voltage;
  
  public BrownoutMonitor(Motors motorType, int motorsPerSide, double min_voltage){
    this.motorType = motorType;
    this.motorsPerSide = motorsPerSide;
    this.min_voltage = min_voltage;
    
    updateBatteryVoltage();
  }
  
  public void updateBatteryVoltage(){
	  if(Robot.pdp != null){
		  BATTERY_VOLTAGE = Robot.pdp.getVoltage() - 0.5;
	  }
	  else{
		  BATTERY_VOLTAGE = 12;
	  }
  }
  
  public double getEstimatedCurrent(double leftVoltage, double leftSpeed, double rightVoltage, double rightSpeed){
    double leftCurrent = motorsPerSide * motorType.getEstimatedCurrent(leftVoltage, leftSpeed);
    double rightCurrent = motorsPerSide * motorType.getEstimatedCurrent(rightVoltage, rightSpeed);
    
    return leftCurrent + rightCurrent;
  }
  
  public double getEstimatedVoltageDrop(double leftVoltage, double leftSpeed, double rightVoltage, double rightSpeed){
    return BATTERY_RESISTANCE * getEstimatedCurrent(leftVoltage, leftSpeed, rightVoltage, rightSpeed);
  }
  
  public double getScalingFactor(double leftVoltage, double leftSpeed, double rightVoltage, double rightSpeed){
	 double battery_voltage = BATTERY_VOLTAGE - (Robot.compressor != null ? (!Robot.compressor.getPressureSwitchValue() ? 1.5 : 0) : 0);
    List<Double> gammaVals = new ArrayList<>();
    for(int i = 0; i < 2; i++){
      for(int j = 0; j < 2; j++){
        double val1 = (motorType.R_m * (battery_voltage - min_voltage) /
                (motorsPerSide * BATTERY_RESISTANCE)) * (i == 0 ? 1.0 : -1.0);
        double val2 = (motorType.K_i * rightSpeed) + (motorType.K_i * leftSpeed) * (j == 0 ? 1.0 : -1.0);
        double val3 = rightVoltage + leftVoltage * (j == 0 ? 1.0 : -1.0);
        gammaVals.add((val1 + val2) / val3);
      }
    }
    
    if(battery_voltage - getEstimatedVoltageDrop(leftVoltage, leftSpeed, rightVoltage, rightSpeed) < min_voltage){
      double gammaVal = -1.0;
      for(double val : gammaVals){
        if(val >= 0 && val <= 1.0 && val > gammaVal){
          gammaVal = val;
        }
      }
      if(gammaVal == -1)
        gammaVal = 1.0;
      return gammaVal;
    }
    return 1.0;
    
  }
  
  
}