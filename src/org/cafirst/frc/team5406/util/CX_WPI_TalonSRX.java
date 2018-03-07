package org.cafirst.frc.team5406.util;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class CX_WPI_TalonSRX extends WPI_TalonSRX {
  
  private double setPoint = 0.0;
  private double speedMultiplier = 1.0;
  private boolean enableCurrentProtection = false;
  
  public CX_WPI_TalonSRX(int deviceNumber){
    super(deviceNumber);
  }
  
  public void enableCurrentProtection(boolean enable){
    enableCurrentProtection = enable;
  }
  
  @Override
  public void set(double value){
    setPoint = value;
    value *= (enableCurrentProtection) ? speedMultiplier : 1.0;
    super.set(value);
  }
  
  public void set(ControlMode mode,
          double value,
          double value2) {
	    setPoint = value;
	    value *= (enableCurrentProtection && mode == ControlMode.PercentOutput) ? speedMultiplier : 1.0;
	    super.set(mode, value, value2);
	  }
  
  public double getSetpoint(){
    return setPoint;
  }
  
  public void setSpeedMultiplier(double value){
    speedMultiplier = value;
  }
}
