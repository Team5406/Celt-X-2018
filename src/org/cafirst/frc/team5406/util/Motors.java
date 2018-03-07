package org.cafirst.frc.team5406.util;

/**
 * Created by kestin.
 */
public class Motors {
  
  public static Motor CIM = new Motor(5330, 2.7, 337, 2.41, 131);
  public static Motor MiniCIM = new Motor(5840, 3, 215, .141, 89);
  public static Motor BagMotor = new Motor(13180, 1.8, 149, 0.43, 53);
  public static Motor _775Pro = new Motor(18730, 0.7, 347, 0.71, 134);
  public static Motor AndyMark_RS775_125 = new Motor(5800, 1.6, 43, 0.28, 18);
  public static Motor BaneBots_RS_775_18V = new Motor(13050, 2.7, 246, 0.72, 97);
  public static Motor AndyMark_9015 = new Motor(14270, 3.7, 134, 0.36, 71);
  public static Motor BaneBots_RS_550 = new Motor(19000, 0.4, 190, 0.38, 84);
  
  
  public static class Motor {
    public double free_speed, free_current, max_power, stall_torque, stall_current;
    public double R_m, K_i;
    
    public Motor(double free_speed, double free_current, double max_power, double stall_torque, double stall_current){
      this.free_speed = free_speed;
      this.free_current = free_current;
      this.max_power = max_power;
      this.stall_torque = stall_torque;
      this.stall_current = stall_current;
      
      this.R_m = (12 / this.stall_current);
      this.K_i = -(this.free_current * this.R_m - 12) / (this.free_speed * 2 * Math.PI / 60.0);  // (this.free_speed * Math.PI / 180.0)
    }
    
    /**
     * @param voltage - The requested voltage (V) from the speed controller.
     * @param speed   - The actual speed (rad/s) of the motor.
     * @return The estimated current draw of the motor in Amps (A).
     */
    public double getEstimatedCurrent(double voltage, double speed){
      return Math.abs((voltage - this.K_i * speed) / this.R_m);
    }
  }
  
}
