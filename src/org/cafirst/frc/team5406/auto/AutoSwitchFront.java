package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.auto.MotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.cafirst.frc.team5406.auto.AutonomousRoutine;



public class AutoSwitchFront  extends AutonomousRoutine{
	private Intake robotIntake;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
	MotionProfile motionProfiler = new MotionProfile();
	int wristUpDelay = 0;
	boolean drivePathDone = false;
        double startTime = 0;
        int lastPoint = 0;
        double targetTime = 0;
    		
        class AutoRunnable implements java.lang.Runnable {
    	private double targetAngle;
    	private double accumI = 0.0;
    	public double lastAngle = 0;
    	private double previousError = 0.0;
    	
    	
    	
	    public void run() {
	    	
	    	double leftSpeed = 0;
			double rightSpeed = 0;
			double dSpeed = 0;
			double speedChangeMultiplier = 0;
			double targetSpeedLeft = 0;
			double targetSpeedRight = 0;
			double currentAngle = Constants.navX.getYaw();

		    
	    	
	    	if(!drivePathDone) {
	    		System.out.println("Cur:" +  currentAngle + "Tar:"+ motionProfiler.motionProfile.get(lastPoint)[5]);
				speedChangeMultiplier = calcSpeed(motionProfiler.motionProfile.get(lastPoint)[5] - currentAngle);
	    		double elapsedTime = Timer.getFPGATimestamp() - startTime; 	
		    	int numPoints = motionProfiler.motionProfile.size();
		    	if (elapsedTime > targetTime) {
		    		if(lastPoint < numPoints -2) {
		    			//System.out.println(lastPoint + ", " + motionProfile.get(lastPoint)[2]/1000 + ", " + motionProfile.get(lastPoint)[1]/1000);
			    		targetTime += motionProfiler.motionProfile.get(lastPoint)[0]/1000;
			    		targetSpeedLeft = motionProfiler.motionProfile.get(lastPoint)[2]*(4096/600)*Constants.driveGearRatio;
						leftSpeed =targetSpeedLeft+Math.signum(targetSpeedLeft)*targetSpeedLeft*speedChangeMultiplier; //-1*400-1200 = -1800
			    		targetSpeedRight = motionProfiler.motionProfile.get(lastPoint)[4]*(4096/600)*Constants.driveGearRatio;
						rightSpeed = targetSpeedRight-Math.signum(targetSpeedRight)*targetSpeedRight*speedChangeMultiplier; //400-1200 = -800
						System.out.println("LS: "+ leftSpeed + ", LT: " + targetSpeedLeft + ", LA:" + robotDrive._frontLeftMotor.getSelectedSensorVelocity(0) + ", RS: "+ rightSpeed + ", RT: " + targetSpeedRight + ", RA:" + robotDrive._frontRightMotor.getSelectedSensorVelocity(0));
			    		robotDrive._frontRightMotor.set(ControlMode.Velocity,rightSpeed);
			    		robotDrive._frontLeftMotor.set(ControlMode.Velocity, leftSpeed);
			    		lastPoint++;
		    		}else {
		    			robotDrive._frontLeftMotor.set(ControlMode.Velocity,0);
		    			robotDrive._frontRightMotor.set(ControlMode.Velocity,0);
		    			drivePathDone = true;
		    		}
		    	}
	    	}

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
	Notifier _autoLoop = new Notifier(new AutoRunnable());

	public AutoSwitchFront(Drive _robotDrive, Intake _robotIntake){
		super("2 - Switch Front");
		robotDrive = _robotDrive;
		robotIntake = _robotIntake;
	}
	
	public void init(){
		robotIntake.setupMotors();
		robotDrive.setupMotors();
    	robotIntake._wristMotor.set(ControlMode.MotionMagic, robotIntake.getWristPosition());

    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    	startTime = Timer.getFPGATimestamp();
         lastPoint = 0;
        targetTime = 0;
        motionProfiler.motionProfile = new ArrayList<double[]>();
 		drivePathDone = false;
         autoStep = 0;
         Constants.navX.zeroYaw();
         wristUpDelay = 0;
	}
	
	public void end(){
		_autoLoop.stop();
	}

	public void periodic(){
		switch(autoStep) {
	   	case 0:

			if(!robotIntake.wristZeroed) {
				robotIntake._wristMotor.configForwardSoftLimitEnable(false, Constants.kTimeoutMs);
				robotIntake._wristMotor.configReverseSoftLimitEnable(false, Constants.kTimeoutMs);
				robotIntake._wristMotor.set(0.5);
				System.out.print(robotIntake._wristMotor.getOutputCurrent());
				if(robotIntake._wristMotor.getOutputCurrent()>15) {
					robotIntake.setWristPosition(150);
					robotIntake._wristMotor.configForwardSoftLimitEnable(false, Constants.kTimeoutMs);
					robotIntake._wristMotor.configReverseSoftLimitEnable(false, Constants.kTimeoutMs);
					robotIntake.wristUp();
					robotIntake.wristZeroed = true;
				}
			}
			
			if(!robotIntake.armZeroed) {
				robotIntake._armMotor.set(-0.5);
				robotIntake._armMotor.configForwardSoftLimitEnable(false, Constants.kTimeoutMs);
				robotIntake._armMotor.configReverseSoftLimitEnable(false, Constants.kTimeoutMs);

				if(robotIntake._armMotor.getOutputCurrent()>5) {
					robotIntake._armMotor.setSelectedSensorPosition(-10, 0, Constants.kTimeoutMs);
					robotIntake._armMotor.configForwardSoftLimitEnable(false, Constants.kTimeoutMs);
					robotIntake._armMotor.configReverseSoftLimitEnable(false, Constants.kTimeoutMs);
					robotIntake.armDown();
					robotIntake.armZeroed = true;
				}
			}
			
			if(!robotIntake.elevatorZeroed) {
				robotIntake._elevatorMotor.set(-0.5);
				robotIntake._elevatorMotor.configForwardSoftLimitEnable(false, Constants.kTimeoutMs);
				robotIntake._elevatorMotor.configReverseSoftLimitEnable(false, Constants.kTimeoutMs);
				if(robotIntake._elevatorMotor.getOutputCurrent()>5) {
					robotIntake.setElevatorPosition(-500);
					robotIntake._elevatorMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
					robotIntake._elevatorMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);
					robotIntake.elevatorDown();
					robotIntake.elevatorZeroed = true;
				}
			}
			
			if(robotIntake.elevatorZeroed && robotIntake.armZeroed && robotIntake.wristZeroed) {
				autoStep++;
			}
			break;
	   	case 1:
	   	   String gameData;
	   	   gameData = DriverStation.getInstance().getGameSpecificMessage();
           if(gameData.length() > 0)
           {
        	   ArrayList<Point2D> left = new ArrayList<Point2D>();
        	   left.add(new Point2D.Double(0, 0));
				
			  if(gameData.charAt(0) == 'L')
			  {
				  left.add(new Point2D.Double(-55, 109));
				  motionProfiler.bezierPoints(left, 0, 5);
			  } else {
				  left.add(new Point2D.Double(40, 108));
				  motionProfiler.bezierPoints(left, 0, 355);
			  }
				
			  _autoLoop.startPeriodic(0.005);
			  autoStep++;
           }
           break;
	   	case 2:
	   		if(drivePathDone) {
	   			autoStep++;
	   		}
	   		break;
	   	case 3:
	   		robotIntake.wristSlightDown();
   	   		robotIntake.needsWristUp = true;
   	   		if(robotIntake._wristMotor.getSelectedSensorPosition(0) < -350) {
   	   		robotIntake._intakeMotor.set(0.7);
   	   		}
	   		break;
	   	case 4:
	   		wristUpDelay++;
	   		if(wristUpDelay > 200) {
	   			robotIntake._intakeMotor.set(0);
	   			robotIntake.wristUp();
	   		}
	   	}
	}
}
