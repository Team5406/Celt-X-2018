package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.auto.MotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;

import org.cafirst.frc.team5406.util.AccelFilter;
import org.cafirst.frc.team5406.util.PID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.cafirst.frc.team5406.auto.AutonomousRoutine;



public class AutoSwitchScalePrep  extends AutonomousRoutine{
	private Intake robotIntake;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
    String gameData;
	ArrayList<Point2D> left = new ArrayList<Point2D>();
	
    private PID turnToAnglePid = new PID();
    private AccelFilter turnAccelFilter = new AccelFilter(0.04);	
    int k = 0;
	private boolean turnToFirstRun = true;


	boolean driveBackwards = false;
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
    	boolean droveLast = false;
    	
    	
		public void run() {

			double leftSpeed = 0;
			double rightSpeed = 0;
			double dSpeed = 0;
			double speedChangeMultiplier = 0;
			double targetSpeedLeft = 0;
			double targetSpeedRight = 0;
			double currentAngle = Constants.navX.getYaw();

			if (!drivePathDone) {
				double elapsedTime = Timer.getFPGATimestamp() - startTime;
				/*System.out.println("eTime: " + elapsedTime + "tTime: " + targetTime + "Cur:" + currentAngle + "Tar:"
						+ motionProfiler.motionProfile.get(lastPoint)[5]);*/
				int numPoints = motionProfiler.motionProfile.size();
				if (elapsedTime > targetTime || droveLast) {
					if (lastPoint < numPoints - 2) {
						speedChangeMultiplier = calcSpeed(motionProfiler.motionProfile.get(lastPoint)[5] - currentAngle);
						if (elapsedTime > targetTime + motionProfiler.motionProfile.get(lastPoint)[0] / 1000) {
							targetSpeedLeft = 0;
							targetSpeedRight = 0;
							double profilesSkipped = 0;

							while (elapsedTime > targetTime + motionProfiler.motionProfile.get(lastPoint)[0] / 1000) {
								profilesSkipped += motionProfiler.motionProfile.get(lastPoint)[0] / 1000;
								targetTime += motionProfiler.motionProfile.get(lastPoint)[0] / 1000;

								targetSpeedLeft += (motionProfiler.motionProfile.get(lastPoint)[0] / 1000)
										* motionProfiler.motionProfile.get(lastPoint)[2] * (4096 / 600)
										* Constants.driveGearRatio;
								targetSpeedRight += (motionProfiler.motionProfile.get(lastPoint)[0] / 1000)
										* motionProfiler.motionProfile.get(lastPoint)[4] * (4096 / 600)
										* Constants.driveGearRatio;
								lastPoint++;
							}

							double leftOverTime = elapsedTime - targetTime;
							profilesSkipped += leftOverTime;

							targetSpeedLeft += leftOverTime * motionProfiler.motionProfile.get(lastPoint)[2]
									* (4096 / 600) * Constants.driveGearRatio;
							targetSpeedLeft /= profilesSkipped;
							leftSpeed = targetSpeedLeft
									+ Math.signum(targetSpeedLeft) * targetSpeedLeft * speedChangeMultiplier; // -1*400-1200
																												// =
																												// -1800
							targetSpeedRight += leftOverTime * motionProfiler.motionProfile.get(lastPoint)[4]
									* (4096 / 600) * Constants.driveGearRatio;
							targetSpeedRight /= profilesSkipped;
							rightSpeed = targetSpeedRight
									- Math.signum(targetSpeedRight) * targetSpeedRight * speedChangeMultiplier; // 400-1200
																												// =
																												// -800

							droveLast = true;

							/*System.out.println(lastPoint + " (" + (numPoints - 2) + "), "
									+ motionProfiler.motionProfile.get(lastPoint)[0] / 1000
									+ +motionProfiler.motionProfile.get(lastPoint)[2] + ", "
									+ motionProfiler.motionProfile.get(lastPoint)[1]);*/
						} else {
							droveLast = false;
							targetTime += motionProfiler.motionProfile.get(lastPoint)[0] / 1000;
							targetSpeedLeft = motionProfiler.motionProfile.get(lastPoint)[2] * (4096 / 600)
									* Constants.driveGearRatio;
							leftSpeed = targetSpeedLeft
									+ Math.signum(targetSpeedLeft) * targetSpeedLeft * speedChangeMultiplier; // -1*400-1200
																												// =
																												// -1800
							targetSpeedRight = motionProfiler.motionProfile.get(lastPoint)[4] * (4096 / 600)
									* Constants.driveGearRatio;
							rightSpeed = targetSpeedRight
									- Math.signum(targetSpeedRight) * targetSpeedRight * speedChangeMultiplier; // 400-1200
																												// =
																												// -800

							/*System.out.println("LS: " + leftSpeed + ", LT: " + targetSpeedLeft + ", LA:"
									+ robotDrive._frontLeftMotor.getSelectedSensorVelocity(0) + ", RS: " + rightSpeed
									+ ", RT: " + targetSpeedRight + ", RA:"
									+ robotDrive._frontRightMotor.getSelectedSensorVelocity(0));*/
							lastPoint++;
						}
						
						if(driveBackwards) {
							robotDrive._frontRightMotor.set(ControlMode.Velocity, -1 * leftSpeed);
							robotDrive._frontLeftMotor.set(ControlMode.Velocity, -1 * rightSpeed);
						}else {
							robotDrive._frontRightMotor.set(ControlMode.Velocity, rightSpeed);
							robotDrive._frontLeftMotor.set(ControlMode.Velocity, leftSpeed);
						}
							
							
					} else {
						robotDrive._frontLeftMotor.set(ControlMode.Velocity, 0);
						robotDrive._frontRightMotor.set(ControlMode.Velocity, 0);
						drivePathDone = true;

						System.out.println("DELTA TIME: " + elapsedTime);
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

	public AutoSwitchScalePrep(Drive _robotDrive, Intake _robotIntake){
		super("6 - Switch with Scale Prep");
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
    	turnToAnglePid.setConstants(Constants.GYRO_PID_P, Constants.GYRO_PID_I, Constants.GYRO_PID_D);

 		drivePathDone = false;
         autoStep = 0;
         Constants.navX.zeroYaw();
         wristUpDelay = 0;
	}
	
	public void end(){
		_autoLoop.stop();
	}

	public void periodic(){
//		System.out.println(autoStep);
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
	   	   gameData = DriverStation.getInstance().getGameSpecificMessage();
           if(gameData.length() > 0)
           {
        	   
        	   ArrayList<Point2D> left = new ArrayList<Point2D>();
        	   left.add(new Point2D.Double(0, 0));
				
			  if(gameData.charAt(0) == 'L')
			  {
				  left.add(new Point2D.Double(-55, 115));
				  motionProfiler.bezierPoints(left, 0, 5, 9, 1);
			  } else {
				  left.add(new Point2D.Double(40, 113));
				  motionProfiler.bezierPoints(left, 0, -5, 9, 2);
			  }
			  driveBackwards = false;
			  _autoLoop.startPeriodic(0.005);
			  autoStep++;
           }
           break;
	   	case 2:
	   		if(drivePathDone) {
	   			 _autoLoop.stop();
	   			 autoStep++;
	   		}
	   		break;
	   	case 3:
			robotIntake.wristPuntMore();
			robotIntake.needsWristUp = true;
			if (robotIntake._wristMotor.getSelectedSensorPosition(0) < -350) {
				robotIntake._intakeLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
				robotIntake._intakeRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
				robotIntake.spinIntake(500);
				autoStep++;
			}
	   		break;
	   	case 4:
	   		if(robotIntake._intakeLeftMotor.getSelectedSensorPosition(0) + robotIntake._intakeRightMotor.getSelectedSensorPosition(0)> 50000) {
	   			robotIntake.spinIntake(0);
	   			robotIntake.wristUp();
	   			autoStep++;
	   		}
	   		break;
	   	case 5:
 	        
	    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	startTime = Timer.getFPGATimestamp();
	         lastPoint = 0;
	        targetTime = 0;
	 		drivePathDone = false;

		   left = new ArrayList<Point2D>();
    	   left.add(new Point2D.Double(0, 0));
			
			  if(gameData.charAt(0) == 'L')
			  {
				  left.add(new Point2D.Double(-35, 105));
				  motionProfiler.bezierPoints(left, 5, 0, 9, 1);
			  } else {
				  left.add(new Point2D.Double(35, 103));
				  motionProfiler.bezierPoints(left, -5, 0, 9, 2);
			  }
		  
		  driveBackwards = true;
		  _autoLoop.startPeriodic(0.005);
		  autoStep++;
		  break;
		  
	   	case 6:
			if (drivePathDone) {
				  _autoLoop.stop();
				robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
				robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
				startTime = Timer.getFPGATimestamp();
				lastPoint = 0;
				targetTime = 0;
				motionProfiler.motionProfile = new ArrayList<double[]>();
				drivePathDone = false;

				left = new ArrayList<Point2D>();
				left.add(new Point2D.Double(0, 0));

				if (gameData.charAt(0) == 'L') {
					left.add(new Point2D.Double(0, 65));
					motionProfiler.bezierPoints(left, 0, 0, 9, 1);
				} else {
					left.add(new Point2D.Double(0, 60));
					motionProfiler.bezierPoints(left, 0, 0, 9, 2);
				}

				driveBackwards = false;
				robotIntake.wristDownMore();
				robotIntake.gripOpen();
				robotIntake.spinIntake(-400);
				_autoLoop.startPeriodic(0.005);
				autoStep++;
			}
		  break;
	   	case 7:
	   		if(drivePathDone) {
				  _autoLoop.stop();
		   		robotIntake._intakeLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		    	robotIntake._intakeRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	   			robotIntake.gripFirm();
	   			autoStep++;
	   			wristUpDelay=0;
	   		}
	   		break;
	   	case 8:

	   		wristUpDelay++;
	   		if(wristUpDelay > 20) {
	   			robotIntake.spinIntake(0);
	   			robotIntake.wristUp();
	    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	startTime = Timer.getFPGATimestamp();
	         lastPoint = 0;
	        targetTime = 0;
	        motionProfiler.motionProfile = new ArrayList<double[]>();
	 		drivePathDone = false;

		   left = new ArrayList<Point2D>();
    	   left.add(new Point2D.Double(0, 0));
			
			  if(gameData.charAt(0) == 'L')
			  {
				  left.add(new Point2D.Double(0, 64));
				  motionProfiler.bezierPoints(left, 0, 0, 9, 1);
			  } else {
				  left.add(new Point2D.Double(0, 60));
				  motionProfiler.bezierPoints(left, 0, 0, 9, 2);
			  }
		  
		  driveBackwards = true;
		  _autoLoop.startPeriodic(0.005);
		  autoStep++;
	   		}
		  break;
	   	case 9:
	   		if(drivePathDone) {
	   			_autoLoop.stop();
		    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		    	startTime = Timer.getFPGATimestamp();
		         lastPoint = 0;
		        targetTime = 0;
		        motionProfiler.motionProfile = new ArrayList<double[]>();
		 		drivePathDone = false;

	        	   ArrayList<Point2D> left = new ArrayList<Point2D>();
	        	   left.add(new Point2D.Double(0, 0));
					
				  if(gameData.charAt(1) == 'L')
				  {
					  left.add(new Point2D.Double(-105, 85));
					  motionProfiler.bezierPoints(left, 0, -45, 9, 2);
				  } else {
					  left.add(new Point2D.Double(95, 85));
					  motionProfiler.bezierPoints(left, 0, 45, 9, 2);
				  }
				  driveBackwards = false;
				  _autoLoop.startPeriodic(0.005);
				  autoStep++;
	   		}
	           break;
		   	case 10:
		   		if(drivePathDone) {
		   			_autoLoop.stop();
		   			robotIntake.spinIntake(0);
		   			autoStep++;
		   			turnToFirstRun = true;
		   		}
		   		break;
		   	/*case 11:
					if(turnToAngle(180)) {
			   			autoStep++;
			   		}
					break;
			case 12:
		   		if(drivePathDone) {
		   			_autoLoop.stop();
			    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
			    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
			    	startTime = Timer.getFPGATimestamp();
			         lastPoint = 0;
			        targetTime = 0;
			        motionProfiler.motionProfile = new ArrayList<double[]>();
			 		drivePathDone = false;

		        	   ArrayList<Point2D> left = new ArrayList<Point2D>();
		        	   left.add(new Point2D.Double(0, 0));
						
					  if(gameData.charAt(1) == 'L')
					  {
						  left.add(new Point2D.Double(0, 45));
						  motionProfiler.bezierPoints(left, 0, 0, 9, 2);
					  } else {
						  left.add(new Point2D.Double(0, 45));
						  motionProfiler.bezierPoints(left, 0, 0, 9, 2);
					  }
					  driveBackwards = false;
					  _autoLoop.startPeriodic(0.005);
					  autoStep++;
		   		}
		           break;
			   	case 13:
			   		if(drivePathDone) {
			   			_autoLoop.stop();
			   			autoStep++;
			   		}
			   		break;*/
	   	}
	}
	
	public boolean turnToAngle(double angle){
		if(turnToFirstRun){
			turnToAnglePid.setDesiredPosition(angle);
			turnToFirstRun = false;
			turnAccelFilter.reset();
			k = 0;
		}
		double currentAngle =  Constants.navX.getYaw(); //(navX.getYaw() + 180) % 360;
		double speed = turnToAnglePid.calcSpeed(currentAngle);
		turnAccelFilter.set(limitValue(speed, 1.0));
		if(k++ < 1 / 0.03){
			speed = turnAccelFilter.get();
		}
		speed = limitValue(applyMin(speed, 0.1), 1.0);
		
		
		robotDrive._frontLeftMotor.set(speed);
		robotDrive._frontRightMotor.set(-1*speed);

		
		return turnToAnglePid.isDone(currentAngle, 5);
	}
	
	double applyMin(double value, double min){
		if(value > -min && value < 0){
			return -min;
		}
		else if(value < min && value >0){
			return min;
		}
		else{
			return value;
		}
	}
	
	double limitValue(double value, double limit){
		if(value > limit){
			return limit;
		}
		else if(value < -limit){
			return -limit;
		}
		else{
			return value;
		}
	}
}
