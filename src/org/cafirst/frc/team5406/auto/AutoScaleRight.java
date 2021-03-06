package org.cafirst.frc.team5406.auto;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;
import org.cafirst.frc.team5406.subsystems.Intake;
import org.cafirst.frc.team5406.auto.MotionProfile;
import org.cafirst.frc.team5406.auto.AutoSwitchFront.AutoRunnable;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.cafirst.frc.team5406.auto.AutonomousRoutine;



public class AutoScaleRight  extends AutonomousRoutine{
	private Intake robotIntake;
	private Drive robotDrive;
	private int autoStep = 0;
	private boolean gearDelay = false;
	private double[] robotPosition;
	private int direction = 1;
    int flipWrist = 0;
    double pathComplete = 0;

    String gameData;
	ArrayList<Point2D> left = new ArrayList<Point2D>();

	boolean driveBackwards = false;
	MotionProfile motionProfiler = new MotionProfile();
	int wristUpDelay = 0;
	boolean drivePathDone = false;
        double startTime = 0;
        int lastPoint = 0;
        double targetTime = 0;
        int autoDelay =0;
        class PeriodicRunnable implements java.lang.Runnable {
    	    public void run() { 

    	    	int armPos;
    	    	
    	    	switch (flipWrist) {
    	    	case 1:
    	    		robotIntake._wristMotor.selectProfileSlot(0,0);
    	    		armPos = robotIntake.getArmPosition();
    	    		robotIntake._wristMotor.set(ControlMode.MotionMagic, robotIntake.wristFlipPosDown(armPos));
    	    		if(armPos < 100) {
    	    			flipWrist = 5;
    	    		}
    	    		break;
    	    	case 3:
    	    		robotIntake._wristMotor.selectProfileSlot(0,0);
    	    		armPos = robotIntake.getArmPosition();
    	    		robotIntake._wristMotor.set(ControlMode.MotionMagic, robotIntake.wristFlipPosUp(armPos));
    	    		if(armPos > Constants.ARM_UP - 100) {
    	    			flipWrist = 5;
    	    		}
    	    		break;
    	    		
    	    	}

    	    }
    	}
    	Notifier _notifier = new Notifier(new PeriodicRunnable());
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
				pathComplete = (double)lastPoint/numPoints;
				System.out.println(lastPoint + "/" + numPoints);
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
						_autoLoop.stop();

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

	public AutoScaleRight(Drive _robotDrive, Intake _robotIntake){
		super("3 - Scale Auto Right");
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
         autoDelay =0;
	    	robotIntake._armMotor.configMotionAcceleration(6000, Constants.kTimeoutMs);
	    	robotIntake._wristMotor.configMotionAcceleration(6000, Constants.kTimeoutMs);
	    	robotIntake._elevatorMotor.configMotionAcceleration(75000, Constants.kTimeoutMs);
			robotIntake.compressor.stop();

	}
	
	public void end(){
		_autoLoop.stop();
		robotIntake.compressor.start();
    	robotIntake._armMotor.configMotionAcceleration(12000, Constants.kTimeoutMs);
    	robotIntake._wristMotor.configMotionAcceleration(8000, Constants.kTimeoutMs);
    	robotIntake._elevatorMotor.configMotionAcceleration(100000, Constants.kTimeoutMs);

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
	   		autoDelay++;
	   		if(autoDelay > 1) {
	   			autoStep++;
	   		}
	   		break;
	   	case 2:
	   		//drive forward, cube 1
	   	   gameData = DriverStation.getInstance().getGameSpecificMessage();
           if(gameData.length() > 0)
           {
           	startTime = Timer.getFPGATimestamp();
            lastPoint = 0;
            targetTime = 0;
    		drivePathDone = false;
        	   left = new ArrayList<Point2D>();
        	   left.add(new Point2D.Double(0, 0));
        	   autoStep++;
			  if(gameData.charAt(1) == 'R'){
				  left.add(new Point2D.Double(15, 170));
				  left.add(new Point2D.Double(-22, 267));
				  motionProfiler.bezierPoints(left, 0, -15, 10, 2);
			  } else {
				  left.add(new Point2D.Double(5, 190));
				  left.add(new Point2D.Double(-150, 220));
				  left.add(new Point2D.Double(-155, 260));
				  motionProfiler.bezierPoints(left, 0, 25, 9, 2);
			  }
			  
			  
			  driveBackwards = true;
			  _autoLoop.startPeriodic(0.005);
           }
           break;
	   	case 3:
	   	//arm position, cube 1
 		System.out.println(pathComplete);
	   	if(pathComplete > 0.60) {

			//robotIntake.wristUp();
			robotIntake.armUp();
			robotIntake.elevatorFast();
			robotIntake.elevatorSwitchMid();
			flipWrist =3;
			robotIntake.wristOut = false;
			_notifier.startPeriodic(0.005);
			robotIntake.needsWristUp = false;	
			autoStep++;
	   		}
	   		break;
	   	case 4:
	   		if(robotIntake.getArmPosition() > Constants.ARM_UP - 300) {
	   			_notifier.stop();
	   			robotIntake._wristMotor.set(ControlMode.MotionMagic, -2400);
	   			autoStep++;
	   		}
	   		break;
	   	case 5:
	   	//shoot, cube 1
	   		if(drivePathDone) {
	   			_autoLoop.stop();
	   			robotIntake._intakeLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	   		
	    	robotIntake._intakeRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	if(gameData.charAt(1) == 'R'){
	   			robotIntake.spinIntake(210); //250
	   		}else {
	   			robotIntake.spinIntake(220);
	   		}
	   		autoStep++;
	   		wristUpDelay=0;
	   		}
	   		break;
		 case 6:
			//arm down, cube 2
	   			robotIntake.gripOpen();
	   			robotIntake.armDown();
				robotIntake.wristOut = true;
				flipWrist =1;
				_notifier.startPeriodic(0.005);
				autoStep++;
		   		break;
	   	case 7:
	   	//drive forward, cube 2
	   		if(robotIntake._intakeLeftMotor.getSelectedSensorPosition(0) + robotIntake._intakeRightMotor.getSelectedSensorPosition(0)> 8000) {
	   			robotIntake.spinIntake(0);

	    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	startTime = Timer.getFPGATimestamp();
	         lastPoint = 0;
	        targetTime = 0;
	        motionProfiler.motionProfile = new ArrayList<double[]>();
	 		drivePathDone = false;

		   left = new ArrayList<Point2D>();
    	   left.add(new Point2D.Double(0, 0));
			
		  if(gameData.charAt(1) == 'R'){
			  left.add(new Point2D.Double(9, 57));
			  motionProfiler.bezierPoints(left, 0, 0, 8, 2);
		  } else {
			  left.add(new Point2D.Double(-15, 57));
			  motionProfiler.bezierPoints(left, 0, -10, 8, 2);
		  }
		  
		  driveBackwards = false;
		  _autoLoop.startPeriodic(0.005);
		  autoStep++;
		  
	   		}
		  break;

	   	case 8:
			   wristUpDelay++;
			   if(wristUpDelay > 10) {
					robotIntake.elevatorFast();
					robotIntake.elevatorDown();
					autoStep++;
		   		}
	   break;
	   case 9:
	   		if(robotIntake.getArmPosition() < 200) {
	   		_notifier.stop();
	   		robotIntake.wristDownMore();
	   		robotIntake.spinIntake(-200);
			autoStep++;
	   		}
	   break;
	   case 10:
	   		if(drivePathDone) {
				  _autoLoop.stop();
		   		robotIntake._intakeLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		    	robotIntake._intakeRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	   			robotIntake.gripFirm();
	   			autoStep++;
	   			wristUpDelay=0;
	   		}
	   		break;
	   case 11:
		   wristUpDelay++;
		   if(wristUpDelay > 20) {
	   			robotIntake.spinIntake(0);

	    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	startTime = Timer.getFPGATimestamp();
	         lastPoint = 0;
	        targetTime = 0;
	        motionProfiler.motionProfile = new ArrayList<double[]>();
	 		drivePathDone = false;

		   left = new ArrayList<Point2D>();
  	   left.add(new Point2D.Double(0, 0));
			
		  if(gameData.charAt(1) == 'R'){
			  left.add(new Point2D.Double(5, 47));
			  motionProfiler.bezierPoints(left, 0, 0, 8, 1);
		  } else {
			  left.add(new Point2D.Double(-1, 65));
			  motionProfiler.bezierPoints(left, -10, 30, 8, 2);
		  }
		  
		  driveBackwards = true;
		  _autoLoop.startPeriodic(0.005);
		  autoStep++;
		   }
		  break;
	   case 12:
	   			robotIntake.armUp();
				robotIntake.elevatorFast();
				robotIntake.elevatorSwitchMid();
				flipWrist =3;
				robotIntake.wristOut = false;
				_notifier.startPeriodic(0.005);
				robotIntake.needsWristUp = false;	
				autoStep++;
				break;

	   case 13:
	   		if(drivePathDone && robotIntake.getArmPosition() > Constants.ARM_UP - 300) {
  			robotIntake.gripOpen();
	   		_notifier.stop();
	   		_autoLoop.stop();
	   		if(gameData.charAt(1) == 'R'){
	   			robotIntake.spinIntake(90);
	   		}else {
	   			robotIntake.spinIntake(40);
	   		}
			autoStep++;
	   		}
	   break;

	   case 14:
		   if(gameData.charAt(1) == 'R') {
	    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	startTime = Timer.getFPGATimestamp();
	         lastPoint = 0;
	        targetTime = 0;
	        motionProfiler.motionProfile = new ArrayList<double[]>();
	 		drivePathDone = false;

		   left = new ArrayList<Point2D>();
   	   left.add(new Point2D.Double(0, 0));

			  left.add(new Point2D.Double(0, 15));
			  left.add(new Point2D.Double(65, 52));
			  motionProfiler.bezierPoints(left, 0, 30, 7, 2);
		  
		  driveBackwards = false;
		  _autoLoop.startPeriodic(0.005);
		  autoStep++;
		  wristUpDelay=0;
		   }else {
			   autoStep=21;
		   }
		  break;
	   case 15:
		   wristUpDelay++;
		   if(wristUpDelay > 15) {
	   		//if(robotIntake._intakeLeftMotor.getSelectedSensorPosition(0) + robotIntake._intakeRightMotor.getSelectedSensorPosition(0)> 20) {
	   			robotIntake.spinIntake(0);
	   			robotIntake.armDown();
				robotIntake.elevatorFast();
				robotIntake.elevatorDown();
				robotIntake.wristOut = true;
				flipWrist =1;
				_notifier.startPeriodic(0.005);
				autoStep++;
				wristUpDelay=0;
	   		}
	   		break;
	   case 16:
	   		if(robotIntake.getArmPosition() < 200) {
	   		_notifier.stop();
	   		robotIntake.wristDownMore();
	   		robotIntake.spinIntake(-1000);
	   		robotIntake.gripOpen();
			autoStep++;
	   		}
	   break;
	   case 17:
	   		if(drivePathDone) {
				_autoLoop.stop();
		   		robotIntake._intakeLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		    	robotIntake._intakeRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	   			robotIntake.gripFirm();
	   			autoStep++;
	   			wristUpDelay=0;
	   		}
	   		break;
	   case 18:
		   wristUpDelay++;
		   if(wristUpDelay > 20) {
	    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	startTime = Timer.getFPGATimestamp();
	         lastPoint = 0;
	        targetTime = 0;
	        motionProfiler.motionProfile = new ArrayList<double[]>();
	 		drivePathDone = false;

		   left = new ArrayList<Point2D>();
  	   left.add(new Point2D.Double(0, 0));
			
		  if(gameData.charAt(1) == 'R'){
			  left.add(new Point2D.Double(24, 40));
			  motionProfiler.bezierPoints(left, 0, -45, 9, 1);
		  } else {
			  left.add(new Point2D.Double(-25, 63));
			  motionProfiler.bezierPoints(left, 0, 15, 10, 2);
		  }
		  
		  driveBackwards = true;
		  _autoLoop.startPeriodic(0.005);
		  autoStep++;
		   }
		  break;
	   case 19:
	   			robotIntake.spinIntake(0);
	   			robotIntake.armUp();
				robotIntake.elevatorFast();
				robotIntake.elevatorSwitchMid();
				flipWrist =3;
				robotIntake.wristOut = false;
				_notifier.startPeriodic(0.005);
				robotIntake.needsWristUp = false;	
				autoStep++;
				break;

	   case 20:
	   		if(drivePathDone && robotIntake.getArmPosition() > Constants.ARM_UP - 300) {
	   		_notifier.stop();
	   		_autoLoop.stop();
  			robotIntake.gripOpen();
	   		robotIntake.spinIntake(25);
			autoStep++;
	   		}
	   break;
	   case 21:
		   
	    	robotDrive._frontLeftMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	robotDrive._frontRightMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	    	startTime = Timer.getFPGATimestamp();
	         lastPoint = 0;
	        targetTime = 0;
	        motionProfiler.motionProfile = new ArrayList<double[]>();
	 		drivePathDone = false;

		   left = new ArrayList<Point2D>();
  	   left.add(new Point2D.Double(0, 0));
			
		  if(gameData.charAt(1) == 'R'){
			  left.add(new Point2D.Double(0, 10));
			  motionProfiler.bezierPoints(left, 0, 0, 10, 1);
		  } else {
			  left.add(new Point2D.Double(0, 10));
			  motionProfiler.bezierPoints(left, 0, 0, 10, 2);
		  }
		  
		  driveBackwards = false;
		  _autoLoop.startPeriodic(0.005);
		  autoStep++;
		  wristUpDelay=0;
		  
		  break;
	   case 22:
		  wristUpDelay++;
		  if(wristUpDelay > 15) {
	   		//if(robotIntake._intakeLeftMotor.getSelectedSensorPosition(0) + robotIntake._intakeRightMotor.getSelectedSensorPosition(0)> 20) {
	   			robotIntake.spinIntake(0);
	   			robotIntake.armDown();
	  			robotIntake.gripFirm();
				robotIntake.elevatorFast();
				robotIntake.elevatorDown();
				robotIntake.wristOut = false;
				flipWrist =1;
				_notifier.startPeriodic(0.005);
				autoStep++;
	   		}
	   		break;
	   	}
		
	}
}
