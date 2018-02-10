/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.cafirst.frc.team5406.robot;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.cafirst.frc.team5406.util.XboxController;;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(1);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(5);

	

	/* extra talons for six motor drives */
	WPI_VictorSPX _leftSlave1 = new WPI_VictorSPX(2);
	WPI_VictorSPX _leftSlave2 = new WPI_VictorSPX(3);
	WPI_VictorSPX _leftSlave3 = new WPI_VictorSPX(4);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(6);
	WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(7);
	WPI_VictorSPX _rightSlave3 = new WPI_VictorSPX(8);
	
	double speed = 0;
	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);
   // DoubleSolenoid shiftSolenoid;
    

	int autoLoop = 0;
	int step =0;
	
    private XboxController driverGamepad;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	//shiftSolenoid = new DoubleSolenoid(0,1);

    	driverGamepad = new XboxController(0);

    	
    	/* take our extra talons and just have them follow the Talons updated in arcadeDrive */
    	_leftSlave1.follow(_frontLeftMotor);
    	_leftSlave2.follow(_frontLeftMotor);
    	_leftSlave3.follow(_frontLeftMotor);
    	_rightSlave1.follow(_frontRightMotor);
    	_rightSlave2.follow(_frontRightMotor);
    	_rightSlave3.follow(_frontRightMotor);
    	
    	autoLoop = 0;
    	/*_leftSlave1.setInverted(true);
    	_leftSlave2.setInverted(true);
    	_rightSlave1.setInverted(true);
    	_rightSlave2.setInverted(true);*/
    	_frontRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 50);
		_frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
		_frontLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 50);
		_frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
		_frontLeftMotor.configContinuousCurrentLimit(30, 0);
		_frontLeftMotor.configPeakCurrentLimit(30, 0);
		_frontLeftMotor.configPeakCurrentDuration(30, 0);
		_frontLeftMotor.enableCurrentLimit(true);
		_frontRightMotor.configContinuousCurrentLimit(30, 0);
		_frontRightMotor.configPeakCurrentLimit(30, 0);
		_frontRightMotor.configPeakCurrentDuration(30, 0);
		_frontRightMotor.enableCurrentLimit(true);

		/*_frontLeftMotor.configOpenloopRamp(0.05, 0);
		_leftSlave1.configOpenloopRamp(0, 0); // no need since master ramps 
		_leftSlave2.configOpenloopRamp(0, 0); // no need since master ramps 
		_leftSlave3.configOpenloopRamp(0, 0); // no need since master ramps 
		_frontRightMotor.configOpenloopRamp(0.05, 0);
		_rightSlave1.configOpenloopRamp(0, 0); // no need since master ramps
		_rightSlave2.configOpenloopRamp(0, 0); // no need since master ramps
		_rightSlave3.configOpenloopRamp(0, 0); // no need since master ramps*/
		
    }

    /**
     * This function is called periodically during operator control
     */
    
    public void teleopPeriodic() {
        _drive.arcadeDrive(driverGamepad.getLeftY(), driverGamepad.getLeftX());
    }
    
    public void autonomousInit() {
    	autoLoop=0;
    	//shiftSolenoid.set(DoubleSolenoid.Value.kReverse);
    	_frontLeftMotor.setSelectedSensorPosition(0, 0, 10);
    	_frontRightMotor.setSelectedSensorPosition(0, 0, 10);
    	speed =0;
    	step =0;
    }
    
    public void autonomousPeriodic() {
    	/*
    	double positionLeft = _frontLeftMotor.getSelectedSensorPosition(0);
    	double positionRight = _frontRightMotor.getSelectedSensorPosition(0);
    	
    	switch(step) {
    	case 0:
    		speed+=0.015;
    		if (positionRight>2000) {
    			step =1;
    		}
    		break;
    	case 1:
    		speed = 0.7;
    		if(positionRight > 28000) {
    			step =2;
    		}
    		break;
    	case 2:
    		speed-=0.015;
    		if(positionRight > 33800 || speed <=0.1) {
    			step =3;
    		}
    		break;
    	case 3:
    		
    		if(positionRight > 34200) {
    			speed = -0.1;
    		}else {
    			speed=0;
    		}
    		break;    	 	
    	}
    	System.out.println(positionRight);*/
    	//_drive.tankDrive(0.5,0.5);
    	
    }
}