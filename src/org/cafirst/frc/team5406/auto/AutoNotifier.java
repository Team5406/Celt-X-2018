package org.cafirst.frc.team5406.auto;

import java.util.ArrayList;

import org.cafirst.frc.team5406.robot.Constants;
import org.cafirst.frc.team5406.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

public class AutoNotifier extends Notifier{
	
	AutoRunnable runner;
	
	private AutoNotifier(Runnable run) {
		super(run);
		runner = (AutoRunnable) run;
	}
	
	public boolean IsDrivePathDone() { return runner.IsDrivePathDone(); }
	
	public static AutoNotifier CreateAutoNotifier(Drive _robotDrive, ArrayList<double[]> drivePath, boolean _isBackwards)
	{
		AutoRunnable _runner = new AutoRunnable(_robotDrive, drivePath, _isBackwards);
		
		return new AutoNotifier(_runner);
	}

	

}
