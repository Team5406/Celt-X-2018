package org.cafirst.frc.team5406.auto;

public abstract class AutonomousRoutine implements Cloneable{
	
	protected final int kTimeoutMs = 10;
	
	String name = "";
	
	public abstract void init();
	public abstract void periodic();
	
	@Override
	public Object clone() throws CloneNotSupportedException
	{
		return super.clone();
		
	}
	
	public String getName()
	{
		return name;
	}
}
