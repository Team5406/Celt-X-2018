package org.cafirst.frc.team5406.auto;


public abstract class AutonomousRoutine {

	private String name = "Default";
	private boolean running;
	

	public AutonomousRoutine(String name){
		this.name = name;
		running = false;
	}
	
	public boolean isRunning(){
		return running;
	}
	
	public String getName(){
		return name;
	}
	
	public void start(){
		running = true;
		init();
	}
	
	public void stop(){
		running = false;
		end();
	}
	
	public void run(){
		periodic();
	}
	
	public abstract void init();
	public abstract void periodic();
	public abstract void end();
	
}
