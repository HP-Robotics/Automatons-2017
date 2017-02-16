package org.usfirst.frc.team2823.robot;

public class EmptyAutonomous extends Autonomous {
	int waitTime = 5000;
	
	public EmptyAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		System.out.println("Nothing to initialize!");
	}
	
	@Override
	public void periodic() {
		System.out.println("Doing nothing for " + (waitTime / 1000) + " second(s)...");
		
		try {
			Thread.sleep(waitTime);
			
		} catch(Exception e) {
			System.out.println("Fatal error! Who cares!");
		}
		
		System.out.println("Successfully did nothing for " + (waitTime / 1000) + " second(s)!");
	}
}
