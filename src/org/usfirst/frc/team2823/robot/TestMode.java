package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	Robot robot;
	
	public TestMode (Robot robot){
		this.robot = robot;
	}
	
	public void testInit(){
    	if(robot.stick1.getName().contains("3D")){
    		robot.driverStick = robot.stick1;
    		robot.operatorStick = robot.stick2;
    	} else {
    		robot.driverStick = robot.stick2;
    		robot.operatorStick = robot.stick1;
    	}
	}
	
	public void testPeriodic(){
		
	}
}
