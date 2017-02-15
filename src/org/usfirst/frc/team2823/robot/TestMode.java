package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	Robot robot;
	
	public TestMode (Robot robot){
		this.robot = robot;
	}
	
	public void testInit(){
    	if(robot.stick1.getName().contains("3D") || robot.stick1.getName().isEmpty()){
    		robot.driverStick = robot.stick1;
    		robot.operatorStick = robot.stick2;
    	} else {
    		robot.driverStick = robot.stick2;
    		robot.operatorStick = robot.stick1;
    	}
    	
    	//robot.log.open("kA_kV.csv", "Time:,Y:,Vy:,Ay:\n");
	}
	
	public void testPeriodic(){
		robot.robotButton.update(robot.driverStick.getRawButton(2));
		
		robot.xControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		robot.yControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		
		if(robot.robotButton.changed()) {
			if(robot.robotButton.on()) {
				System.out.println("on");
				robot.driveTo(2, 0);
				robot.xControl.enable();
				robot.xControl.enableLog("Xtestpid.csv");
				
				robot.yControl.enable();
				robot.yControl.enableLog("Ytestpid.csv");
				
				//robot.vControl.enable();
				//robot.vControl.enableLog("testPID.csv");
			} else {
				robot.xControl.disable();
				robot.xControl.closeLog();
				
				robot.yControl.disable();
				robot.yControl.closeLog();
				//robot.vControl.disable();
				//robot.vControl.closeLog();
			}
		}
		
		robot.setDriveT(-robot.gyro.getAngle());
		//robot.robotDrive.mecanumDrive_Cartesian(0.1, 0, 0, robot.getDriveT());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
}
