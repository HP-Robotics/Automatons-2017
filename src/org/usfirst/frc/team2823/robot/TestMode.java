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
    	
    	robot.log.open("kA_kV.csv", "Time:,X:,Y:,Vx:,Vy:,Ax:,Ay:\n");
	}
	
	public void testPeriodic(){
		robot.robotButton.update(robot.driverStick.getRawButton(2));
		
		//robot.xControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		//robot.yControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		System.out.println("x: " + robot.encoderThread.getX() + " y: " + robot.encoderThread.getY() + " r: " + robot.encoderThread.getR());
		
		if(robot.robotButton.changed()) {
			if(robot.robotButton.on()) {
				System.out.println("on");
				
				//robot.driveTo_Cartesian(2, 2);
				//robot.xControl.configureGoal(2, robot.MAX_SIDE_VEL, robot.MAX_SIDE_ACCEL);
				robot.yControl.configureGoal(2, robot.MAX_FORWARD_VEL, robot.MAX_FORWARD_ACCEL);
				
				//robot.xControl.enable();
				robot.yControl.enable();
				
				robot.rControl.setSetpoint(0);
				robot.rControl.enable();
				
				//robot.driveTo(2, 0);
				//robot.vControl.enable();
				//robot.vControl.enableLog("testPID.csv");
			} else {
				System.out.println("off");
				
				robot.xControl.disable();
				robot.yControl.disable();
				robot.rControl.disable();
				
				robot.xControl.closeLog();
				robot.yControl.closeLog();
				//robot.rControl.closeLog();
				
				//robot.vControl.disable();
				//robot.vControl.closeLog();
			}
		}
		
		robot.log.write(Timer.getFPGATimestamp() + "," + robot.encoderThread.getX() + "," + robot.encoderThread.getY() + "\n");
		
		robot.setDriveT(-robot.gyro.getAngle());
		//robot.robotDrive.mecanumDrive_Cartesian(1, 1, 0, robot.getDriveT());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
}
