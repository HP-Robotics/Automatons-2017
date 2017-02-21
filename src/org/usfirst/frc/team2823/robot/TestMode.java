package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	Robot robot;
	
	public TestMode (Robot robot){
		this.robot = robot;
	}
	
	public void testInit(){
		robot.ahrs.reset();
		
    	if(robot.stick1.getName().contains("3D") || robot.stick1.getName().isEmpty()){
    		robot.driverStick = robot.stick1;
    		robot.operatorStick = robot.stick2;
    	} else {
    		robot.driverStick = robot.stick2;
    		robot.operatorStick = robot.stick1;
    	}
    	
    	//robot.aEncoder.reset();
    	robot.log.open("kA_kV.csv", "Time:,X:,Y:,Vx:,Vy:,Ax:,Ay:\n");
	}
	
	public void testPeriodic(){
		robot.robotButton.update(robot.driverStick.getRawButton(2));
		
		if(robot.robotButton.on()){
			//System.out.println("l: " + robot.lEncoder.getDistance() + " r: " + robot.rEncoder.getDistance() + " a: " + robot.aEncoder.getDistance());
			System.out.print(robot.xControl.getError());
			System.out.println(" " + robot.yControl.getError());
		}
		
		//System.out.println(robot.rControl.getError());
		
		//robot.xControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		//robot.yControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		//robot.rControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		
		//System.out.println("x: " + robot.encoderThread.getX() + " y: " + robot.encoderThread.getY() + " r: " + robot.encoderThread.getR());
		//robot.log.write(Timer.getFPGATimestamp() + "," + 0.0 + "," + robot.aEncoder.getDistance() + "\n");
		
		if(robot.robotButton.changed()) {
			if(robot.robotButton.on()) {
				System.out.println("on");
				
				robot.driveTo_Cartesian(35, 70);
				robot.rotateTo(-60);
				
				//robot.xControl.configureGoal(2, robot.MAX_SIDE_VEL, robot.MAX_SIDE_ACCEL);
				
				//robot.aEncoder.reset();
				//robot.yControl.reset();
				//robot.yControl.setKaKv(robot.FORWARD_KA * SmartDashboard.getNumber("KaMult", 2.0), robot.FORWARD_KV * SmartDashboard.getNumber("KvMult", 1.0));
				//robot.xSource.reset();
				//robot.xControl.configureGoal(SmartDashboard.getNumber("Setpoint", 48), robot.MAX_FORWARD_VEL, robot.MAX_FORWARD_ACCEL * 0.8);
				//robot.xControl.enable();
				//robot.yControl.setSetpoint(robot.encoderThread.getY() + 24);
				
				//robot.yControl.setSetpoint(24);
				//robot.xControl.enableLog("xControl.csv");
				//robot.yControl.enable();
				
				//robot.xControl.enable();
				//robot.yControl.enable();
				
				//robot.rControl.reset();
				//robot.ahrs.reset();
				//robot.rControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0.0));
				//robot.rControl.enable();

				//System.out.println(robot.ahrs.getAngle());
				//robot.rotateTo(robot.ahrs.getAngle() + 90.0);
				
				//robot.driveTo(2, 0);
				//robot.vControl.enable();
				//robot.vControl.enableLog("testPID.csv");
			} else {
				System.out.println("off");
				
				robot.xControl.reset();
				robot.yControl.reset();
				robot.rControl.reset();
				
				robot.xControl.closeLog();
				robot.yControl.closeLog();
				robot.rControl.closeLog();
				
				//robot.vControl.disable();
				//robot.vControl.closeLog();
			}
		}
		
		if(robot.xControl.isPlanFinished()) {
			robot.xControl.reset();
			robot.xControl.closeLog();
		}
		
		if(robot.yControl.isPlanFinished()) {
			robot.yControl.reset();
			robot.yControl.closeLog();
		}
		
		robot.setDriveT(robot.ahrs.getAngle());
		//robot.robotDrive.mecanumDrive_Cartesian(0, -1, 0, robot.getDriveT());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
}
