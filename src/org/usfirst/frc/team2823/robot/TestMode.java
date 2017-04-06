package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	Robot robot;
	
	public TestMode (Robot robot){
		this.robot = robot;
	}
	
	public void testInit(){
		robot.navx.reset();
		//robot.gyro.reset();
		
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
		robot.shootTrigger.update(robot.driverStick.getRawButton(1));
		robot.robotButton.update(robot.driverStick.getRawButton(2));
		robot.intakeButton.update(robot.driverStick.getRawButton(3));
		robot.gearButton.update(robot.driverStick.getRawButton(4));
		robot.fieldButton.update(robot.driverStick.getRawButton(5));
		robot.gearKickButton.update(robot.operatorStick.getRawButton(1));
		
		/*if(robot.gearButton.changed()) {
			robot.rControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		}
		
		if(robot.intakeButton.changed()) {
			robot.xControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		}
		
		if(robot.fieldButton.changed()) {
			robot.yControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		}
		
		
		if(robot.robotButton.on()) {
			System.out.println("r: " + robot.ahrs.getAngle() + " re: " + robot.rControl.getError());
		}
		
		if(robot.shootTrigger.on()){
			//System.out.println("l: " + robot.encoderThread.getLDistance() + " r: " + robot.encoderThread.getRDistance() + " c: " + robot.encoderThread.getCDistance());
			System.out.println("x: " + robot.encoderThread.getX() + " xe: " + robot.xControl.getError() + " y: " + robot.encoderThread.getY() + " ye: " + robot.yControl.getError());
		}*/
		
		//System.out.println("a: " + robot.ahrs.getAngle() + " c: " + robot.getCousin(robot.ahrs.getAngle(), SmartDashboard.getNumber("Setpoint", 0.0)));
		
		//System.out.println(robot.rControl.getError());
		
		//robot.xControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		//robot.yControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		//robot.rMotionControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		
		//robot.xControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		
		System.out.println("l: " + robot.encoderThread.getLDistance() + " r: " + robot.encoderThread.getRDistance() + " c: " + robot.encoderThread.getCDistance());
		System.out.println("x: " + robot.encoderThread.getX() + " y: " + robot.encoderThread.getY() + " r: " + robot.encoderThread.getR());
		//System.out.println("g: " + robot.gyro.getAngle() + " n: " + robot.navx.getAngle() + " e: " + robot.encoderThread.getR());
		//robot.log.write(Timer.getFPGATimestamp() + "," + 0.0 + "," + robot.aEncoder.getDistance() + "\n");
		
		//robot.gearControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
		
		/*if(robot.gearKickButton.held()){
			if(robot.gearKickButton.changed()){
				robot.gearControl.enableLog("gearPID.csv");
				robot.gearControl.enable();
				robot.gearControl.setSetpoint(robot.GEAR_KICK_OUT * robot.DEG_TO_G_ENC);
			}
		}else{
			robot.gearControl.enable();
			robot.gearControl.setSetpoint(robot.GEAR_KICK_IN * robot.DEG_TO_G_ENC);
		}*/
		
		if(robot.robotButton.changed()) {
			if(robot.robotButton.on()) {
				robot.rotateTo_Relative(SmartDashboard.getNumber("RSetpoint", 0.0));
				//robot.rotateTo(robot.navx.getAngle() + SmartDashboard.getNumber("RSetpoint", 0.0));
			} else {
				robot.rMotionControl.closeLog();
				robot.rMotionControl.reset();
				//robot.rControl.closeLog();
				//robot.rControl.reset();
			}
		}
		
		if(robot.shootTrigger.changed()) {
			if(robot.shootTrigger.on()) {
				robot.driveTo_Cartesian(SmartDashboard.getNumber("XSetpoint", 0), SmartDashboard.getNumber("YSetpoint", 0));/*, SmartDashboard.getNumber("KvMult", 1), SmartDashboard.getNumber("KaMult", 1));*/
				robot.rotateTo(0);
			} else {
				robot.xControl.closeLog();
				robot.yControl.closeLog();
				robot.rControl.closeLog();
				
				robot.xControl.reset();
				robot.yControl.reset();
				robot.rControl.reset();
			}
		}
		
		/*if(robot.robotButton.changed()) {
			if(robot.robotButton.on()) {
				System.out.println("on");
				
				//robot.driveTo_Cartesian(35, 70);
				//robot.rotateTo(-60);
				
				//robot.xControl.configureGoal(2, robot.MAX_SIDE_VEL, robot.MAX_SIDE_ACCEL);
				
				//robot.driveTo_Cartesian(SmartDashboard.getNumber("Setpoint", 48), 0);
				robot.rotateTo(SmartDashboard.getNumber("Setpoint", 180));
				
				//robot.aEncoder.reset();
				//robot.ySource.reset();
				//robot.yControl.reset();
				//robot.yControl.setKaKv(SmartDashboard.getNumber("Ka", robot.FORWARD_KA) * SmartDashboard.getNumber("KaMult", 2.0), SmartDashboard.getNumber("Kv", robot.FORWARD_KV) * SmartDashboard.getNumber("KvMult", 1.0));
				//robot.yControl.configureGoal(SmartDashboard.getNumber("Setpoint", 48), SmartDashboard.getNumber("Max a", robot.MAX_FORWARD_VEL), SmartDashboard.getNumber("Max v", robot.MAX_FORWARD_ACCEL * 0.8));
				//robot.yControl.enableLog("yControlTest.csv");
				//robot.yControl.enable();
				
				//robot.xSource.reset();
				//robot.xControl.reset();
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
				//robot.rControl.setSetpoint(0);
				//robot.rControl.enableLog("rControl.csv");
				//robot.rControl.enable();

				//System.out.println(robot.ahrs.getAngle());
				//robot.rotateTo(robot.ahrs.getAngle() + 90.0);
				
				//robot.driveTo(2, 0);
				//robot.vControl.enable();
				//robot.vControl.enableLog("testPID.csv");
			} else {
				System.out.println("off");
				//robot.xControl.reset();
				//robot.yControl.reset();
				robot.rControl.reset();
				
				//robot.xControl.closeLog();
				//robot.yControl.closeLog();
				robot.rControl.closeLog();
				
				//robot.vControl.disable();
				//robot.vControl.closeLog();
			}
		}*/
		
		/**if(robot.xControl.isPlanFinished()) {
			robot.xControl.reset();
			robot.xControl.closeLog();
		}
		
		if(robot.yControl.isPlanFinished()) {
			robot.yControl.reset();
			robot.yControl.closeLog();
		}**/
		
		/*if(Math.abs(robot.rControl.getError()) < 3 && robot.rControl.isEnabled()) {
			robot.rControl.reset();
			robot.rControl.closeLog();
		}*/
		
		robot.leftServo.set((robot.operatorStick.getRawAxis(1) + 1) / 2);
		robot.rightServo.set((robot.operatorStick.getRawAxis(3) + 1) / 2);
		
		//System.out.println(((robot.operatorStick.getRawAxis(1) + 1) / 2) + " " + ((robot.operatorStick.getRawAxis(3) + 1) / 2));
		
		//System.out.println("Gyro: " + robot.ahrs.getAngle() + " navX: " + robot.navx.getAngle() + " fused: " + robot.navx.getCompassHeading());
		
		robot.setDriveT(robot.navx.getAngle());
		//robot.robotDrive.mecanumDrive_Cartesian(0, -1, 0, robot.getDriveT());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
}
