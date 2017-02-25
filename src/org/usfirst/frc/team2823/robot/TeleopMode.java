package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopMode {
	Robot robot;
	
	//DriveMode defaultMode = DriveMode.INTAKE;
	DriveMode defaultMode = DriveMode.FIELD; //for testing purposes, the final default will likely be INTAKE mode
	DriveMode mode = defaultMode;
	DriveMode prevMode = defaultMode;
	
	double angle;
	
	double prevTime = Timer.getFPGATimestamp();
	
	private enum DriveMode {
		ROBOT, FIELD, INTAKE, GEAR
	}
	
	public TeleopMode(Robot robot) {
		this.robot = robot;
		
		angle = robot.MIDDLE_LIFT_ANGLE;
	}
	
	public void teleopInit() {
    	if(robot.stick1.getName().contains("3D") || robot.stick1.getName().isEmpty()){
    		robot.driverStick = robot.stick1;
    		robot.operatorStick = robot.stick2;
    	}else {
    		robot.driverStick = robot.stick2;
    		robot.operatorStick = robot.stick1;
    	}
    	
    	robot.log.open("gyro_comparison.csv", "ADXR,navX,encR\n");
	}
	
	public void teleopPeriodic() {
		//update buttons
		robot.robotButton.update(robot.driverStick.getRawButton(2));
		robot.fieldButton.update(robot.driverStick.getRawButton(5));
		robot.gearButton.update(robot.driverStick.getRawButton(6));
		robot.intakeButton.update(robot.driverStick.getRawButton(3));
		robot.gyroResetButton1.update(robot.driverStick.getRawButton(11));
		robot.gyroResetButton2.update(robot.driverStick.getRawButton(12));
		
		try{
			robot.shootTrigger.update(robot.driverStick.getRawButton(1) || robot.operatorStick.getRawButton(2));
			robot.shooterWheelsButton.update(robot.operatorStick.getRawButton(1));
			robot.climbButton.update(robot.operatorStick.getRawButton(4));
			robot.intakeState.update(robot.operatorStick.getRawButton(7));
			
			
		}catch (Exception e){
			
		}
		//prevent joysticks from driving robot when within a threshold value of zero
		double x = Math.abs(robot.driverStick.getX()) < robot.STICKTHRESHOLD ? 0.0 : Math.pow(robot.driverStick.getX(), 3);
		double y = Math.abs(robot.driverStick.getY()) < robot.STICKTHRESHOLD ? 0.0 : Math.pow(robot.driverStick.getY(), 3);
		double r = Math.abs(robot.driverStick.getZ()) < robot.ROTATIONTHRESHOLD ? 0.0 : 0.75 * robot.driverStick.getZ();
		
		//double opx = Math.abs(robot.opponentStick.getX()) < robot.kStickThreshold ? 0.0 : robot.opponentStick.getX();
		//double opy = Math.abs(robot.opponentStick.getY()) < robot.kStickThreshold ? 0.0 : robot.opponentStick.getY();
		//double opr = Math.abs(robot.opponentStick.getZ()) < robot.kStickThreshold ? 0.0 : robot.opponentStick.getZ();
		
		//get gyro angle
		double t = robot.ahrs.getAngle();
		//double opt = -(robot.opponentGyro.getAngle() + 90);
		
		//robot.log.write(-robot.gyro.getAngle() + "," + -robot.ahrs.getAngle() + "," + robot.encoderThread.getR() + "\n");
		//System.out.println(-robot.gyro.getAngle() + " " + -robot.ahrs.getAngle() + " " + robot.encoderThread.getR());
		
		//update which drive mode the robot is in
		mode = setDriveMode();
		
		//System.out.println(robot.ahrs.getAngle());
		
		//update whether rotation PID is enabled
		/** CHANGE NAME? GOOD ENOUGH?**/
		setGearAngle();
		setDrivePIDs();
				
		if(Math.abs(Timer.getFPGATimestamp() - prevTime) > 1.0) {
			//System.out.print("x: " + robot.encoderThread.getX() + " y: " + robot.encoderThread.getY() + " r: " + robot.encoderThread.getR());
			//System.out.println(" l: " + robot.encoderThread.getLDistance() + " r: " + robot.encoderThread.getRDistance() + " c: " + robot.encoderThread.getCDistance());
			//System.out.println("l: " + robot.encoderThread.getLDistance() + " r: " + robot.encoderThread.getRDistance() + " c: " + robot.encoderThread.getCDistance());
			System.out.println(robot.ahrs.getAngle());
			prevTime = Timer.getFPGATimestamp();
		}
		
		//determine PID setpoint and drive motor outputs based on drive mode
		setDriveOutputs(x, y, r, t);
		
		SmartDashboard.putNumber("X: ", robot.encoderThread.getX());
        SmartDashboard.putNumber("Y: ", robot.encoderThread.getY());
        SmartDashboard.putNumber("R: ", robot.encoderThread.getR());
        SmartDashboard.putNumber("L Distance: ", robot.encoderThread.getLDistance());
        SmartDashboard.putNumber("R Distance: ", robot.encoderThread.getRDistance());
        SmartDashboard.putNumber("C Distance: ", robot.encoderThread.getCDistance());
        SmartDashboard.putBoolean("Robot Oriented", mode == DriveMode.ROBOT);
        SmartDashboard.putBoolean("Intake Mode", mode == DriveMode.INTAKE);
        SmartDashboard.putBoolean("Field Mode", mode == DriveMode.FIELD);
        SmartDashboard.putBoolean("Gear Mode", mode == DriveMode.GEAR);
        SmartDashboard.putBoolean("Climber", robot.climbButton.on());
        SmartDashboard.putBoolean("Intake", robot.intakeState.on());
		
		/*if(robot.xButton.changed()) {
			if(robot.xButton.on()) {
				robot.xControl.setSetpoint(1000 * -27);
				robot.xControl.enableLog("zanzibar_spumoni.csv");
				robot.xControl.enable();
			} else {
				robot.xControl.disable();
				robot.xControl.closeLog();
			}
		}*/
		
		//drive robot using calculated values
		//robot.robotDrive.mecanumDrive_Cartesian(x, y, r, t);
		
		if(robot.shootTrigger.held()){
			robot.uptake.set(1.0);
			robot.beltFeed.set(-0.5);
		} else{
			robot.uptake.set(0.0);
			robot.beltFeed.set(0.0);
		}
		
		if(robot.shooterWheelsButton.on()){
			robot.topShooter.speedMode();
			//robot.topShooter.set(-SmartDashboard.getNumber("Setpoint", 4300));
			robot.topShooter.set(-robot.CLOSE_SHOT_SPEED);
			robot.bottomShooter.speedMode();
			//robot.bottomShooter.set(SmartDashboard.getNumber("Setpoint", 4300));
			robot.bottomShooter.set(robot.CLOSE_SHOT_SPEED);
		} else{
			robot.topShooter.normalMode();
			robot.topShooter.set(0.0);
			robot.bottomShooter.normalMode();
			robot.bottomShooter.set(0.0);
		}
		
		if(robot.intakeState.on()){
			robot.intake.set(1.0);
		}else{
			robot.intake.set(0.0);
		}
		
		if(robot.climbButton.on()){
			robot.climbMotor1.set(-1.0);//not production values
			robot.climbMotor2.set(-1.0);
		} else {
			robot.climbMotor1.set(0.0);
			robot.climbMotor2.set(0.0);
		}
		
		if(robot.gyroResetButton1.held() && robot.gyroResetButton2.held()) {
			if(!robot.resettingGyro) {
				robot.resettingGyro = true;
				
				robot.ahrs.reset();
			}
		} else {
			robot.resettingGyro = false;
		}
		
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
		//robot.opponentDrive.mecanumDrive_Cartesian(opx, opy, opr, opt);
		//System.out.println(opx + " " + opy + " " + opr + " " + opt);
	}
	
	//select a drive mode based on button input
	public DriveMode setDriveMode() {
		
		//ROBOT mode, is press-hold, has priority over other modes but reverts to previous mode when released
		if(robot.robotButton.held()) {
			if(mode != DriveMode.ROBOT) {
				prevMode = mode;		//if switching to ROBOT mode, store the previous drive mode
			}
			return DriveMode.ROBOT;
			
		} else if(mode == DriveMode.ROBOT) {
			return prevMode;			//if switching out of ROBOT mode, revert to the previous drive mode
		}
		
		//FIELD mode, is toggle, reverts to the default drive mode when released
		if(robot.fieldButton.changed()) {
			if(robot.fieldButton.on()) {
				return DriveMode.FIELD;
			} else {
				return defaultMode;
			}
		}
		
		//INTAKE mode, is toggle, is the default drive mode
		if(robot.intakeButton.changed()) {
			if(robot.intakeButton.on()) {
				return DriveMode.INTAKE;
			} else {
				return defaultMode;		//returns default drive mode for the sake of it, but since INTAKE is the default this does essentially nothing
			}
		}
		
		//GEAR_OUT mode, is toggle, reverts to the default drive mode when released
		if(robot.gearButton.changed()) {
			if(robot.gearButton.on()) {
				return DriveMode.GEAR;
			} else {
				return defaultMode;
			}
		}
		
		//if nothing has changed, maintain current drive mode
		return mode;
	}
	
	//determine which drive PID controllers should be active based on drive mode
	/** IS THIS FINE? Can I reset() or enable() PIDs every time, or should I do this only once?**/
	public void setDrivePIDs() {
		switch(mode) {
 		case ROBOT:		//ROBOT mode, no PIDs
			if (robot.rControl.isEnabled()) {
				robot.rControl.reset();
				robot.rControl.closeLog();
				System.out.println("Robot");
			}
 			break;
 		
 		case FIELD:		//FIELD mode, no PIDs
			if (robot.rControl.isEnabled()) {
				robot.rControl.reset();
				robot.rControl.closeLog();
				System.out.println("Field");
			}
 			break;
 		
 		case INTAKE:	//INTAKE mode, PIDing along direction of travel
			if (!robot.rControl.isEnabled()) {
				robot.rControl.enable();
				robot.rControl.enableLog("intake.csv");
				System.out.println("Intake");
			}
 			break;
 		
 		case GEAR:	//GEAR_OUT mode, PIDing to the nearest airship lift
			if (!robot.rControl.isEnabled()) {
				robot.rControl.enable();
				robot.rControl.enableLog("gearout.csv");
				System.out.println("GearOut");
				
			}
 			break;
 		}
	}
	
	//update PID setpoints and determine XYZ power outputs
	public void setDriveOutputs(double x, double y, double r, double t) {
		switch(mode) {
		case ROBOT:
			robot.setDriveX(x);
			robot.setDriveY(y);
			robot.setDriveR(r);
			robot.setDriveT(0);
			break;
		
		case FIELD:
			robot.setDriveX(x);
			robot.setDriveY(y);
			robot.setDriveR(r);
			robot.setDriveT(t);
			break;
		
		case INTAKE:
			//robot.rControl.setSetpoint(0);	//temporary, should calculate trajectory eventually
			robot.rControl.setSetpoint(robot.getCousin(robot.ahrs.getAngle(), getTrajectoryAngle(x, y)));
			
			robot.setDriveX(x);
			robot.setDriveY(y);
			robot.setDriveT(t);
			break;
		
		case GEAR:
			robot.rControl.setSetpoint(robot.getCousin(robot.ahrs.getAngle(), angle));
			robot.setDriveX(x);
			robot.setDriveY(y);
			robot.setDriveT(t);
			break;
		}
	}
	
	public void setGearAngle(){
		if(robot.operatorStick.getPOV() >= 0 && robot.operatorStick.getPOV() <= 45 || robot.operatorStick.getPOV()>=315) {
			angle = robot.MIDDLE_LIFT_ANGLE;
			
		} else if(robot.operatorStick.getPOV() == 90) {
			angle = robot.LEFT_LIFT_ANGLE;
			
		} else if(robot.operatorStick.getPOV() == 270) {
			angle = robot.RIGHT_LIFT_ANGLE;
			
		} else if(robot.operatorStick.getPOV() >= 135 && robot.operatorStick.getPOV()<= 225){
			angle = robot.GEAR_IN_ANGLE * robot.allianceMult;
		}
		
	}
	
	//calculate the current trajectory angle of the robot
	public double getTrajectoryAngle(double x, double y) {
		double p = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
		double t = Math.atan2(x, -y);
		
		//if the output is not defined or the change in position is very small (standing still), PID to the current rotation
		if(Double.isNaN(t) || Math.abs(p) < robot.INTAKE_ROTATION_THRESHOLD) {
			return robot.ahrs.getAngle();
		}
		
		System.out.println("p: " + p + " t: " + t * robot.RAD_TO_DEG);
		return t * robot.RAD_TO_DEG;
	}
}
