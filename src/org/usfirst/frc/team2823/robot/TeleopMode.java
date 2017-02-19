package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;

public class TeleopMode {
	Robot robot;
	
	//DriveMode defaultMode = DriveMode.INTAKE;
	DriveMode defaultMode = DriveMode.FIELD; //for testing purposes, the final default will likely be INTAKE mode
	DriveMode mode = defaultMode;
	DriveMode prevMode = defaultMode;
	
	//previous position data, used for calculation direction of travel
	double px;
	double py;
	
	double prevTime = Timer.getFPGATimestamp();
	
	private enum DriveMode {
		ROBOT, FIELD, INTAKE, GEAR_IN, GEAR_OUT
	}
	
	public TeleopMode(Robot robot) {
		this.robot = robot;
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
		robot.intakeButton.update(robot.driverStick.getRawButton(3));
		robot.gearOutButton.update(robot.driverStick.getRawButton(6));
		robot.gearInButton.update(robot.driverStick.getRawButton(4));
		robot.shootTrigger.update(robot.driverStick.getRawButton(1));
		
		try{
			robot.shooterWheelsButton.update(robot.operatorStick.getRawButton(1));
			robot.climbButton.update(robot.operatorStick.getRawButton(4));
			robot.intakeState.update(robot.operatorStick.getPOV() >= 135 && robot.operatorStick.getPOV()<= 225);
		}catch (Exception e){
			
		}
		//prevent joysticks from driving robot when within a threshold value of zero
		double x = Math.abs(robot.driverStick.getX()) < robot.STICKTHRESHOLD ? 0.0 : Math.pow(robot.driverStick.getX(), 3);
		double y = Math.abs(robot.driverStick.getY()) < robot.STICKTHRESHOLD ? 0.0 : Math.pow(robot.driverStick.getY(), 3);
		double r = Math.abs(robot.driverStick.getZ()) < robot.STICKTHRESHOLD ? 0.0 : Math.pow(robot.driverStick.getZ(), 3);
		
		//double opx = Math.abs(robot.opponentStick.getX()) < robot.kStickThreshold ? 0.0 : robot.opponentStick.getX();
		//double opy = Math.abs(robot.opponentStick.getY()) < robot.kStickThreshold ? 0.0 : robot.opponentStick.getY();
		//double opr = Math.abs(robot.opponentStick.getZ()) < robot.kStickThreshold ? 0.0 : robot.opponentStick.getZ();
		
		
		//get gyro angle
		double t = -robot.ahrs.getAngle();
		//double opt = -(robot.opponentGyro.getAngle() + 90);
		
		//robot.log.write(-robot.gyro.getAngle() + "," + -robot.ahrs.getAngle() + "," + robot.encoderThread.getR() + "\n");
		//System.out.println(-robot.gyro.getAngle() + " " + -robot.ahrs.getAngle() + " " + robot.encoderThread.getR());
		
		//update which drive mode the robot is in
		mode = setDriveMode();
		
		//update whether rotation PID is enabled
		/** CHANGE NAME? GOOD ENOUGH?**/
		setDrivePIDs();
				
		if(Math.abs(Timer.getFPGATimestamp() - prevTime) > 1.0) {
			//System.out.println("x: " + robot.encoderThread.getX() + " y: " + robot.encoderThread.getY() + " r: " + robot.encoderThread.getR());
			System.out.println("l: " + robot.encoderThread.getLDistance() + " r: " + robot.encoderThread.getRDistance() + " c: " + robot.encoderThread.getCDistance());
			prevTime = Timer.getFPGATimestamp();
		}
		
		//determine PID setpoint and drive motor outputs based on drive mode
		setDriveOutputs(x, y, r, t);
		
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
			robot.beltFeed.set(-1.0);
		} else{
			robot.uptake.set(0.0);
			robot.beltFeed.set(0.0);
		}
		
		if(robot.shooterWheelsButton.on()){
			robot.topShooter.speedMode();
			robot.topShooter.set(-4500);
			robot.bottomShooter.speedMode();
			robot.bottomShooter.set(4500);
		} else{
			robot.topShooter.normalMode();
			robot.topShooter.set(0.0);
			robot.bottomShooter.normalMode();
			robot.bottomShooter.set(0.0);
		}
		
		/*if(robot.intakeState.on()){
			robot.beltFeed.set(-1.0);
		}else{
			robot.beltFeed.set(0.0);
		}*/
		
		if(robot.climbButton.on()){
			robot.climbMotor1.set(-1.0);//not production values
			robot.climbMotor2.set(-1.0);
		} else {
			robot.climbMotor1.set(0.0);
			robot.climbMotor2.set(0.0);
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
		if(robot.gearOutButton.changed()) {
			if(robot.gearOutButton.on()) {
				return DriveMode.GEAR_OUT;
			} else {
				return defaultMode;
			}
		}
		
		//GEAR_IN mode, is toggle, reverts to the default drive mode when released
		if(robot.gearInButton.changed()) {
			if(robot.gearInButton.on()) {
				return DriveMode.GEAR_IN;
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
 		
 		case GEAR_OUT:	//GEAR_OUT mode, PIDing to the nearest airship lift
			if (!robot.rControl.isEnabled()) {
				robot.rControl.enable();
				robot.rControl.enableLog("gearout.csv");
				System.out.println("GearOut");
			}
 			break;
 		
 		case GEAR_IN:	//GEAR_IN mode, PIDing to the feeder station
			if (!robot.rControl.isEnabled()) {
				robot.rControl.setSetpoint(1.5707);
				robot.rControl.enable();
				robot.rControl.enableLog("gearin.csv");
				System.out.println("GearIn");
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
			robot.rControl.setSetpoint(getTrajectoryAngle());
			
			robot.setDriveX(x);
			robot.setDriveY(y);
			robot.setDriveT(t);
			break;
		
		case GEAR_IN:
			//robot.rControl.setSetpoint(/*some calculated angle based on field position*/);
			
			robot.setDriveX(x);
			robot.setDriveY(y);
			robot.setDriveT(t);
			break;
		
		case GEAR_OUT:
			//robot.rControl.setSetpoint(/*some precalculated angle to the feeder station*/);
			
			robot.setDriveX(x);
			robot.setDriveY(y);
			robot.setDriveT(t);
			break;
		}
	}
	
	//calculate the current trajectory angle of the robot
	public double getTrajectoryAngle() {
		double x = robot.encoderThread.getX();
		double y = robot.encoderThread.getY();
		
		double dx = x - px;
		double dy = y - py;
		
		double dp = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		double t = Math.atan(dx / dy);
		
		px = x;
		py = y;
		
		//if the output is not defined or the change in position is very small (standing still), PID to the current rotation
		if(Double.isNaN(t) || Math.abs(dp) < robot.INTAKE_ROTATION_THRESHOLD) {
			return robot.encoderThread.getR();
		}
		
		return t;
	}
}
