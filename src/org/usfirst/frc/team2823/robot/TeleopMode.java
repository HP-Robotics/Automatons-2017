package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopMode {
	Robot robot;
	
	//DriveMode defaultMode = DriveMode.INTAKE;
	DriveMode defaultMode = DriveMode.FIELD; //for testing purposes, the final default will likely be INTAKE mode
	DriveMode mode = defaultMode;
	DriveMode prevMode = defaultMode;
	
	double angle;
	
	double prevTime = Timer.getFPGATimestamp();
	double shotStartTime = Timer.getFPGATimestamp();
	
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
		
		updateButtons();
		
		//prevent joysticks from driving robot when within a threshold value of zero
		double x = Math.abs(robot.driverStick.getX()) < robot.STICKTHRESHOLD ? 0.0 : Math.pow(robot.driverStick.getX(), 3);
		double y = Math.abs(robot.driverStick.getY()) < robot.STICKTHRESHOLD ? 0.0 : Math.pow(robot.driverStick.getY(), 3);
		double r = Math.abs(robot.driverStick.getZ()) < robot.ROTATIONTHRESHOLD ? 0.0 : 0.75 * robot.driverStick.getZ();
		
		if(mode == DriveMode.GEAR || mode == DriveMode.ROBOT) {
			if(x > -0.4 && x < 0.4) {
				x = 0;
			} else if(x < 0) {
				x = Math.pow(robot.driverStick.getX(), 3) + 0.4;
			} else if(x > 0) {
				x = Math.pow(robot.driverStick.getX(), 3) - 0.4;
			}
			
		}
		
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
			System.out.println("a: " + robot.ahrs.getAngle() + " c: " + robot.getCousin(robot.ahrs.getAngle(), SmartDashboard.getNumber("Setpoint", 0.0)));
			prevTime = Timer.getFPGATimestamp();
		}
		
		//determine PID setpoint and drive motor outputs based on drive mode
		setDriveOutputs(x, y, r, t);
		
		//drive robot using calculated values
		//robot.robotDrive.mecanumDrive_Cartesian(x, y, r, t);
		//robot.robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
		//robot.opponentDrive.mecanumDrive_Cartesian(opx, opy, opr, opt);
		
		runShooter();
		runMechanisms();
		resetGyro();
		updateSmartDashboard();
	}
	
	public void updateButtons(){
		//update buttons
		robot.robotButton.update(robot.driverStick.getRawButton(2));
		robot.fieldButton.update(robot.driverStick.getRawButton(5));
		robot.gearButton.update(robot.driverStick.getRawButton(4));
		robot.intakeButton.update(robot.driverStick.getRawButton(3));
		robot.gyroResetButton1.update(robot.driverStick.getRawButton(11));
		robot.gyroResetButton2.update(robot.driverStick.getRawButton(12));
		
		try{
			robot.shootTrigger.update(robot.driverStick.getRawButton(1) || robot.operatorStick.getRawButton(2));
			robot.climbButton.update(robot.operatorStick.getRawButton(4));
			robot.reverseBeltButton.update(robot.operatorStick.getRawButton(3));
			robot.farShotButton.update(robot.operatorStick.getRawButton(7));
			robot.nearShotButton.update(robot.operatorStick.getRawButton(5));
			robot.intakeState.update(robot.operatorStick.getRawButton(8));
					
		}catch (Exception e){
					
		}
	}
	
	//select a drive mode based on button input
	public DriveMode setDriveMode() {
		
		//ROBOT mode, is press-hold, has priority over other modes but reverts to previous mode when released
		// is not field-oriented, gear catcher is forward in this mode
		if(robot.robotButton.held()) {
			if(mode != DriveMode.ROBOT) {
				prevMode = mode;		//if switching to ROBOT mode, store the previous drive mode
			}
			return DriveMode.ROBOT;
			
		} else if(mode == DriveMode.ROBOT) {
			return prevMode;			//if switching out of ROBOT mode, revert to the previous drive mode
		}
		
		//FIELD mode, is toggle
		if(robot.fieldButton.changed()) {
			return DriveMode.FIELD;
		}
		
		//INTAKE mode, is toggle, is the default drive mode
		if(robot.intakeButton.changed()) {
			return DriveMode.INTAKE;
		}
		
		//GEAR_OUT mode, is toggle
		if(robot.gearButton.changed()) {
			return DriveMode.GEAR;
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
			robot.setDriveX(-y);	//switch x and y to make gear catcher forward
			robot.setDriveY(x);
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
			double p = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
			
			if(Math.abs(p) > robot.INTAKE_ROTATION_THRESHOLD) {
				if(!robot.rControl.isEnabled()) {
					robot.rControl.enable();
				}
				
				//System.out.println(robot.ahrs.getAngle() + " " + robot.getCousin(robot.ahrs.getAngle(),  getTrajectoryAngle(x, y)) + " " + getTrajectoryAngle(x, y));
				robot.rControl.setSetpoint(robot.getCousin(robot.ahrs.getAngle(), getTrajectoryAngle(x, y)));
			} else {
				if(robot.rControl.isEnabled()) {
					robot.rControl.reset();
				}
			}
			
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
	
	//calculate the current trajectory angle of the robot, return the angle
	public double getTrajectoryAngle(double x, double y) {
		double p = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
		double t = Math.atan2(x, -y);
		
		//if the output is not defined or the change in position is very small (standing still), PID to the current rotation
		if(Double.isNaN(t) || Math.abs(p) < robot.INTAKE_ROTATION_THRESHOLD) {
			return robot.ahrs.getAngle();
		}
		
		//System.out.println("p: " + p + " t: " + t * robot.RAD_TO_DEG);
		return t * robot.RAD_TO_DEG;
	}
	
	//control shooter wheels, uptake, belt feed, and climber-agitator
	public void runShooter() {
		if(robot.shootTrigger.held()) {
			//capture initial time
			if(robot.shootTrigger.changed()) {
				shotStartTime = Timer.getFPGATimestamp();
			}
			
			//start shooter wheels first
			if(Timer.getFPGATimestamp() - shotStartTime > 0.1) {
				robot.topShooter.speedMode();
				robot.bottomShooter.speedMode();
				
				//robot.topShooter.set(-SmartDashboard.getNumber("Setpoint", 4300));
				//robot.bottomShooter.set(SmartDashboard.getNumber("Setpoint", 4300));
				
				//set shot speed based on solenoid position
				if(robot.shooterSolenoid.get() == robot.CLOSE_SOLENOID) {
					robot.topShooter.set(-robot.CLOSE_SHOT_SPEED);
					robot.bottomShooter.set(robot.CLOSE_SHOT_SPEED);
				} else {
					robot.topShooter.set(-robot.FAR_SHOT_SPEED);
					robot.bottomShooter.set(robot.FAR_SHOT_SPEED);
				}
			}
			
			//start uptake next
			if(Timer.getFPGATimestamp() - shotStartTime > 0.2) {
				robot.uptake.set(1.0);
			}
			
			//start belt feed and agitators last
			if(Timer.getFPGATimestamp() - shotStartTime > 0.3) {
				robot.runningFeeder = true;
				
				robot.climbMotor1.set(-1.0);
				robot.climbMotor2.set(-1.0);
			}
		} else {
			//capture initial time
			if(robot.shootTrigger.changed()) {
				shotStartTime = Timer.getFPGATimestamp();
			}
			
			//stop belt feed first
			if(Timer.getFPGATimestamp() - shotStartTime > 0.1) {
				robot.runningFeeder = false;
				robot.beltFeed.set(0.0);
			}
			
			//stop uptake next
			if(Timer.getFPGATimestamp() - shotStartTime > 0.2) {
				robot.uptake.set(0.0);
			}
			
			//stop shooter wheels last, leaving a larger time interval to smooth transition
			if(Timer.getFPGATimestamp() - shotStartTime > 0.5) {
				robot.topShooter.normalMode();
				robot.bottomShooter.normalMode();
				
				robot.topShooter.set(0.0);
				robot.bottomShooter.set(0.0);
			}
		}
		
		//operator control of reverse feeder
		if(robot.runningFeeder) {
			if(robot.reverseBeltButton.held()) {
				robot.beltFeed.set(-robot.BELT_FEED_SPEED);
			} else {
				robot.beltFeed.set(robot.BELT_FEED_SPEED);
			}
		}
	}
	
	public void runMechanisms(){
		if(robot.farShotButton.changed()) {
			robot.shooterSolenoid.set(robot.FAR_SOLENOID);
			
		} else if(robot.nearShotButton.changed()) {
			robot.shooterSolenoid.set(robot.CLOSE_SOLENOID);
		}
		
		if(robot.intakeState.on()){
			robot.intake.set(1.0);
		}else{
			robot.intake.set(0.0);
		}
		
		if(robot.climbButton.on() && !robot.shootTrigger.held()){
			robot.climbMotor1.set(-1.0);//not production values
			robot.climbMotor2.set(-1.0);
		} else if(!robot.climbButton.on() && !robot.shootTrigger.held()) {
			robot.climbMotor1.set(0.0);
			robot.climbMotor2.set(0.0);
		}
	}
	
	public void resetGyro(){
		if(robot.gyroResetButton1.held() && robot.gyroResetButton2.held()) {
			if(!robot.resettingGyro) {
				robot.resettingGyro = true;
				
				robot.ahrs.reset();
			}
		} else {
			robot.resettingGyro = false;
		}
	}
	
	public void updateSmartDashboard (){
		SmartDashboard.putNumber("X: ", robot.encoderThread.getX());
        SmartDashboard.putNumber("Y: ", robot.encoderThread.getY());
        SmartDashboard.putNumber("R: ", robot.encoderThread.getR());
        SmartDashboard.putNumber("L Distance: ", robot.encoderThread.getLDistance());
        SmartDashboard.putNumber("R Distance: ", robot.encoderThread.getRDistance());
        SmartDashboard.putNumber("C Distance: ", robot.encoderThread.getCDistance());
        SmartDashboard.putBoolean("Robot Mode", mode == DriveMode.ROBOT);
        SmartDashboard.putBoolean("Intake Mode", mode == DriveMode.INTAKE);
        SmartDashboard.putBoolean("Field Mode", mode == DriveMode.FIELD);
        SmartDashboard.putBoolean("Gear Mode", mode == DriveMode.GEAR);
        SmartDashboard.putBoolean("Climber", robot.climbButton.on());
        SmartDashboard.putBoolean("Intake", robot.intakeState.on());
	}
}
