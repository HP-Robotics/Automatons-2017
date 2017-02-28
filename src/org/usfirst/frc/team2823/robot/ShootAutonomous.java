package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;

public class ShootAutonomous extends Autonomous {
	
	public ShootAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {0.1, 0.1, 5.0, 2.0, 0.1, 2.0, 1.5, 0.1, 0.1, 15.0, 15.0};
		setStageTimeouts(timeouts);
		
		start();
	}
	
	@Override
	public void periodic() {
		if(checkStageTimeout()) {
			return;
		}
		
		switch(stage) {
		case 0:
			startShooterWheels();
			break;
		
		case 1:
			startAgitators();
			break;
		
		case 2:
			driveForward();
			break;
		
		case 3:
			turnRight();
			break;
		
		case 4:
			startUptake();
			break;
		
		case 5:
			driveIntoHopper();
			break;
		
		case 6:
			waitForBalls();
			break;
		
		case 7:
			turnToShoot();
			break;
		
		case 8:
			startBeltFeeder();
			break;
			
		case 9:
			stopBeltFeeder();
			break;
		
		case 10:
			stopUptake();
			break;
		}
		
		//update drive motors regardless of stage
		robot.setDriveT(robot.ahrs.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//start the shooter wheel PIDs, relying on fast timeout to continue
	private void startShooterWheels() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.topShooter.speedMode();
			robot.topShooter.set(-robot.FAR_SHOT_SPEED);
			
			robot.bottomShooter.speedMode();
			robot.bottomShooter.set(robot.FAR_SHOT_SPEED);
			
			stageData[stage].entered = true;
			
			nextStage();
		}
	}
	
	//start the agitation mechanisms to prime the balls for shooting
	private void startAgitators() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.intake.set(1.0);
			
			robot.climbMotor1.set(-1.0);
			robot.climbMotor2.set(-1.0);
			
			stageData[stage].entered = true;
			
			nextStage();
		}
	}
	
	//drive next to the hopper
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			robot.driveTo_Cartesian(0, 71);
			robot.rotateTo(0);
			
			//robot.yControl.configureGoal(76.4, robot.MAX_FORWARD_VEL, robot.MAX_FORWARD_ACCEL * 0.8);
			//robot.yControl.enableLog("yControl.csv");
			
			//robot.rControl.setSetpoint(0);
			
			//robot.yControl.enable();
			//robot.rControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage once plan is complete
		if(robot.yControl.isPlanFinished()) {
			robot.xControl.closeLog();
			robot.yControl.closeLog();
			robot.rControl.closeLog();
			
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			nextStage();
		}
	}
	
	//turn to point shooter toward boiler
	private void turnRight() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.rControl.reset();
			
			robot.rotateTo(90);
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage once plan is complete
		if(Math.abs(robot.rControl.getError()) < 10) {
			nextStage();
		}
	}
	
	//start the uptake early
	private void startUptake() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.uptake.set(1.0);
			
			stageData[stage].entered = true;
			
			nextStage();
		}
	}
	
	//drive robot until stage times out
	private void driveIntoHopper() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.rControl.reset();
			
			robot.driveTo_Cartesian(-42, 0);
			robot.rotateTo(90);
			
			stageData[stage].entered = true;
		}
		
		if(robot.xControl.isPlanFinished()) {
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			robot.xControl.closeLog();
			robot.yControl.closeLog();
			robot.rControl.closeLog();
			
			nextStage();
		}
	}
	
	//wait for the balls to fall into the robot
	private void waitForBalls() {
		//do nothing, wait for stage to time out
	}
	
	//turn rotation PID on, turn to the right shooting angle while shooting
	private void turnToShoot() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.rControl.reset();
			
			robot.rotateTo((robot.ahrs.getAngle() + 10) * robot.allianceMult);
			
			stageData[stage].entered = true;
			
			nextStage();
		}
	}
	
	//start belt feeder and begin shooting
	private void startBeltFeeder() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.beltFeed.set(-0.5);
			
			stageData[stage].entered = true;
			
			nextStage();
		}
	}
	
	//turn off belt feeder at time 14.25 seconds
	private void stopBeltFeeder(){
		if(Timer.getFPGATimestamp() - initTime >= 14.25){
			robot.beltFeed.set(0.0);
			nextStage();
		}
	}
	
	//turn off uptake at time 14.75 seconds
	private void stopUptake(){
		if(Timer.getFPGATimestamp() - initTime >= 14.75){
			robot.uptake.set(0.0);
			robot.climbMotor1.set(0.0);
			robot.climbMotor2.set(0.0);
			nextStage();
		}
	}
}
