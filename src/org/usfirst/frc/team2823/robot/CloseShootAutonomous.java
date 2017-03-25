package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class CloseShootAutonomous extends Autonomous {
	
	public CloseShootAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {5.0, 0.9, 2.0, 0.5, 0.5, 0.5, 0.9, 5.0, 0.6, 0.1, 2.0, 15.0, 15.0};
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
			driveForward();
			break;
		
		case 1:
			turnRight();
			break;
		
		case 2:
			driveIntoHopper();
			break;
			
		case 3:
			startShooterWheels();
			break;
		
		case 4:
			startAgitators();
			break;
		
		case 5:
			startUptake();
			break;
		
		case 6:
			turnLeft();
			break;
		
		case 7:
			driveToBoiler();
			break;
			
		case 8:
			turnToShoot();
			break;
		
		case 9:
			startBeltFeeder();
			break;
		
		case 10:
			driveIntoBoiler();
			break;
			
		case 11:
			stopBeltFeeder();
			break;
		
		case 12:
			stopUptake();
			break;
		}
		
		//update drive motors regardless of stage
		robot.setDriveT(robot.navx.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//drive next to the hopper
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
		robot.driveTo_Cartesian(0, 84, "x-" + stage + ".csv", "y-" + stage + ".csv");
			robot.rotateTo(0, "r-" + stage + ".csv");
			
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
			
			robot.rotateTo(90, "r-" + stage + ".csv");
			
			stageData[stage].entered = true;
		}
		
	}
	
	//drive robot until stage times out
	private void driveIntoHopper() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.rControl.reset();
			
			robot.driveTo_Cartesian(-62 * robot.allianceMult, 0, "x-" + stage + ".csv", "y-" + stage + ".csv");
			robot.rotateTo(90, "r-" + stage + ".csv");
			
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
	
	//start the shooter wheel PIDs, relying on timeout to continue
	private void startShooterWheels() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.topShooter.speedMode();
			robot.bottomShooter.speedMode();
			
			robot.topShooter.set(-robot.CLOSE_SHOT_SPEED);
			robot.bottomShooter.set(robot.CLOSE_SHOT_SPEED);
			
			robot.shooterSolenoid.set(robot.CLOSE_SOLENOID);
			
			stageData[stage].entered = true;
		}
	}
	
	//start the agitation mechanisms to prime the balls for shooting, relying on timeout to continue
	private void startAgitators() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.intake.set(1.0);
			
			robot.climbMotor1.set(-1.0);
			robot.climbMotor2.set(-1.0);
			
			stageData[stage].entered = true;
		}
	}
	
	//start the uptake, relying on timeout to continue
	private void startUptake() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.uptake.set(1.0);
			
			stageData[stage].entered = true;
		}
	}
	
	//wait for the balls to fall into the robot
	private void waitForBalls() {
		//do nothing, wait for stage to time out
	}
	
	private void turnLeft() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.rControl.reset();
			
			robot.rotateTo(0, "r-" + stage + ".csv");
			
			stageData[stage].entered = true;
		}
		
	}
	
	//drive next to the hopper
	private void driveToBoiler() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			robot.driveTo_Cartesian(0, -56, "x-" + stage + ".csv", "y-" + stage + ".csv"); //BAD NUMBER, MEASURE REAL
			robot.rotateTo(0, "r-" + stage + ".csv");
			
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
	
	//turn rotation PID on, turn to the right shooting angle while shooting
	private void turnToShoot() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.rControl.reset();
			
			robot.rotateTo(90 + (45 * robot.allianceMult), "r-" + stage + ".csv");
			
			stageData[stage].entered = true;
		}
	
	}
	
	//start belt feeder and begin shooting
	private void startBeltFeeder() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.beltFeed.set(robot.BELT_FEED_SPEED);
			
			stageData[stage].entered = true;
			
			nextStage();
		}
	}
	
	//drive robot until stage times out
	private void driveIntoBoiler() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.rControl.reset();
			
			robot.driveTo_Cartesian(-10, -23, "x-" + stage + ".csv", "y-" + stage + ".csv");
			robot.rotateTo(90 + (45 * robot.allianceMult), "r-" + stage + ".csv");
			
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
	
	//turn off belt feeder at time 14.25 seconds
	private void stopBeltFeeder(){
		if(Timer.getFPGATimestamp() - initTime >= 14.6){
			robot.beltFeed.set(0.0);
			nextStage();
		}
	}
	
	//turn off uptake at time 14.75 seconds
	private void stopUptake(){
		if(Timer.getFPGATimestamp() - initTime >= 14.8){
			robot.uptake.set(0.0);
			robot.climbMotor1.set(0.0);
			robot.climbMotor2.set(0.0);
			nextStage();
		}
	}
}
