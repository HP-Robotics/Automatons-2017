package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class FarShootAutonomous extends Autonomous {
	double unjamInitTime;
	
	public FarShootAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {0.2, 5.0, 2.0, 2.0, 0.5, 0.5, 0.5, 0.1, 0.1, 15.0, 15.0};
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
			setServoAngle();
			break;
		
		case 1:
			driveForward();
			break;
		
		case 2:
			turnRight();
			break;
		
		case 3:
			driveIntoHopper();
			break;
		
		case 4:
			startShooterWheels();
			break;
		
		case 5:
			startAgitators();
			break;
		
		case 6:
			startUptake();
		
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
		robot.setDriveT(robot.navx.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//set the shooter servos to the far shot position
	private void setServoAngle() {
		//run entry code
		if(!stageData[stage].entered) {
			//set servos to the far position
			robot.leftServo.set(robot.FAR_SERVO);
			robot.rightServo.set(robot.FAR_SERVO);
			
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
			
			robot.configureStrafe(robot.xControl);
			robot.configureStraight(robot.yControl);
			
			if(robot.allianceMult > 0) {
				robot.driveTo_Cartesian(0, 76);	//blue, real 81
			} else {
				robot.driveTo_Cartesian(0, 86); //red
			}
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
			
			robot.rotateTo_Relative(90);
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage once plan is complete
		//if(Math.abs(robot.rControl.getError()) < 2) {
		//	nextStage();
		//}
		if(robot.rMotionControl.isPlanFinished() && Math.abs(robot.navx.getAngle() - 90) < 2) {
			nextStage();
		}
	}
	
	//drive robot until stage times out
	private void driveIntoHopper() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.rMotionControl.reset();
			
			robot.configureStraightWithI(robot.xControl);
			robot.configureStrafe(robot.yControl);
			
			robot.driveTo_Cartesian(-52 * robot.allianceMult, robot.encoderThread.getY());	//real 62, 81
			robot.rotateTo(90);
			
			robot.yControl.disable();
			
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
			if(robot.allianceMult < 0){
				robot.topShooter.set(-robot.FAR_SHOT_SPEED + 100);
				robot.bottomShooter.set(robot.FAR_SHOT_SPEED - 100);
			}else{
				robot.topShooter.set(-robot.FAR_SHOT_SPEED);
				robot.bottomShooter.set(robot.FAR_SHOT_SPEED);
			}
			
			robot.shooterSolenoid.set(robot.FAR_SOLENOID);
			
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
			
			robot.rotateTo_Relative(9 * robot.allianceMult);
			
			stageData[stage].entered = true;
			
			nextStage();
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
	
	//periodically reverse the belt feeder to keep balls from jamming while shooting
	//turn off belt feeder at time 14.25 seconds
	private void stopBeltFeeder(){
		//run entry code
		if(!stageData[stage].entered) {
			unjamInitTime = Timer.getFPGATimestamp();
			
			stageData[stage].entered = true;
		}
		
		if((Timer.getFPGATimestamp() - unjamInitTime) > 2.0) {
			robot.beltFeed.set(-robot.BELT_FEED_SPEED);
		}
		
		if((Timer.getFPGATimestamp() - unjamInitTime) > 2.1) {			
			robot.beltFeed.set(robot.BELT_FEED_SPEED);
			
			unjamInitTime = Timer.getFPGATimestamp();
		}
		
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
