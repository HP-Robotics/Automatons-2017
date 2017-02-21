package org.usfirst.frc.team2823.robot;

public class ShootAutonomous extends Autonomous {
	
	public ShootAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {0.1, 5.0, 2.0, 0.1, 0.4, 2.0, 0.1};
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
			driveForward();
			break;
		
		case 2:
			turnRight();
			break;
		
		case 3:
			startUptake();
			break;
		
		case 4:
			driveIntoHopper();
			break;
		
		case 5:
			turnToShoot();
			break;
		
		case 6:
			startBeltFeeder();
			break;
		}
		//at time 14.25 turn off belt feeder
		//at time 14.75 turn off uptake
		
		//update drive motors regardless of stage
		robot.setDriveT(robot.ahrs.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//start the shooter wheel PIDs, relying on fast timeout to continue
	private void startShooterWheels() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.topShooter.speedMode();
			robot.topShooter.set(-4300);
			
			robot.bottomShooter.speedMode();
			robot.bottomShooter.set(4300);
			
			stageData[stage].entered = true;
			
			nextStage();
		}
	}
	
	//drive next to the hopper
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.yControl.reset();
			robot.rControl.reset();
			
			robot.yControl.configureGoal(70.4, robot.MAX_FORWARD_VEL, robot.MAX_FORWARD_ACCEL * 0.8);
			robot.yControl.enableLog("yControl.csv");
			
			robot.rControl.setSetpoint(0);
			
			robot.yControl.enable();
			robot.rControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage once plan is complete
		if(robot.yControl.isPlanFinished()) {
			robot.yControl.closeLog();
			
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
			robot.rControl.setSetpoint(90);
			robot.rControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage once plan is complete
		if(Math.abs(robot.rControl.getError()) < 2) {
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
			
			robot.setDriveX(-1.0);
			
			stageData[stage].entered = true;
		}
	}
	
	//turn to the right shooting angle
	private void turnToShoot() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.setDriveX(0.0);
			
			robot.rControl.reset();
			robot.rControl.setPID(robot.rControl.getP(), 0.0002, robot.rControl.getD());
			robot.rControl.setSetpoint(robot.ahrs.getAngle() + 10);
			
			robot.rControl.enable();
			
			stageData[stage].entered = true;
		}
		
		System.out.println(robot.rControl.getError());
		
		//move on to the next stage once plan is complete
		if(Math.abs(robot.rControl.getError()) < 0.5) {
			robot.rControl.reset();
			robot.rControl.setPID(robot.rControl.getP(), 0.0, robot.rControl.getD());
			
			nextStage();
		}
	}
	
	private void startBeltFeeder() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.beltFeed.set(-0.5);
			
			stageData[stage].entered = true;
			
			nextStage();
		}
	}
}
