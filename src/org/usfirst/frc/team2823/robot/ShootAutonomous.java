package org.usfirst.frc.team2823.robot;

public class ShootAutonomous extends Autonomous {
	
	public ShootAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {5.0, 2.0, 0.25, 2.0};
		setStageTimeouts(timeouts);
		
		robot.ahrs.reset();
		robot.aEncoder.reset();
		
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
			turnToShoot();
			break;
		}
		
		//update drive motors regardless of stage
		robot.setDriveT(-robot.ahrs.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//drive next to the hopper
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.aEncoder.reset();
			robot.ahrs.reset();
			
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
			robot.ahrs.reset();
			
			robot.rControl.reset();
			robot.rControl.setSetpoint(90);
			robot.rControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage once plan is complete
		if(Math.abs(robot.rControl.getError()) < 2) {
			robot.rControl.reset();
			
			nextStage();
		}
	}
	
	//drive robot until stage times out
	private void driveIntoHopper() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.setDriveY(1.0);
			
			stageData[stage].entered = true;
		}
	}
	
	//turn to the right shooting angle
	private void turnToShoot() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.setDriveY(0.0);
			
			robot.ahrs.reset();
			
			robot.rControl.reset();
			
			robot.rControl.setSetpoint(10);
			
			robot.rControl.enable();
			
			stageData[stage].entered = true;
		}
		
		System.out.println(robot.rControl.getError());
		
		//move on to the next stage once plan is complete
		if(Math.abs(robot.rControl.getError()) < 0.5) {
			robot.rControl.reset();
			
			nextStage();
		}
	}
}
