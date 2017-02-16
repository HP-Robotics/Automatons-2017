package org.usfirst.frc.team2823.robot;

public class ShootAutonomous extends Autonomous {
	
	public ShootAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {60.0, 3.0, 60.0};
		setStageTimeouts(timeouts);
		
		start();
	}
	
	@Override
	public void periodic() {
		if(checkStageTimeout()) {
			return;
		}
		
		//select which stage should be run, this influences which PIDs and which motors will run
		switch(stage) {
		case 0:
			driveForward();
			break;
		
		case 1:
			driveToHopper();
			break;
		
		case 2:
			driveToShootPosition();
			break;
		}
		
		//update heading regardless of stage
		robot.setDriveT(-robot.gyro.getAngle());
		
		//set drive motor powers after PIDs have run, regardless of stage
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//Robot action functions, in sequential order:
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(2.017, 0);
			robot.vControl.enable();
			
			stageData[stage].entered = true;
		}
		//System.out.println(robot.encoderThread.getX() + " " + robot.vControl.getError());// + robot.encoderThread.getY());
		
		//move on once the robot is acceptably close to the target
		if(Math.abs(robot.vControl.getError()) < 0.1) {
			nextStage();
		}
	}
	
	private void driveToHopper() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(2.017, -1.0);
			robot.vControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on once the robot is acceptably close to the target
		if(Math.abs(robot.vControl.getError()) < 0.1) {
			nextStage();
		}
	}
	
	private void driveToShootPosition() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(1.609, -1.0);
			robot.rControl.setSetpoint(3.14);
			
			robot.vControl.enable();
			robot.rControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on once the robot is acceptably close to the target
		if(Math.abs(robot.vControl.getError()) < 0.1) {
			nextStage();
		}
	}

}
