package org.usfirst.frc.team2823.robot;

public class ShootAutonomous extends Autonomous {
	
	public ShootAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {60.0, 60.0};
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
			driveSideways();
			break;
		}
		
		//update heading regardless of stage
		robot.setDriveT(robot.gyro.getAngle());
		
		//set drive motor powers after PIDs have run, regardless of stage
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//Robot action functions, in sequential order:
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.driveTo(1000, 0);
			robot.vControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on once the robot is acceptably close to the target
		if(Math.abs(robot.vControl.getError()) < 50) {
			nextStage();
		}
	}
	
	private void driveSideways() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.driveTo(2000, -1000);
			robot.vControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on once the robot is acceptably close to the target
		if(Math.abs(robot.vControl.getError()) < 50) {
			nextStage();
		}
	}

}
