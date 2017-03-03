package org.usfirst.frc.team2823.robot;

public class DriveForwardAutonomous extends Autonomous {
	
	public DriveForwardAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {7.0};
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
		}
		
		//update drive motors regardless of stage
		robot.setDriveT(robot.ahrs.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//drive across the baseline
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.xControl.reset();
			robot.yControl.reset();
			
			robot.driveTo_Cartesian(0, 60, 0.6, 0.6);
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage once plan is complete
		if(robot.yControl.isPlanFinished()) {
			robot.xControl.closeLog();
			robot.yControl.closeLog();
			
			robot.xControl.reset();
			robot.yControl.reset();
			
			nextStage();
		}
	}
}
