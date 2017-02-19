package org.usfirst.frc.team2823.robot;

public class DriveForwardAutonomous extends Autonomous {

	public DriveForwardAutonomous(Robot robot) {
		super(robot);
	
	}
	@Override
	public void init(){
		double[] timeouts = {60.0};
		setStageTimeouts(timeouts);
		
		start();
	}
	
	@Override
	public void periodic(){
		if(checkStageTimeout()){
			return;
		}
		
		switch(stage){
		case 0:
			driveForward();
			break;
		}
		
		robot.setDriveT(-robot.ahrs.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	public void driveForward(){
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(0, 2.37);
			
			stageData[stage].entered = true;
		}
		
		if(Math.abs(robot.yControl.getError()) < 0.1){
			nextStage();
		}
	}
}
