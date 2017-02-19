package org.usfirst.frc.team2823.robot;

public class GearAutonomous extends Autonomous {

	public GearAutonomous(Robot robot) {
		super(robot);

	}
	
	public void init(){
		double[] timeouts = {60.0 , 60.0, 5.0, 60.0};
		setStageTimeouts(timeouts);
		
		start();
	}
	
	public void periodic(){
		if(checkStageTimeout()){
			return;
		}
		
		switch(stage){
		case 0:
			driveForward();
			break;
		case 1:
			driveToLift();
			break;
		case 2:
			break;
		case 3:
			driveBack();
			break;
		}
		
		robot.setDriveT(-robot.ahrs.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	public void driveForward(){
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(0, 1.877);
			robot.rotateTo(30 * robot.DEG_TO_RAD);
			stageData[stage].entered = true;
		}
		
		if(Math.abs(robot.yControl.getError()) < 0.1 && (Math.abs(robot.rControl.getError()) * robot.RAD_TO_DEG )< 1.0 ){
			nextStage();
		}
	}
	
	public void driveToLift(){
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(1, 2.724);
			robot.rotateTo(30 * robot.DEG_TO_RAD);
			stageData[stage].entered = true;
		}
		if(Math.abs(robot.yControl.getError()) < 0.1 && Math.abs(robot.xControl.getError()) < 1.0){
			nextStage();
		}
	}
	
	public void driveBack(){
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(0.9, 2.624);
			stageData[stage].entered = true;
		}
		if(Math.abs(robot.yControl.getError()) < 0.1 && Math.abs(robot.xControl.getError()) < 1.0){
			nextStage();
	
		}
	}
}
