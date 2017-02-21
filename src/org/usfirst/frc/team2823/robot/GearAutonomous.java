package org.usfirst.frc.team2823.robot;

public class GearAutonomous extends Autonomous {

	public GearAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {5.0, 2.0, 5.0};
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
			rotateToLift();
			break;
		
		case 2:
			driveToLift();
			break;
		}
		
		//update drive motors regardless of stage
		robot.setDriveT(robot.ahrs.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//drive next to the lift
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			robot.driveTo_Cartesian(0, 70.4);
			robot.rotateTo(0);
			
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
	
	//rotate to point gear loader toward lift
	/** TODO make this read SmartDashboard to decide which lift to rotate to **/
	private void rotateToLift() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.rControl.reset();
			
			robot.rotateTo(-60 * robot.allianceMult);
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage once sufficiently close to target
		if(Math.abs(robot.rControl.getError()) < 0.5) {
			robot.rControl.closeLog();
			robot.rControl.reset();
			
			nextStage();
		}
	}
	
	//drive to place the gear
	private void driveToLift() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			robot.driveTo_Cartesian(45 * robot.allianceMult, 45);
			robot.rotateTo(-60 * robot.allianceMult);
			
			stageData[stage].entered = true;
		}
		
		if(robot.yControl.isPlanFinished() && robot.xControl.isPlanFinished()) {
			robot.xControl.closeLog();
			robot.yControl.closeLog();
			robot.rControl.closeLog();
			
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			nextStage();
		}
	}
}
