package org.usfirst.frc.team2823.robot;

public class GearAutonomous extends Autonomous {
	Side side;
	
	enum Side {
		LEFT, CENTER, RIGHT
	}
	
	public GearAutonomous(Robot robot, Side s) {
		super(robot);
		
		side = s;
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
		robot.setDriveT(robot.navx.getAngle());
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//drive next to the lift
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			if(side == Side.CENTER) {
				robot.driveTo_Cartesian(0, 63.25, 0.6, 0.6);
			} else {
				robot.driveTo_Cartesian(0, 70.4, 0.6, 0.6);
			}
			
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
			
			if(side == Side.LEFT) {
				robot.rotateTo(-30/* * robot.allianceMult*/);	/** ARE THE SIDEWAYS DISTANCES THE SAME? CHECK **/
			} else if(side == Side.RIGHT) {
				robot.rotateTo(30);
			} else {
				robot.rotateTo(-90);
			}
			
			stageData[stage].entered = true;
		}
	}
	
	//drive to place the gear
	private void driveToLift() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			if(side == Side.LEFT) {
				robot.driveTo_Cartesian(72, 41, 0.6, 0.6);
				robot.rotateTo(-30);
			} else if(side == Side.RIGHT) {
				robot.driveTo_Cartesian(-72, 41, 0.6, 0.6);
				robot.rotateTo(30);
			} else{
				robot.driveTo_Cartesian(4, 36, 0.6, 0.6);
				robot.rotateTo(-90);
			}
			
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
