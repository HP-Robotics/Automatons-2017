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
		double[] timeouts = {5.0, 2.0, 5.0, 1.0, 2.5, 0.1};
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
		
		case 3:
			placeGear();
			break;
		
		case 4:
			driveBack();
			break;
		
		case 5:
			retractKicker();
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
				robot.rotateTo(robot.LEFT_LIFT_ANGLE);
			} else if(side == Side.RIGHT) {
				robot.rotateTo(robot.RIGHT_LIFT_ANGLE);
			} else {
				robot.rotateTo(robot.MIDDLE_LIFT_ANGLE);
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
				robot.rotateTo(robot.LEFT_LIFT_ANGLE);
			} else if(side == Side.RIGHT) {
				robot.driveTo_Cartesian(-72, 41, 0.6, 0.6);
				robot.rotateTo(robot.RIGHT_LIFT_ANGLE);
			} else{
				robot.driveTo_Cartesian(4, 36, 0.6, 0.6);
				robot.rotateTo(robot.MIDDLE_LIFT_ANGLE);
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
	
	//kick the gear onto the spring
	private void placeGear() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.gearControl.setSetpoint(robot.GEAR_KICK_OUT * robot.DEG_TO_G_ENC);
			
			stageData[stage].entered = true;
		} 
	}
	
	
	//drive clear of the lift
	private void driveBack() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.xControl.reset();
			robot.yControl.reset();
			robot.rControl.reset();
			
			if(side == Side.LEFT) {
				robot.driveTo_Cartesian(-72, -41, 0.6, 0.6);
				robot.rotateTo(robot.LEFT_LIFT_ANGLE);
			} else if(side == Side.RIGHT) {
				robot.driveTo_Cartesian(72, -41, 0.6, 0.6);
				robot.rotateTo(robot.RIGHT_LIFT_ANGLE);
			} else{
				robot.driveTo_Cartesian(-4, -36, 0.6, 0.6);
				robot.rotateTo(robot.MIDDLE_LIFT_ANGLE);
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
	
	//retract the gear kicker
	private void retractKicker() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.gearControl.setSetpoint(robot.GEAR_KICK_IN * robot.DEG_TO_G_ENC);
			
			stageData[stage].entered = true;
		} 
	}
	
}
