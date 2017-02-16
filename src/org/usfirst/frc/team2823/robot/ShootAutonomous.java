package org.usfirst.frc.team2823.robot;

public class ShootAutonomous extends Autonomous {
	
	public ShootAutonomous(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		double[] timeouts = {0.1, 60.0, 3.0, 60.0, 60.0};
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
			startShooterWheels();
			break;
			
		case 1:
			driveForward();
			break;
		
		case 2:
			driveToHopper();
			break;
		
		case 3:
			driveToShootPosition();
			break;
		
		case 4:
			startShooting();
			break;
		}
		
		//System.out.println(robot.encoderThread.getX() + " " + robot.vControl.getError());// + robot.encoderThread.getY());
		
		//update heading regardless of stage
		robot.setDriveT(-robot.gyro.getAngle());
		
		//set drive motor powers after PIDs have run, regardless of stage
		robot.robotDrive.mecanumDrive_Cartesian(robot.getDriveX(), robot.getDriveY(), robot.getDriveR(), robot.getDriveT());
	}
	
	//Robot action functions, in sequential order:
	//turn on shooter PIDs so they can spin up during movement
	private void startShooterWheels() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.topShooter.speedMode();
			robot.bottomShooter.speedMode();
			
			robot.topShooter.set(4500);
			robot.bottomShooter.set(4500);
			
			nextStage();
		}
	}
	
	//drive next to the hopper
	private void driveForward() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(0, 2.017);
			robot.rotateTo(-1.5707);
			
			stageData[stage].entered = true;
		}
		
		//move on once the robot is acceptably close to the target
		if(Math.abs(robot.yControl.getError()) < 0.1) {
			nextStage();
		}
	}
	
	//drive into the hopper press plate
	private void driveToHopper() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(-1.0, 2.017);
			
			stageData[stage].entered = true;
		}
		
		//move on once the robot is acceptably close to the target
		if(Math.abs(robot.xControl.getError()) < 0.1) {
			nextStage();
		}
	}
	
	//drive forward a bit to align fully with the hopper
	private void driveToShootPosition() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.driveTo_Cartesian(-1.18, 1.68);
			robot.rotateTo(-1.5707);
			
			stageData[stage].entered = true;
		}
		
		//move on once the robot is acceptably close to the target
		if(Math.abs(robot.xControl.getError()) < 0.01 && Math.abs(robot.yControl.getError()) < 0.05) {
			nextStage();
		}
	}
	
	private void startShooting() {
		//run entry code
		if(!stageData[stage].entered) {
			robot.intake.set(1.0);
			robot.uptake.set(1.0);
		}
	}

}
