package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;

public class TeleopMode {
	Robot robot;
	
	//DriveMode defaultMode = DriveMode.INTAKE;
	DriveMode defaultMode = DriveMode.FIELD; //for testing purposes, the final default will likely be INTAKE mode
	DriveMode mode = defaultMode;
	DriveMode prevMode = defaultMode;
	
	private enum DriveMode {
		ROBOT, FIELD, INTAKE, GEAR_IN, GEAR_OUT
	}
	
	public TeleopMode(Robot robot) {
		this.robot = robot;
	}
	
	public void teleopInit() {
		
	}
	
	public void teleopPeriodic() {
		//update buttons
		robot.robotButton.update(robot.stick1.getRawButton(2));
		robot.fieldButton.update(robot.stick1.getRawButton(5));
		robot.intakeButton.update(robot.stick1.getRawButton(3));
		robot.gearOutButton.update(robot.stick1.getRawButton(6));
		robot.gearInButton.update(robot.stick1.getRawButton(4));
		
		robot.xButton.update(robot.stick2.getRawButton(1));
		
		//prevent joysticks from driving robot when within a threshold value of zero
		double x = Math.abs(robot.stick1.getX()) < robot.kStickThreshold ? 0.0 : robot.stick1.getX();
		double y = Math.abs(robot.stick1.getY()) < robot.kStickThreshold ? 0.0 : robot.stick1.getY();
		double z = Math.abs(robot.stick1.getZ()) < robot.kStickThreshold ? 0.0 : robot.stick1.getZ();
		
		//get gyro angle
		double t = robot.gyro.getAngle();
		
		//update which drive mode the robot is in
		mode = setDriveMode();
		
		/*if(robot.xButton.changed()) {
			if(robot.xButton.on()) {
				robot.xControl.setSetpoint(1000 * -27);
				robot.xControl.enableLog("zanzibar_spumoni.csv");
				robot.xControl.enable();
			} else {
				robot.xControl.disable();
				robot.xControl.closeLog();
			}
		}*/
		
		//apply drive values to drive the robot
		//robot.robotDrive.mecanumDrive_Cartesian(x, y, z, t);
		robot.robotDrive.mecanumDrive_Cartesian(robot.driveX, robot.driveY, robot.driveR, -t);
	}
	
	//select a drive mode based on button input
	public DriveMode setDriveMode() {
		
		//ROBOT mode, is press-hold, has priority over other modes but reverts to previous mode when released
		if(robot.robotButton.held()) {
			if(mode != DriveMode.ROBOT) {
				prevMode = mode;		//if switching to ROBOT mode, store the previous drive mode
			}
			return DriveMode.ROBOT;
			
		} else if(mode == DriveMode.ROBOT) {
			return prevMode;			//if switching out of ROBOT mode, revert to the previous drive mode
		}
		
		//FIELD mode, is toggle, reverts to the default drive mode when released
		if(robot.fieldButton.changed()) {
			if(robot.fieldButton.on()) {
				return DriveMode.FIELD;
			} else {
				return defaultMode;
			}
		}
		
		//INTAKE mode, is toggle, is the default drive mode
		if(robot.intakeButton.changed()) {
			if(robot.intakeButton.on()) {
				return DriveMode.INTAKE;
			} else {
				return defaultMode;		//returns default drive mode for the sake of it, but since INTAKE is the default this does essentially nothing
			}
		}
		
		//GEAR_OUT mode, is toggle, reverts to the default drive mode when released
		if(robot.gearOutButton.changed()) {
			if(robot.gearOutButton.on()) {
				return DriveMode.GEAR_OUT;
			} else {
				return defaultMode;
			}
		}
		
		//GEAR_IN mode, is toggle, reverts to the default drive mode when released
		if(robot.gearInButton.changed()) {
			if(robot.gearInButton.on()) {
				return DriveMode.GEAR_IN;
			} else {
				return defaultMode;
			}
		}
		
		//if nothing has changed, maintain current drive mode
		return mode;
	}
}
