package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;

public class TeleopMode {
	Robot robot;
	
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
		
		if(robot.xButton.changed()) {
			if(robot.xButton.on()) {
				robot.xControl.setSetpoint(1000 * -27);
				robot.xControl.enableLog("zanzibar_spumoni.csv");
				robot.xControl.enable();
			} else {
				robot.xControl.disable();
				robot.xControl.closeLog();
			}
		}
		
		//apply drive values to drive the robot
		robot.robotDrive.mecanumDrive_Cartesian(x, y, z, t);
		//robot.robotDrive.mecanumDrive_Cartesian(robot.driveX, robot.driveY, robot.driveR, -t);
	}
}
