package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;

public class TeleopMode {
	Robot robot;
	double initTime = Timer.getFPGATimestamp();
	
	public TeleopMode(Robot robot) {
		this.robot = robot;
	}
	
	public void teleopInit() {
		initTime = Timer.getFPGATimestamp();
		
		//xControl.setSetpoint(5000 * -27);
		//yControl.setSetpoint(5000 * -27);
		robot.rControl.setSetpoint(3.14 * -27);
		
		//driveTo(5000, 2500);
		//rControl.setSetpoint(0);
		
		//xControl.enable();
		//yControl.enable();
		//tControl.enable();
		//rControl.enable();
	}
	
	public void teleopPeriodic() {
		//update buttons
		robot.xButton.update(robot.stick1.getRawButton(1));
		
		//prevent joysticks from driving robot when within a threshold value of zero
		double x = Math.abs(robot.stick1.getX()) < robot.kStickThreshold ? 0.0 : robot.stick1.getX();
		double y = Math.abs(robot.stick1.getY()) < robot.kStickThreshold ? 0.0 : robot.stick1.getY();
		double z = Math.abs(robot.stick1.getZ()) < robot.kStickThreshold ? 0.0 : robot.stick1.getZ();
		
		//get gyro angle
		double t = robot.gyro.getAngle();
		
		//update parametric PID setpoints
		//double xPoint = 1000 - Math.pow(20 * (Timer.getFPGATimestamp() - initTime) - Math.sqrt(1000), 2);
		//double xPoint = 2500 / (1 + Math.pow(Math.E, -2 * ((Timer.getFPGATimestamp() - initTime) - 2.5)));
		//double yPoint = (10000 / 5) * (Timer.getFPGATimestamp() - initTime);
		
		//double xPoint = 1000 * Math.sin(0.5 * (Timer.getFPGATimestamp() - initTime));
		//double yPoint = 1000 * Math.cos(0.5 * (Timer.getFPGATimestamp() - initTime));
		
		//xControl.setSetpoint(xPoint * -27);
		//yControl.setSetpoint(yPoint * -27);
		
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
		
		//if(yPoint > 10000) {
		//	yControl.setSetpoint(10000 * -27);
		//}
		
		/*if(xPoint < 0) {
			xControl.setSetpoint(0);
		}*/
		
		//System.out.println(t);
		//System.out.println(driveR + " " + encoderThread.getR());
		//System.out.println(encoderThread.getX() + " " + encoderThread.getY() + " | " + encoderThread.getR());

		//apply drive values to drive the robot
		robot.robotDrive.mecanumDrive_Cartesian(robot.stick1.getX(), robot.stick1.getY(), robot.stick1.getZ(), -t);
		//robot.robotDrive.mecanumDrive_Cartesian(robot.driveX, robot.driveY, robot.driveR, -t);
	}
}
