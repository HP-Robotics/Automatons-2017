package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class GyroPIDOutput implements PIDOutput {
	private Robot robot;
	
	public GyroPIDOutput(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void pidWrite(double output) {
		robot.setDriveR(output);
	}
}
