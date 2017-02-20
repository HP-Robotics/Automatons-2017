package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class AverageEncoderOutput implements PIDOutput {
	Robot robot;
	
	public AverageEncoderOutput(Robot robot) {
		this.robot = robot;
	}
	
	@Override
	public void pidWrite(double output) {
		robot.setDriveY(-output);
	}
}
