package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class EncoderVectorPIDOutput implements PIDOutput {
	private Robot robot;
	private Axis a;
	private double t;
	
	public enum Axis {
		T, R
	}
	
	public EncoderVectorPIDOutput(Robot robot, Axis a, double t) {
		this.robot = robot;
		this.a = a;
		this.t = t;
	}

	@Override
	public void pidWrite(double output) {
		switch(a) {
			case T:
				robot.setDriveX(output * Math.sin(t));
				robot.setDriveY(output * Math.cos(t));
				break;
			case R:
				robot.setDriveR(output);
				break;
		}
	}
	
	public void setDirection(double t) {
		this.t = t;
	}
}
