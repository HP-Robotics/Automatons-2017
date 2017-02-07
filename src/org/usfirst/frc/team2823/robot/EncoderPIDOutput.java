package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class EncoderPIDOutput implements PIDOutput {
	private Robot robot;
	private Axis a;
	private double t;
	
	public enum Axis {
		V, X, Y, R
	}
	
	public EncoderPIDOutput(Robot robot, Axis a) {
		this.robot = robot;
		this.a = a;
	}
	
	public EncoderPIDOutput(Robot robot, Axis a, double t) {
		this.robot = robot;
		this.a = a;
		this.t = t;
	}

	@Override
	public void pidWrite(double output) {
		switch(a) {
			case V:
				robot.setDriveX(output * Math.sin(t));
				robot.setDriveY(output * Math.cos(t));
				break;
			case X:
				robot.setDriveX(output);
				break;
			case Y:
				robot.setDriveY(output);
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
