package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class EncoderPIDOutput implements PIDOutput {
	private Robot robot;
	private Axis a;
	private double t;
	
	public enum Axis {
		X, Y, T, R
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
			case X:
				robot.setDriveX(output);
				break;
			case Y:
				robot.setDriveY(output);
				break;
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
