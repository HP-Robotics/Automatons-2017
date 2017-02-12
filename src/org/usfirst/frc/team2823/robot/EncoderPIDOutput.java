package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class EncoderPIDOutput implements PIDOutput {
	private Robot robot;
	private EncoderThread e;
	private Axis a;
	private double x;
	private double y;
	
	public enum Axis {
		V, X, Y, R
	}
	
	public EncoderPIDOutput(Robot robot, EncoderThread e, Axis a) {
		this.robot = robot;
		this.e = e;
		this.a = a;
	}

	@Override
	public void pidWrite(double output) {
		switch(a) {
			case V:
				double dx = x - e.getX();
				double dy = y - e.getY();
				double t = Math.atan2(dy, dx);
				
				robot.setDriveX(output * Math.sin(t));
				robot.setDriveY(output * -Math.cos(t));
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
	
	public void setTarget(double x, double y) {
		this.x = x;
		this.y = y;
	}
}
