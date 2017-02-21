package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Encoder;

public class AverageEncoder {
	Robot robot;
	Encoder l;
	Encoder r;
	
	public AverageEncoder(Robot robot, Encoder l, Encoder r) {
		this.robot = robot;
		this.l = l;
		this.r = r;
	}
	
	public double getDistance() {
		return (l.getDistance() + r.getDistance()) / 2.0 * robot.ENC_TO_IN;
	}
	
	public void reset() {
		l.reset();
		r.reset();
	}
}
