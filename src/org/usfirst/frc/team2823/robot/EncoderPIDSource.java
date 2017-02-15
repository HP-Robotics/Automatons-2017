package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderPIDSource implements PIDSource {
	private EncoderThread e;
	private Axis a;
	private double x;
	private double y;
	
	public enum Axis {
		V, X, Y, R
	}
	
	public EncoderPIDSource(EncoderThread e, Axis a) {
		this.e = e;
		this.a = a;
	}
	
	@Override
	public double pidGet() {
		switch(a) {
		case V:
			double dx = x - e.getX();
			double dy = y - e.getY();
			return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		case X:
			return e.getX();
		case Y:
			return e.getY();
		case R:
			return e.getR();
		default:
			return 0.0;
		}
	}
	
	public void setTarget(double x, double y) {
		this.x = x;
		this.y = y;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}		
}