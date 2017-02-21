package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderPIDSource implements PIDSource {
	private EncoderThread e;
	private Axis a;
	private double x;
	private double y;
	private double xZero = 0;
	private double yZero = 0;
	
	public enum Axis {
		V, X, Y, R
	}
	
	public EncoderPIDSource(EncoderThread e, Axis a) {
		this.e = e;
		this.a = a;
		
		reset();
	}
	
	@Override
	public double pidGet() {
		switch(a) {
		case V:
			double dx = x - e.getX();
			double dy = y - e.getY();
			return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		case X:
			return e.getX() - xZero;
		case Y:
			return e.getY() - yZero;
		case R:
			return e.getR();
		default:
			return 0.0;
		}
	}
	
	public void reset() {
		xZero = e.getX();
		yZero = e.getY();
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