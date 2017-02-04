package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderVectorPIDSource implements PIDSource {
	private EncoderThread e;
	private Axis a;
	private double t;
	
	public enum Axis {
		T, R
	}
	
	public EncoderVectorPIDSource(EncoderThread e, Axis a, double t) {
		this.e = e;
		this.a = a;
		this.t = t;
	}
	
	@Override
	public double pidGet() {
		switch(a) {
		case T:
			double dx = Math.sin(t);
			double dy = Math.cos(t);
			
			return (dx * e.getX()) + (dy * e.getY());
		case R:
			return e.getR();
		default:
			return 0.0;
		}
	}
	
	public void setDirection(double t) {
		this.t = t;
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