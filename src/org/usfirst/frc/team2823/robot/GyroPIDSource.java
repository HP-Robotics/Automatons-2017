package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class GyroPIDSource implements PIDSource {
	private OurADXRS450_Gyro g;
	private OurAHRS a;
	private boolean isGyro;
	
	public GyroPIDSource(OurADXRS450_Gyro g) {
		this.g = g;
		this.isGyro = true;
	}
	
	public GyroPIDSource(OurAHRS a) {
		this.a = a;
		this.isGyro = false;
	}
	
	@Override
	public double pidGet() {
		if(isGyro) {
			return g.getAngle();
		} else {
			return a.getFusedHeading();
		}
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