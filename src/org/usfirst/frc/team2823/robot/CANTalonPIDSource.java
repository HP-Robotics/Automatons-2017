package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;

public class CANTalonPIDSource implements PIDSource {
	
	OurCANTalon talon;
	double lastTime = Timer.getFPGATimestamp();
	double lastPosition;
	
	public CANTalonPIDSource (OurCANTalon talon){
		this.talon = talon;
		
		lastPosition = talon.getEncPosition();
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		
		return PIDSourceType.kRate;
	}


	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		double currentTime = Timer.getFPGATimestamp();
		double currentPosition = talon.getEncPosition();
		
		
		
		if(currentTime == lastTime) {
			return 0.0;
		}
		
		if(lastPosition < currentPosition) {
			return 0.0;
		}
		
		double velocity = (lastPosition - currentPosition)/(currentTime - lastTime);
		
		lastTime = currentTime;
		lastPosition = currentPosition;
		
		return Math.abs(velocity);
	}

}
