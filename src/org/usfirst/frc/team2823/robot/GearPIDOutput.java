package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class GearPIDOutput implements PIDOutput {
	private OurVictorSP motor;
	
	public GearPIDOutput(OurVictorSP motor){
		this.motor = motor;
	}
	
	@Override
	public void pidWrite(double output) {
		motor.set(output);
	}
}
