package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class OurCANTalon implements PIDOutput {

	public OurCANTalon(int i) {
		// TODO Auto-generated constructor stub
	}

	public void reverseSensor(boolean b) {
		// TODO Auto-generated method stub
		
	}

	public void configNominalOutputVoltage(double d, double e) {
		// TODO Auto-generated method stub
		
	}

	public void configPeakOutputVoltage(double d, double e) {
		// TODO Auto-generated method stub
		
	}

	public void setProfile(int i) {
		// TODO Auto-generated method stub
		
	}

	public double getEncPosition() {
		// TODO Auto-generated method stub
		return 0;
	}

	public void setP(double number) {
		// TODO Auto-generated method stub
		
	}

	public void setI(double number) {
		// TODO Auto-generated method stub
		
	}

	public void setD(double number) {
		// TODO Auto-generated method stub
		
	}

	public void set(double number) {
		// TODO Auto-generated method stub
		
	}

	public double getSpeed() {
		// TODO Auto-generated method stub
		return 0;
	}

	public double getClosedLoopError() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}
	
	public void speedMode() {
		// TODO In the real one set TalonControlMode.Speed
	}
	
	public void normalMode() {
		// TODO In the real one set TalonControlMode.PercentVbus
	}

	public void relativeFeedback() {
	    //TODO In real one do setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	
	}
	
	public void absoluteFeedback() {
	    //TODO In real one do setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);	
	}

}
