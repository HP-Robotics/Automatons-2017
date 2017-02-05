package org.usfirst.frc.team2823.robot;

import org.usfirst.frc.team2823.robot.OurCANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.PIDOutput;

public class OurCANTalon implements PIDOutput {
	
	public enum TalonControlMode {
		Speed,
		PercentVbus
	}
	
	public enum FeedbackDevice {
		AnalogEncoder,
		AnalogPot,
		CtreMagEncoder_Absolute, 
		CtreMagEncoder_Relative,
		EncFalling,
		EncRising,
		PulseWidth, 
		QuadEncoder 
	}

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

	public void changeControlMode(TalonControlMode speed) {
		// TODO Auto-generated method stub
		
	}

	public void setFeedbackDevice(FeedbackDevice ctremagencoderRelative) {
		// TODO Auto-generated method stub
		
	}

}
