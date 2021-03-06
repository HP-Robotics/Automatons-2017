/* In order to use FRCSIM, there are some classes that have to be stubbed out.
 *   This is one of them.  What we do is have 'production' code in the same
 *   file as the simulation code, and then when it's time to simulate, we just
 *   switch which is which.
 */
package org.usfirst.frc.team2823.robot;

/* Non Simulation code */

import com.ctre.CANTalon;
public class OurCANTalon extends CANTalon {

	public OurCANTalon(int deviceNumber) {
		super(deviceNumber);
	}
	
	public void speedMode() {
		changeControlMode(TalonControlMode.Speed);
	}
	
	public void normalMode() {
		changeControlMode(TalonControlMode.PercentVbus);
	}

	public void relativeFeedback() {
	    setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	
	}
	
	public void absoluteFeedback() {
	    setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);	
	}

	
}


/* Simulation code */
/*
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

	public String getP() {
		// TODO Auto-generated method stub
		return null;
	}

	public String getI() {
		// TODO Auto-generated method stub
		return null;
	}

	public String getD() {
		// TODO Auto-generated method stub
		return null;
	}

	public String getF() {
		// TODO Auto-generated method stub
		return null;
	}

	public double getSetpoint() {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getOutputVoltage() {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getBusVoltage() {
		// TODO Auto-generated method stub
		return 0;
	}

	public void setF(double number) {
		// TODO Auto-generated method stub
		
	}

}
*/