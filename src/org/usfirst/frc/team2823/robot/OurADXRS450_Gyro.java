/* In order to use FRCSIM, there are some classes that have to be stubbed out.
 *   This is one of them.  What we do is have 'production' code in the same
 *   file as the simulation code, and then when it's time to simulate, we just
 *   switch which is which.
 */

package org.usfirst.frc.team2823.robot;

/* Non Simulation code */

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;

public class OurADXRS450_Gyro extends ADXRS450_Gyro {
	final double kFieldRotation = 90;
	
	public OurADXRS450_Gyro() {
		super(Port.kOnboardCS0);
	}
	
	public double getAngle() {
		return super.getAngle() - kFieldRotation;
	}
}


/* Simulation code */
/*
import edu.wpi.first.wpilibj.AnalogGyro;

public class OurADXRS450_Gyro {
	final double kFieldRotation = 90;
	AnalogGyro gyro;
	
	public OurADXRS450_Gyro() {
		gyro = new AnalogGyro(0);
		gyro.reset();
		// TODO Auto-generated constructor stub
		//TODO - in the real one use Port.kOnboardCS0
	}

	public double getAngle() {
		// TODO Auto-generated method stub
		return gyro.getAngle() - kFieldRotation;
	}

	public void reset() {
		gyro.reset();
		// TODO Auto-generated method stub
		
	}
}
*/