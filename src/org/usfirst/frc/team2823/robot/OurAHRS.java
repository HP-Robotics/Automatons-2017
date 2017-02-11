/* In order to use FRCSIM, there are some classes that have to be stubbed out.
 *   This is one of them.  What we do is have 'production' code in the same
 *   file as the simulation code, and then when it's time to simulate, we just
 *   switch which is which.
 */
package org.usfirst.frc.team2823.robot;

/* Non Simulation code */

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
public class OurAHRS extends AHRS {
	final float kFieldRotation = 90;

	public OurAHRS() {
		super(I2C.Port.kOnboard);
	}
	
	public float getFusedHeading() {
		return super.getFusedHeading() - kFieldRotation;
	}
	
	public float getCompassHeading() {
		return super.getCompassHeading() - kFieldRotation;
	}
	
}


/* Simulation code */
/*
import edu.wpi.first.wpilibj.AnalogGyro;

public class OurAHRS {
	final double kFieldRotation = 90;
	AnalogGyro gyro;
	
	public OurAHRS() {
		gyro = new AnalogGyro(1);
		gyro.reset();
		// TODO Auto-generated constructor stub
		// TODO in real one use I2C.Port.konboard
	}

	public double getFusedHeading() {
		// TODO Auto-generated method stub
		return gyro.getAngle() - kFieldRotation;
	}

	public double getCompassHeading() {
		// TODO Auto-generated method stub
		return 0;
	}

}
*/