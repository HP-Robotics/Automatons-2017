/* In order to use FRCSIM, there are some classes that have to be stubbed out.
 *   This is one of them.  What we do is have 'production' code in the same
 *   file as the simulation code, and then when it's time to simulate, we just
 *   switch which is which.
 */
package org.usfirst.frc.team2823.robot;

/* Non simulation code */

import edu.wpi.first.wpilibj.VictorSP;

public class OurVictorSP extends VictorSP {

	public OurVictorSP(int channel) {
		super(channel);
	}
	
}


/* Simulation code */
/*
public class OurVictorSP {

	public OurVictorSP(int i) {
		// TODO Auto-generated constructor stub
	}

	public void set(double number) {
		// TODO Auto-generated method stub
		
	}

}
*/