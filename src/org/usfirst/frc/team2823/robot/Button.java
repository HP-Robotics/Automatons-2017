package org.usfirst.frc.team2823.robot;

public class Button {
	private boolean s = false;	//stands for 'state', true if the Button is pressed
	private boolean ls = false;	//stands for 'last state', stores the previous state of the Button
	private boolean c = false;	//stands for 'changed', true if the Button's previous state does not match its current state
	
	//check if the Button is pressed
	public boolean on() {
		return s;
	}
	
	//check if the button has changed
	public boolean changed() {
		return c;
	}
	
	//update the button, should be called periodically
	public void update(boolean b) {
		
		if(b && (b != ls)) {
			s = !s;
			c = true;
			
		} else {
			c = false;
		}
		
		ls = b;
	}
	
	//reset all values
	public void reset() {
		s = false;
		ls = false;
		c = false;
	}

}
