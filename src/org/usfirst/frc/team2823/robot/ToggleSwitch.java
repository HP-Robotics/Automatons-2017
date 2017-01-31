package org.usfirst.frc.team2823.robot;

public class ToggleSwitch {
	private boolean s = false;
	private boolean ls = false;
	private boolean c = false;
	
	public boolean on() {
		return s;
	}
	
	public boolean changed() {
		return c;
	}
	
	public void update(boolean b) {
		
		if(b && (b != ls)) {
			s = !s;
			c = true;
			
		} else {
			c = false;
		}
		
		ls = b;
	}
	
	public void reset() {
		s = false;
		ls = false;
		c = false;
	}

}
