package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class EncoderThread extends Thread {
	Robot robot;
	Encoder lEncoder;
	Encoder rEncoder;
	Encoder cEncoder;
	
	final double kFieldRotation = 90;
	boolean running = true;
	
	double lf = 0;
	double ls = 0;
	double lt = Timer.getFPGATimestamp();
	
	double x = 0;
	double y = 0;
	double r = 0;
	
	public EncoderThread(Robot robot) {
		this.robot = robot;
		
		lEncoder = new Encoder(0, 1, false, EncodingType.k4X);
		rEncoder = new Encoder(2, 3, false, EncodingType.k4X);
		cEncoder = new Encoder(4, 5, false, EncodingType.k4X);
		
		lEncoder.reset();
		rEncoder.reset();
		cEncoder.reset();
	}
	
	public void run() {
		while(running) {
			try {
				double f;
				double s;
				double t;
				
				synchronized(this) {
					//get encoder travel distances (forward, sideways, and rotation)
					f = (rEncoder.getDistance() + lEncoder.getDistance()) / 2;
					s = cEncoder.getDistance();
					r = (rEncoder.getDistance() - lEncoder.getDistance()) / robot.ENCODER_WHEEL_DISTANCE;
					
					//get current time
					t = Timer.getFPGATimestamp();
				}
				
				//in case the simulated FPGA timestamp is the same as last iteration
				if(t == lt) {
					Timer.delay(0.001);
					continue;
				}
				
				//get forward and sideways travel distances
				double df = (f - lf);
				double ds = (s - ls);
				
				synchronized(this) {
					//convert encoder travel distances to field position
					y += (df * Math.cos(r)) - (ds * Math.sin(r));
					x += (ds * Math.cos(r)) + (df * Math.sin(r));
				}
				
				//store values for next iteration
				lf = f;
				ls = s;
				lt = t;
				
				//wait 1 ms
				Timer.delay(0.001);
				
			} catch(Exception e) {
				e.printStackTrace();
				break;
			}
		}
	}
	
	public void reset() {
		lEncoder.reset();
		rEncoder.reset();
		cEncoder.reset();
	}
	
	public double getX() {
		synchronized(this) {
			return x;
		}
	}
	
	public double getY() {
		synchronized(this) {
			return y;
		}
	}
	
	public double getR() {
		synchronized(this) {
			return r  - (kFieldRotation * Math.PI / 180);
		}
	}
}
