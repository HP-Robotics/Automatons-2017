package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class EncoderThread extends Thread {
	Robot robot;
	Encoder lEncoder;
	Encoder rEncoder;
	Encoder cEncoder;
	
	boolean running = true;
	
	double lf = 0;
	double ls = 0;
	double lt = Timer.getFPGATimestamp();
	
	double x = 0;
	double y = 0;
	double r = 0;
	
	double prevTime = Timer.getFPGATimestamp();
	
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
					r = (rEncoder.getDistance() - lEncoder.getDistance()) / (robot.ENCODER_WHEEL_DISTANCE * robot.IN_TO_ENC);
					
					//get current time
					t = Timer.getFPGATimestamp();
					
					//skip calculation if the simulated FPGA timestamp is the same as last iteration
					if(t != lt) {
						
						//get forward and sideways travel distances
						double df = (f - lf);
						double ds = (s - ls);
						
						//convert encoder travel distances to field position
						x += (ds * Math.cos(r)) + (df * Math.sin(r));
						y += (df * Math.cos(r)) - (ds * Math.sin(r));
						
						//store values for next iteration
						lf = f;
						ls = s;
						lt = t;
					}
				}
				
				/*if(Math.abs(Timer.getFPGATimestamp() - prevTime) > 1.0) {
				System.out.println(" x: " + getX() + " y: " + getY() + " r: " + getR());
				prevTime = Timer.getFPGATimestamp();
				}*/
				
				//wait 1 ms
				Timer.delay(0.001);
				
			} catch(Exception e) {
				e.printStackTrace();
				break;
			}
		}
	}
	
	//reset encoder and position variables
	public void reset() {
		synchronized(this) {
			lf = 0;
			ls = 0;
			lt = Timer.getFPGATimestamp();
			
			x = 0;
			y = 0;
			r = 0;
			
			
			lEncoder.reset();
			rEncoder.reset();
			cEncoder.reset();
		}
	}
	
	public double getLDistance() {
		synchronized(this) {
			return lEncoder.getDistance() * robot.ENC_TO_IN;
		}
	}
	
	public double getRDistance() {
		synchronized(this) {
			return rEncoder.getDistance() * robot.ENC_TO_IN;
		}
	}
	
	public double getCDistance() {
		synchronized(this) {
			return cEncoder.getDistance() * robot.ENC_TO_IN;
		}
	}
	
	public double getX() {
		synchronized(this) {
			return -x * robot.ENC_TO_IN;
		}
	}
	
	public double getY() {
		synchronized(this) {
			return -y * robot.ENC_TO_IN;
		}
	}
	
	public double getR() {
		synchronized(this) {
			return -r * robot.RAD_TO_DEG;
		}
	}
}
