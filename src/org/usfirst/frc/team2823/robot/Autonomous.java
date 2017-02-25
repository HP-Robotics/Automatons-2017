package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;

public class Autonomous {
	Robot robot;
	Timer timer;
	double initTime;
	int stage = 0;
	StageDataElement[] stageData;
	
	class StageDataElement {
		double timeout;
		boolean entered;
	}
	
	public Autonomous(Robot robot) {
		this.robot = robot;
	}
	
	/** Java has a dynamic array class called ArrayList, would it be better to use this over the static-length Array? **/
	public void setStageTimeouts(double[] t) {
		stageData = new StageDataElement[t.length];
		
		for(int i = 0; i < t.length; i++) {
			stageData[i] = new StageDataElement();
			
			stageData[i].timeout = t[i];
			stageData[i].entered = false;
		}
	}
	
	public void start() {
		robot.ahrs.reset();
		robot.encoderThread.reset();
		
		robot.rControl.setPID(robot.rControl.getP(), 0.0002, robot.rControl.getD());
		
		stage = 0;
		initTime = Timer.getFPGATimestamp();
		
		timer = new Timer();
		timer.reset();
		timer.start();
	}
	
	public boolean checkStageTimeout() {
		if (stage < 0 || stage >= stageData.length) {
			robot.robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			
			return true;
		}
	
		if (timer.get() > stageData[stage].timeout) {

			System.out.printf("stage %d timed out\n", stage);
			nextStage();
			return true;
		}
		return false;
	}
	
	public void nextStage() {
		System.out.printf("Stage Finished: %d\tTime: %f\tTotal Time: %f\n",stage,timer.get(),Timer.getFPGATimestamp() - initTime);
		timer.reset();
		stage++;
		
		if(stage >= stageData.length) {
			end();
		}
	}
	
	public void end() {
		System.out.println("-----");
		System.out.printf("Auto Finished:\tTotal Time: %f\n",Timer.getFPGATimestamp() - initTime);
		
		//robot.vControl.reset();
		robot.xControl.reset();
		robot.yControl.reset();
		robot.rControl.reset();
		
		robot.robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
	
	public void init() {
		System.out.println("Override me!");
	}
	
	public void periodic() {
		System.out.println("Override me!");
	}

}
