package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	Robot robot;
	
	public TestMode (Robot robot){
		this.robot = robot;
	}
	
	public void testInit(){
		SmartDashboard.putNumber("Somethin", 0);
	}
	
	public void testPeriodic(){
		robot.intakeState.update(robot.stick1.getRawButton(1));
    	robot.shooterState.update(robot.stick1.getRawButton(2));
    	
    	//System.out.println("pos: " + shooter.getEncPosition() + " enc-rate: " + shooter.getEncVelocity() + " pid-rate: " + shooterEncoderSource.pidGet());
    	robot.shooterEncoderSource.pidGet();
    	
    	robot.shooterControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
    	
    	System.out.println("''I've heard that penguins are pretty cool...'' - David Attenborough, 2017.");
    	System.out.println("'Penguins are alright I guess' -Albert Einstein");
    	
    	if(robot.intakeState.changed()){
    		if(robot.intakeState.on()) {
    			//intake.set(-1.0);
    			//uptake.set(-1.0);
    			robot.intake.set(SmartDashboard.getNumber("Intake", 0.0));
    			robot.uptake.set(SmartDashboard.getNumber("Uptake", 0.0));
    			robot.shooter.set(SmartDashboard.getNumber("Shooter", 0.0));
    		}else{
    			robot.intake.set(0.0);
    			robot.uptake.set(0.0);
    			robot.shooter.set(0.0);
    		}
    	}

    	if(robot.shooterState.changed()){
    		//shooter.set(-0.75);
    		if(robot.shooterState.on()){
	    		/*shooterControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0.0));
	    		shooterControl.enableLog("Shooter.csv");*/    		
    			//shooter.set(SmartDashboard.getNumber("Shooter", 0.0));
    			}else{
        		//shooter.disableControl();
        		//shooterControl.reset();
        		//shooterControl.closeLog();
        		//shooter.set(0.0);
        	}
    		
    		//
    		//shooter.enableControl();
    	}
	}
}
