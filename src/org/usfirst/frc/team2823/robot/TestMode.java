package org.usfirst.frc.team2823.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	  Robot robot;
	  private File m_f;
	  private BufferedWriter m_bw;
	  private FileWriter m_fw;
	  
	  private boolean m_logEnabled = false;
	  
	  public void enableLog(String file) {
		  if(m_logEnabled){
			  return;
		  }
		  try {
	  		m_f = new File("home/lvuser/" + file);
	  		
	  		if(!m_f.exists()) {
	  			m_f.createNewFile();
	  		}
	  		m_fw = new FileWriter(m_f);
	  		
	  	} catch(IOException e) {
	  		e.printStackTrace();
	  	}
	  	
	  	m_bw = new BufferedWriter(m_fw);
	  	
	  	try {
	  		m_bw.write("Timestamp, Speed, Error \n" );
	  		
	  		m_logEnabled = true;
	  		
	  	} catch(IOException e) {
	  		e.printStackTrace();
	  	}
		  
	  }
	  
	  public void closeLog() {
		  
		
		if(m_logEnabled){
			  try {
				  m_bw.close();
				  m_fw.close();
				  
			  } catch(IOException e) {
				  
			  }
		 }
		m_logEnabled = false;
	  }
	
	public TestMode (Robot robot){
		this.robot = robot;
	}
	
	public void testInit(){
		SmartDashboard.putNumber("Somethin", 0);
	}
	
	public void testPeriodic(){
		
		robot.talon.setP(SmartDashboard.getNumber("P", 0.0));
		robot.talon.setI(SmartDashboard.getNumber("I", 0.0));
		robot.talon.setD(SmartDashboard.getNumber("D", 0.0));
		
		robot.intakeState.update(robot.stick1.getRawButton(1));
    	robot.shooterState.update(robot.stick1.getRawButton(2));
    	
    	//System.out.println("pos: " + shooter.getEncPosition() + " enc-rate: " + shooter.getEncVelocity() + " pid-rate: " + shooterEncoderSource.pidGet());
    	robot.shooterEncoderSource.pidGet();
    	
    	robot.shooterControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
    	
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

    		if(robot.shooterState.on()){
	    		robot.talon.speedMode();
	    		robot.talon.set(SmartDashboard.getNumber("Setpoint", 0.0));
	    		enableLog("Shooter.csv");
	    		if(m_logEnabled) {
	    	    	  try{
	    	    		  m_bw.write(Timer.getFPGATimestamp() + ", " + robot.talon.getSpeed() + ", " + robot.talon.getClosedLoopError() + "\n");

	    	    	  } catch(IOException e) {
	    	    		  System.out.println("PID logging died on us");
	    	    		  m_logEnabled = false;
	    	    	  }
	    	      }
	    		
    			}else{
        		robot.talon.normalMode();
        		robot.talon.set(0);
        		closeLog();
        	}
    		
    		//
    		//shooter.enableControl();
    	
	}
}
