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
	  		m_bw.write("Timestamp, Speed, Setpoint, Error, P, I, D, F \n" );
	  		
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
		
	}
	
	public void testPeriodic(){
		
		robot.subShooter.setP(SmartDashboard.getNumber("P", 0.0));
		robot.subShooter.setI(SmartDashboard.getNumber("I", 0.0));
		robot.subShooter.setD(SmartDashboard.getNumber("D", 0.0));
		robot.subShooter.setF(SmartDashboard.getNumber("F", 0.0));
		
		robot.intakeState.update(robot.stick1.getRawButton(1));
    	robot.shooterState.update(robot.stick1.getRawButton(2));
    	
    	//System.out.println("pos: " + shooter.getEncPosition() + " enc-rate: " + shooter.getEncVelocity() + " pid-rate: " + shooterEncoderSource.pidGet());
    	robot.shooterEncoderSource.pidGet();
    	
    	robot.shooterControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
    	
    		if(robot.intakeState.on()) {
    			robot.climbMotor1.set(SmartDashboard.getNumber("Climb 1", 0.0));
    			robot.climbMotor2.set(SmartDashboard.getNumber("Climb 2", 0.0));
        		robot.uptake.set(SmartDashboard.getNumber("Uptake", 0.0));
        		robot.intake.set(SmartDashboard.getNumber("Intake", 0.0));
    		}else{
    			robot.intake.set(0.0);
	    		robot.uptake.set(0.0);
    			robot.climbMotor1.set(0.0);
    			robot.climbMotor2.set(0.0);
    		}

    		if(robot.shooterState.on()){
	    		robot.subShooter.speedMode();
	    		robot.subShooter.set(SmartDashboard.getNumber("Setpoint", 0.0));
	    		System.out.println("Shooting");
	    		enableLog("Shooter.csv");
	    		if(m_logEnabled) {
	    	    	  try{
	    	    		  m_bw.write(Timer.getFPGATimestamp() + ", " + robot.subShooter.getSpeed() +  ", " + robot.subShooter.getSetpoint() +", " + (robot.subShooter.getClosedLoopError()/(4096.0/600.0))+ ", ");
	    	    		  m_bw.write(robot.subShooter.getP()+ ", "+ robot.subShooter.getI()+", "+ robot.subShooter.getD()+", "+ robot.subShooter.getF()+"\n");
	    	    	  } catch(IOException e) {
	    	    		  System.out.println("PID logging died on us");
	    	    		  m_logEnabled = false;
	    	    	  }
	    	      }

    			}else{
        		robot.subShooter.normalMode();
        		robot.subShooter.set(SmartDashboard.getNumber("Shooter",0.0));
        		closeLog();
        	}
    		System.out.println("Speed: "+ robot.subShooter.getSpeed() + ", Division: "+ robot.subShooter.getOutputVoltage()/robot.subShooter.getBusVoltage());
    		//shooter.enableControl();
    	
	}
}
