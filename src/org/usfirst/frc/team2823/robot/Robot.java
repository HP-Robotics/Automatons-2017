
package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

public class Robot extends IterativeRobot {
    Jaguar motor1;
	Jaguar motor2;
	
	Compressor c;
	DoubleSolenoid solenoid1;
	double count = 0;
	boolean solenoidForward = false;
	
	Joystick stick1;
	Joystick stick2;
	boolean yButtonClicked = false;
	boolean yButtonWasClicked = false;
	boolean aButtonClicked = false;
	boolean aButtonWasClicked = false;
	double rampSpeed = 0.0;
	
	AHRS ahrs;
	double navxDriftFromOrigin = 0.0;
	double navxOrigin = 0.0;
	double navx2DriftFromOrigin = 0.0;
	double navx2Origin = 0.0;
	
	ADXRS450_Gyro gyro;
	double gyroDriftFromOrigin = 0.0;
	double gyroOrigin = 0.0;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		stick1 = new Joystick (0);
		stick2 = new Joystick(1);
        motor1 = new Jaguar(0);
        SmartDashboard.putNumber("Speed", 0.0);
        motor2 = new Jaguar(1);
        SmartDashboard.putBoolean("M2", false);
        
        ahrs = new AHRS(I2C.Port.kOnboard);
    	gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
        
        c = new Compressor(0);
        c.setClosedLoopControl(true);
        
        solenoid1 = new DoubleSolenoid(0,1);
        
        count = Timer.getFPGATimestamp();
        
        try{
    		gyro.reset();
    	}catch(Exception e) {
    		System.out.println("Gyro not work");
    	}
        rampSpeed = 0.0;
        navxOrigin = ahrs.getFusedHeading();
        navx2Origin = ahrs.getCompassHeading();
        gyroOrigin = gyro.getAngle();
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	double lSpeed = stick1.getRawAxis(1);
    	//double rSpeed = -stick2.getRawAxis(1);
    	//motor1.set(lSpeed);
    	//System.out.println("Speed: " + lSpeed);
    	//motor2.set(rSpeed);
    	//System.out.println("NavX: "+ ahrs.getAngle());
    	navxDriftFromOrigin = Math.abs(ahrs.getFusedHeading()-navxOrigin);
    	navx2DriftFromOrigin =  Math.abs(ahrs.getCompassHeading()-navx2Origin);
    	System.out.println("Navx Fused Origin Drift: " + navxDriftFromOrigin);
    	System.out.println("Navx Compass Origin Drift: " + navx2DriftFromOrigin);
    	//System.out.println("Gyro: " + gyro.getAngle());
    	gyroDriftFromOrigin = Math.abs(gyro.getAngle()-gyroOrigin);
    	System.out.println("Gyro Origin Drift: " + gyroDriftFromOrigin);
    	
    	yButtonClicked = stick1.getRawButton(4);
    	aButtonClicked = stick1.getRawButton(2);
    	if(yButtonClicked){
    		yButtonWasClicked = true;
    	}
    	if(aButtonClicked){
    		aButtonWasClicked = true;
    	}
    	if(yButtonWasClicked && rampSpeed < 1.0 && !yButtonClicked){
    		rampSpeed = (rampSpeed + 0.05);
    		yButtonWasClicked = false;
    	}else if(aButtonWasClicked && rampSpeed > -1.0 && !aButtonClicked){
    		rampSpeed = (rampSpeed - 0.05);
    		aButtonWasClicked = false;
    	}   	
    	motor1.set(rampSpeed);
    	System.out.println(rampSpeed);
    	//System.out.println("Gyro: " + gyro.getAngle());
    	/*if(SmartDashboard.getBoolean("M2")){
    		//motor2.set(0.5);
    		//solenoid1.set(DoubleSolenoid.Value.kReverse);
    		if((Timer.getFPGATimestamp() - count) >= 0.25) {
    			if(solenoidForward) {
    				solenoid1.set(DoubleSolenoid.Value.kForward);
    			} else {
    				solenoid1.set(DoubleSolenoid.Value.kReverse);
    			}
    		
    			solenoidForward = !solenoidForward;
    			count = Timer.getFPGATimestamp();
    		}
    	}
    	else{
    		//motor2.set(0);
    		//solenoid1.set(DoubleSolenoid.Value.kForward);
    		solenoid1.set(DoubleSolenoid.Value.kOff);

    	}*/
    	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }   
}