
package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends IterativeRobot {
    CANTalon shooter;
	VictorSP intake;
	VictorSP uptake;
	
	AdvancedPIDController shooterControl;
	CANTalonPIDSource shooterEncoderSource;
	
	ToggleSwitch intakeState;
	ToggleSwitch shooterState;
	
	Compressor compressor;
	DoubleSolenoid solenoid1;
	
	Joystick stick1;
	
	AHRS ahrs;
	ADXRS450_Gyro gyro;
	
	double navxOrigin;
	double navx2Origin;
	double gyroOrigin;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		stick1 = new Joystick (0);
		
        shooter = new CANTalon(0);
        intake = new VictorSP(4);
        uptake = new VictorSP(5);
        
        //shooter.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        shooter.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Absolute);
        //shooter.configEncoderCodesPerRev(1);
        
        shooterEncoderSource = new CANTalonPIDSource(shooter);
        shooterControl = new AdvancedPIDController(0.0, 0.0, 0.0, shooterEncoderSource, shooter, 0.01);
        shooterControl.setOutputRange(0.0, 1.0);
        
        intakeState = new ToggleSwitch();
        shooterState = new ToggleSwitch();
        
        SmartDashboard.putNumber("Shooter", 0.0);
        SmartDashboard.putNumber("Intake", 0.0);
        SmartDashboard.putNumber("Uptake", 0.0);
        
        SmartDashboard.putNumber("P", 0.0);
        SmartDashboard.putNumber("I", 0.0);
        SmartDashboard.putNumber("D", 0.0);
        SmartDashboard.putNumber("Setpoint", 0.0);
        
        //shooter.changeControlMode(CANTalon.TalonControlMode.Speed);
        //shooter.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        //shooter.setPID(1, 1, 1);
        
        ahrs = new AHRS(I2C.Port.kOnboard);
    	gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
        
        //compressor = new Compressor(0);
        //compressor.setClosedLoopControl(true);
        
        //solenoid1 = new DoubleSolenoid(0,1);
        
        try{
    		gyro.reset();
    	}catch(Exception e) {
    		System.out.println("Gyro not work");
    	}
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
    	intakeState.update(stick1.getRawButton(1));
    	shooterState.update(stick1.getRawButton(2));
    	//System.out.println("pos: " + shooter.getEncPosition() + " enc-rate: " + shooter.getEncVelocity() + " pid-rate: " + shooterEncoderSource.pidGet());
    	shooterEncoderSource.pidGet();
    	
    	shooterControl.setPID(SmartDashboard.getNumber("P", 0.0), SmartDashboard.getNumber("I", 0.0), SmartDashboard.getNumber("D", 0.0));
    	
    	if(intakeState.changed()){
    		if(intakeState.on()) {
    			//intake.set(-1.0);
    			//uptake.set(-1.0);
    			intake.set(SmartDashboard.getNumber("Intake", 0.0));
    			uptake.set(SmartDashboard.getNumber("Uptake", 0.0));
    			shooter.set(SmartDashboard.getNumber("Shooter", 0.0));
    		}else{
    			intake.set(0.0);
    			uptake.set(0.0);
    			shooter.set(0.0);
    		}
    	}

    	if(shooterState.changed()){
    		//shooter.set(-0.75);
    		if(shooterState.on()){
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
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }   
}