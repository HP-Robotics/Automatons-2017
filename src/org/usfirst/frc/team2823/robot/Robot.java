package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import org.usfirst.frc.team2823.robot.OurCANTalon;
import org.usfirst.frc.team2823.robot.OurADXRS450_Gyro;


public class Robot extends IterativeRobot {
	//declare objects
	Joystick stick1;
	Joystick stick2;
	
	Button robotButton;
	Button fieldButton;
	Button intakeButton;
	Button gearOutButton;
	Button gearInButton;
	
	Button xButton;
	
	TeleopMode teleopMode;
	TestMode testMode;
	
	RobotDrive robotDrive;
    OurCANTalon shooter;
	OurVictorSP intake;
	OurVictorSP uptake;
	
	OurCANTalon talon;
	
	EncoderThread encoderThread;
	OurAHRS ahrs;
	OurADXRS450_Gyro gyro;
	
	EncoderPIDSource vSource;
	EncoderPIDSource xSource;
	EncoderPIDSource ySource;
	EncoderPIDSource rSource;
	CANTalonPIDSource shooterEncoderSource;
	
	EncoderPIDOutput vOutput;
	EncoderPIDOutput xOutput;
	EncoderPIDOutput yOutput;
	EncoderPIDOutput rOutput;
	
	AdvancedPIDController vControl;
	AdvancedPIDController xControl;
	AdvancedPIDController yControl;
	AdvancedPIDController rControl;
	AdvancedPIDController shooterControl;
	
	ToggleSwitch intakeState;
	ToggleSwitch shooterState;
	
	//declare constants
	//simulator wheel PWM channels
	final int kFrontLeftChannel = 0;
	final int kRearLeftChannel = 1;
	final int kFrontRightChannel = 2;
	final int kRearRightChannel = 3;
	
	//joystick zero-sensitivity threshold
	final double kStickThreshold = 0.15;
	
	//intake rotation-sensitivity threshold
	final double kIntakeRotationThreshold = 1.0;

	// The channels on the driver station that the joysticks connect to
	final int kJoystick1Channel = 0;
	final int kJoystick2Channel = 1;
	
	//declare variables
	//drive values
	double driveX;
	double driveY;
	double driveR;
	double driveT;
	
	double initTime;
	
	//gyro values
	double navxOrigin;
	double navx2Origin;
	double gyroOrigin;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	stick1 = new Joystick(kJoystick1Channel);
    	stick2 = new Joystick(kJoystick2Channel);
		
		robotButton = new Button();
		fieldButton = new Button();
		intakeButton = new Button();
		gearOutButton = new Button();
		gearInButton = new Button();
    	
		xButton = new Button();
        intakeState = new ToggleSwitch();
        shooterState = new ToggleSwitch();
		
		teleopMode = new TeleopMode(this);
        testMode = new TestMode(this);
        SmartDashboard.putBoolean("TestMode", false);
		
		robotDrive = new RobotDrive(kFrontLeftChannel, kRearLeftChannel, kFrontRightChannel, kRearRightChannel);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
		robotDrive.setExpiration(0.1);
		
        shooter = new OurCANTalon(1);
        intake = new OurVictorSP(4);
        uptake = new OurVictorSP(5);
        
        talon = new OurCANTalon(0);
        talon.relativeFeedback();
        talon.reverseSensor(false);
		
        talon.configNominalOutputVoltage(0.0, 0.0);
        talon.configPeakOutputVoltage(12.0, 0.0);
        
        talon.setProfile(0);
        
		encoderThread = new EncoderThread(this);
		encoderThread.start();
		
		vSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.V, encoderThread.getR());
		xSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.X);
		ySource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.Y);
		rSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.R);
		
		vOutput = new EncoderPIDOutput(this, EncoderPIDOutput.Axis.V, encoderThread.getR());
		xOutput = new EncoderPIDOutput(this, EncoderPIDOutput.Axis.X);
		yOutput = new EncoderPIDOutput(this, EncoderPIDOutput.Axis.Y);
		rOutput = new EncoderPIDOutput(this, EncoderPIDOutput.Axis.R);
		
		//old 0.0045, 0.000001, 0.35
		vControl = new AdvancedPIDController(0.004, 0.000001, 0.4, vSource, vOutput, 0.01);
		xControl = new AdvancedPIDController(0.004, 0.000001, 0.4, xSource, xOutput, 0.01);
		yControl = new AdvancedPIDController(0.004, 0.000001, 0.4, ySource, yOutput, 0.01);
		rControl = new AdvancedPIDController(0.002, 0.000001, 0.5, rSource, rOutput, 0.01);
        
        //shooter.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        shooter.absoluteFeedback();
        //shooter.configEncoderCodesPerRev(1);
        
        shooterEncoderSource = new CANTalonPIDSource(shooter);
        shooterControl = new AdvancedPIDController(0.0, 0.0, 0.0, shooterEncoderSource, shooter, 0.01);
        shooterControl.setOutputRange(0.0, 1.0);
        
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
        
        ahrs = new OurAHRS();
        gyro = new OurADXRS450_Gyro();
        
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
    
    @Override
    public void teleopInit() {
    	
    }
    
    @Override
    public void teleopPeriodic() {
    			
    	if(SmartDashboard.getBoolean("TestMode", false)){
    		testMode.testPeriodic();
    	} else {
    		teleopMode.teleopPeriodic();
    	}
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    @Override
	public void disabledInit() {
		vControl.disable();
		xControl.disable();
		yControl.disable();
		rControl.disable();
		
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
	
	//PID along a straight line to the given x and y values
	public void driveTo(double x, double y) {
		double dx = x - encoderThread.getX();
		double dy = y - encoderThread.getY();
		
		double dp = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		double t = Math.atan(dx / dy);
		
		vSource.setDirection(t);
		vOutput.setDirection(t);
		vControl.setSetpoint(dp * -27);
	}
	
	//get drive values for use in autonomous and teleop
	public double getDriveX() {
		return driveX;
	}
	
	public double getDriveY() {
		return driveY;
	}
	
	public double getDriveR() {
		return driveR;
	}
	
	public double getDriveT() {
		return driveT;
	}
	
	//set drive values based on PIDs and joysticks
	public void setDriveX(double x) {
		driveX = x;
	}
	
	public void setDriveY(double y) {
		driveY = y;
	}
	
	public void setDriveR(double r) {
		driveR = r;
	}
	
	public void setDriveT(double t) {
		driveT = t;
	}
}
