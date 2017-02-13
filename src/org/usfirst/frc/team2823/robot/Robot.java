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
	Joystick driverStick;
	Joystick operatorStick;
	Joystick opponentStick;
	
	Button robotButton;
	Button fieldButton;
	Button intakeButton;
	Button gearOutButton;
	Button gearInButton;
	
	Button xButton;
	Button yButton;
	
	TeleopMode teleopMode;
	ShootAutonomous autonomous;
	TestMode testMode;
	
	RobotDrive robotDrive;
	RobotDrive opponentDrive;
    OurCANTalon shooter;
	OurCANTalon intake;
	OurCANTalon uptake;
	
	OurCANTalon climbMotor1;
	OurCANTalon climbMotor2;
	
	OurCANTalon subShooter;
	
	EncoderThread encoderThread;
	OurAHRS ahrs;
	OurADXRS450_Gyro gyro;
	AnalogGyro opponentGyro;
	
	EncoderPIDSource vSource;
	EncoderPIDSource xSource;
	EncoderPIDSource ySource;
	EncoderPIDSource rSource;
	
	EncoderPIDOutput vOutput;
	EncoderPIDOutput xOutput;
	EncoderPIDOutput yOutput;
	EncoderPIDOutput rOutput;
	
	AdvancedPIDController vControl;
	AdvancedPIDController xControl;
	AdvancedPIDController yControl;
	AdvancedPIDController rControl;
	
	ToggleSwitch intakeState;
	ToggleSwitch shooterState;
	
	//declare constants
	//simulator wheel PWM channels
	final int kFrontLeftChannel = 0;
	final int kRearLeftChannel = 1;
	final int kFrontRightChannel = 2;
	final int kRearRightChannel = 3;
	
	
	final int kOppFrontLeftChannel = 40;
	final int kOppRearLeftChannel = 41;
	final int kOppFrontRightChannel = 42;
	final int kOppRearRightChannel = 43;
	
	final int kShooter1Channel = 0;
	final int kShooter2Channel = 1;
	final int kClimb1Channel = 2;
	final int kClimb2Channel = 3;
	final int kIntakeChannel = 4;
	final int kUptakeChannel = 5;
	
	//joystick zero-sensitivity threshold
	final double kStickThreshold = 0.15;
	
	//intake rotation-sensitivity threshold
	final double kIntakeRotationThreshold = 1.0;
	
	//distance between left and right encoder wheels
	final double kEncoderWheelDistance = 0.4;
	
	//robot initial rotation on field
	final double kFieldRotation = 90;

	// The channels on the driver station that the joysticks connect to
	final int kJoystick1Channel = 0;
	final int kJoystick2Channel = 1;
	final int kJoystickOppChannel = 2;
	
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
    	driverStick = stick1 = new Joystick(kJoystick1Channel);
    	operatorStick = stick2 = new Joystick(kJoystick2Channel);
    	opponentStick = new Joystick(kJoystickOppChannel);
		
		robotButton = new Button();
		fieldButton = new Button();
		intakeButton = new Button();
		gearOutButton = new Button();
		gearInButton = new Button();
    	
		xButton = new Button();
		yButton = new Button();
        intakeState = new ToggleSwitch();
        shooterState = new ToggleSwitch();
		
		teleopMode = new TeleopMode(this);
		autonomous = new ShootAutonomous(this);
        testMode = new TestMode(this);
        SmartDashboard.putBoolean("TestMode", false);
		
        robotDrive = new RobotDrive(kFrontLeftChannel, kRearLeftChannel, kFrontRightChannel, kRearRightChannel);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
		robotDrive.setExpiration(0.1);
		
		opponentDrive = new RobotDrive(kOppFrontLeftChannel, kOppRearLeftChannel, kOppFrontRightChannel, kOppRearRightChannel);
		opponentDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		opponentDrive.setInvertedMotor(MotorType.kRearLeft, true);
		opponentDrive.setExpiration(0.1);
		
        shooter = new OurCANTalon(kShooter2Channel);
        intake = new OurCANTalon(kIntakeChannel);
        uptake = new OurCANTalon(kUptakeChannel);
        
        climbMotor1 = new OurCANTalon(kClimb1Channel);
        climbMotor2 = new OurCANTalon(kClimb2Channel);
        
        subShooter = new OurCANTalon(kShooter1Channel);
        subShooter.absoluteFeedback();
        subShooter.reverseSensor(true);
		
        subShooter.configNominalOutputVoltage(0.0, 0.0);
        subShooter.configPeakOutputVoltage(12.0, 0.0);
        
        subShooter.setProfile(0);
        
        ahrs = new OurAHRS();
        gyro = new OurADXRS450_Gyro();
        opponentGyro = new AnalogGyro(40);
        
		encoderThread = new EncoderThread(this);
		encoderThread.start();
		
		vSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.V);
		xSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.X);
		ySource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.Y);
		rSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.R);
		
		vOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.V);
		xOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.X);
		yOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.Y);
		rOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.R);
		
		//old 0.0045, 0.000001, 0.35
		vControl = new AdvancedPIDController(0.006, 0.000001, 0.2, vSource, vOutput, 0.01);
		xControl = new AdvancedPIDController(0.004, 0.000001, 0.4, xSource, xOutput, 0.01);
		yControl = new AdvancedPIDController(0.004, 0.0001, 0.4, ySource, yOutput, 0.01);
		rControl = new AdvancedPIDController(1.0, 0.0001, 0.4, rSource, rOutput, 0.01);
        
        SmartDashboard.putNumber("Shooter", 0.0);
        SmartDashboard.putNumber("Intake", 0.0);
        SmartDashboard.putNumber("Uptake", 0.0);
        SmartDashboard.putNumber("Climb 1", 0.0);
        SmartDashboard.putNumber("Climb 2", 0.0);
        
        SmartDashboard.putNumber("P", 0.03);
        SmartDashboard.putNumber("I", 0.000006);
        SmartDashboard.putNumber("D", 1.0);
        SmartDashboard.putNumber("F", 0.025);
        SmartDashboard.putNumber("Setpoint", 0.0);
        
        //compressor = new Compressor(0);
        //compressor.setClosedLoopControl(true);
        
        //solenoid1 = new DoubleSolenoid(0,1);

        try{
        	gyro.reset();
        	opponentGyro.reset();
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
    	autonomous.init();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	autonomous.periodic();
    }

    /**
     * This function is called periodically during operator control
     */
    
    @Override
    public void teleopInit() {
    	
    	if(SmartDashboard.getBoolean("TestMode", false)) {
    		testMode.testInit();
    	} else {
    		teleopMode.teleopInit();
    	}
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
		opponentDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
	
	//PID along a straight line to the given x and y values
	public void driveTo(double x, double y) {
		double dx = x - encoderThread.getX();
		double dy = y - encoderThread.getY();
		
		double dp = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		double t = Math.atan(dx / dy);
		
		vSource.setTarget(x, y);
		vOutput.setTarget(x, y);
		vControl.setSetpoint(0);
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
