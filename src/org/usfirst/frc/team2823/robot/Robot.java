package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
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
	//Joystick opponentStick;
	
	Button robotButton;
	Button fieldButton;
	Button intakeButton;
	Button gearOutButton;
	Button gearInButton;
	Button shootTrigger;
	Button shooterWheelsButton;
	
	Button xButton;
	Button climbButton;
	
	TeleopMode teleopMode;
	TestMode testMode;
	
	RobotDrive robotDrive;
	//RobotDrive opponentDrive;
    OurCANTalon bottomShooter;
	OurCANTalon intake;
	OurCANTalon uptake;
	
	OurCANTalon climbMotor1;
	OurCANTalon climbMotor2;
	
	OurCANTalon topShooter;
	
	EncoderThread encoderThread;
	OurAHRS ahrs;
	OurADXRS450_Gyro gyro;
	//AnalogGyro opponentGyro;
	
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
	
	CSVLogger log;
	
	SendableChooser autonomousChooser;
	
	//declare constants
	//simulator wheel PWM channels
	final int FRONT_LEFT_CHANNEL = 0;
	final int REAR_LEFT_CHANNEL = 1;
	final int FRONT_RIGHT_CHANNEL = 2;
	final int REAR_RIGHT_CHANNEL = 3;
	
	//final int kOppFrontLeftChannel = 40;
	//final int kOppRearLeftChannel = 41;
	//final int kOppFrontRightChannel = 42;
	//final int kOppRearRightChannel = 43;
	
	final int TOP_SHOOTER_CHANNEL = 0;
	final int BOTTOM_SHOOTER_CHANNEL = 1;
	final int CLIMB1_CHANNEL = 2;
	final int CLIMB2_CHANNEL = 3;
	final int INTAKE_CHANNEL = 4;
	final int UPTAKE_CHANNEL = 5;
	
	//joystick zero-sensitivity threshold
	final double STICKTHRESHOLD = 0.15;
	
	//intake rotation-sensitivity threshold
	final double INTAKE_ROTATION_THRESHOLD = 1.0;
	
	//distance between left and right encoder wheels
	final double ENCODER_WHEEL_DISTANCE = 0.4;
	
	//robot initial rotation on field
	final double FIELD_ROTATION = 90;

	// The channels on the driver station that the joysticks connect to
	final int JOYSTICK1_CHANNEL = 0;
	final int JOYSTICK2_CHANNEL = 1;
	final int JOYSTICKOPP_CHANNEL = 2;
	
	//motion profiling constants
	final double MAX_FORWARD_VEL = 4.8;
	final double MAX_FORWARD_ACCEL = 19.0;
	
	final double MAX_DIAGONAL_VEL = 3.5;	//4.6669?
	final double MAX_DIAGONAL_ACCEL = 19.8;	//0.1071?
	
	final double MAX_SIDE_VEL = 1.8;
	final double MAX_SIDE_ACCEL = 9.0;
	
	final double FORWARD_KA = 0.0526;
	final double FORWARD_KV = 0.2083;
	
	final double DIAGONAL_KA = 0.0505;
	final double DIAGONAL_KV = 0.2823;
	
	final double SIDE_KA = 0.1111;
	final double SIDE_KV = 0.5556;
	
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
    	driverStick = stick1 = new Joystick(JOYSTICK1_CHANNEL);
    	operatorStick = stick2 = new Joystick(JOYSTICK2_CHANNEL);
    	//opponentStick = new Joystick(kJoystickOppChannel);
		
		robotButton = new Button();
		fieldButton = new Button();
		intakeButton = new Button();
		gearOutButton = new Button();
		gearInButton = new Button();
		shootTrigger = new Button();
    	
		shooterWheelsButton = new Button();
		climbButton = new Button();
        intakeState = new ToggleSwitch();
        shooterState = new ToggleSwitch();
		
		teleopMode = new TeleopMode(this);
        testMode = new TestMode(this);
        SmartDashboard.putBoolean("TestMode", false);
        
		autonomousChooser = new SendableChooser();
		autonomousChooser.addDefault("Cross Baseline", new DriveForwardAutonomous(this));
		autonomousChooser.addObject("Shoot w/o Gear", new ShootAutonomous(this));
		SmartDashboard.putData("Autonomous Mode", autonomousChooser);
		
        robotDrive = new RobotDrive(FRONT_LEFT_CHANNEL, REAR_LEFT_CHANNEL, FRONT_RIGHT_CHANNEL, REAR_RIGHT_CHANNEL);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
		robotDrive.setExpiration(0.1);
		
		//opponentDrive = new RobotDrive(kOppFrontLeftChannel, kOppRearLeftChannel, kOppFrontRightChannel, kOppRearRightChannel);
		//opponentDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		//opponentDrive.setInvertedMotor(MotorType.kRearLeft, true);
		//opponentDrive.setExpiration(0.1);
		
       
        intake = new OurCANTalon(INTAKE_CHANNEL);
        uptake = new OurCANTalon(UPTAKE_CHANNEL);
        
        climbMotor1 = new OurCANTalon(CLIMB1_CHANNEL);
        climbMotor2 = new OurCANTalon(CLIMB2_CHANNEL);
        
        topShooter = new OurCANTalon(TOP_SHOOTER_CHANNEL);
        topShooter.absoluteFeedback();
        topShooter.reverseSensor(true); 
		
        topShooter.configNominalOutputVoltage(0.0, 0.0);
        topShooter.configPeakOutputVoltage(12.0, 0.0);
        
        topShooter.setP(0.03);
        topShooter.setI(0.000006);
        topShooter.setD(1.0);
        topShooter.setF(0.025);
        topShooter.setProfile(0);
        
        bottomShooter = new OurCANTalon(BOTTOM_SHOOTER_CHANNEL);
        bottomShooter.absoluteFeedback();
        bottomShooter.reverseSensor(true); 
		
        bottomShooter.configNominalOutputVoltage(0.0, 0.0);
        bottomShooter.configPeakOutputVoltage(12.0, 0.0);
        
        bottomShooter.setP(0.03);
        bottomShooter.setI(0.000006);
        bottomShooter.setD(1.0);
        bottomShooter.setF(0.025);
        bottomShooter.setProfile(0);
        
        ahrs = new OurAHRS();
        gyro = new OurADXRS450_Gyro();
        //opponentGyro = new AnalogGyro(40);
        
		encoderThread = new EncoderThread(this);
		encoderThread.reset();
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
		vControl = new AdvancedPIDController(0.2, 0.0006, 0.1, vSource, vOutput, 0.01);
		xControl = new AdvancedPIDController(1.0, 0.001, 0.1, xSource, xOutput, 0.01);
		yControl = new AdvancedPIDController(1.0, 0.001, 0.1, ySource, yOutput, 0.01);
		rControl = new AdvancedPIDController(1.0, 0.0001, 0.4, rSource, rOutput, 0.01);
		
		//these should be calculated per-move based on robot rotation
		xControl.setKaKv(SIDE_KA, SIDE_KV);
		yControl.setKaKv(FORWARD_KA, FORWARD_KV);
		
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
        
        //use System.getProperty("user.home") to get path to home directory
        log = new CSVLogger("/tmp");
        
        try{
        	gyro.reset();
        	//opponentGyro.reset();
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
    	((Autonomous) autonomousChooser.getSelected()).init();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	((Autonomous) autonomousChooser.getSelected()).periodic();
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
    	log.close();
    	
		vControl.disable();
		xControl.disable();
		yControl.disable();
		rControl.disable();
		
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
		//opponentDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
	
	//PID along a straight line to the given x and y values
	public void driveTo_Vector(double x, double y) {
		double dx = x - encoderThread.getX();
		double dy = y - encoderThread.getY();
		double dp = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		
		vSource.setTarget(x, y);
		vOutput.setTarget(x, y);
		
		//vControl.configureGoal(0, MAX_FORWARD_VEL, MAX_FORWARD_ACCEL);
		vControl.setSetpoint(0);
		vControl.enableLog("vControlPID.csv");
		vControl.enable();
	}
	
	//PID to the given x and y values using two separate PIDs
	public void driveTo_Cartesian(double x, double y) {
		xControl.setSetpoint(x);
		yControl.setSetpoint(y);
		
		xControl.enableLog("xControlPID.csv");
		yControl.enableLog("yControlPID.csv");
		
		xControl.enable();
		yControl.enable();
	}
	
	//PID to the given theta (in radians) using a single rotation PID
	public void rotateTo(double t) {
		rControl.setSetpoint(t);
		rControl.enableLog("rControlPID.csv");
		rControl.enable();
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
