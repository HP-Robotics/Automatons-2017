package org.usfirst.frc.team2823.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import org.usfirst.frc.team2823.robot.OurCANTalon;
import org.opencv.core.Size;
import org.usfirst.frc.team2823.robot.OurADXRS450_Gyro;


public class Robot extends IterativeRobot {
	//declare objects
	Joystick stick1;
	Joystick stick2;
	Joystick driverStick;
	Joystick operatorStick;
	//Joystick opponentStick;
	
	Button robotButton;
	Button robotIntakeButton;
	Button fieldButton;
	Button intakeButton;
	Button gearButton;
	Button shootTrigger;
	Button gyroResetButton1;
	Button gyroResetButton2;
	Button climbButton;
	Button reverseBeltButton;
	Button farShotButton;
	Button nearShotButton;
	Button gearKickButton;
	
	TeleopMode teleopMode;
	TestMode testMode;
	SendableChooser<String> allianceChooser;
	SendableChooser<Autonomous> autonomousChooser;
	
	RobotDrive robotDrive;
	//RobotDrive opponentDrive;
	
	OurCANTalon beltFeed;
	OurCANTalon intake;
	OurCANTalon uptake;
	
	OurCANTalon climbMotor1;
	OurCANTalon climbMotor2;
	
	OurCANTalon topShooter;
    OurCANTalon bottomShooter;
    
    OurVictorSP gearMotor;
	
	Servo leftServo;
	Servo rightServo;
	
	//Compressor compressor;
	DoubleSolenoid shooterSolenoid;
	
	EncoderThread encoderThread;
	
	OurAHRS navx;
	OurADXRS450_Gyro gyro;
	//AnalogGyro opponentGyro;
	Encoder gearEncoder;
	
	//EncoderPIDSource vSource;
	EncoderPIDSource xSource;
	EncoderPIDSource ySource;
	GyroPIDSource rSource;
	GyroPIDSource rMotionSource;
	GearPIDSource gearSource;
	
	//EncoderPIDOutput vOutput;
	EncoderPIDOutput xOutput;
	EncoderPIDOutput yOutput;
	GyroPIDOutput rOutput;
	GyroPIDOutput rMotionOutput;
	GearPIDOutput gearOutput;
	
	//AdvancedPIDController vControl;
	AdvancedPIDController xControl;
	AdvancedPIDController yControl;
	AdvancedPIDController rControl;
	AdvancedPIDController rMotionControl;
	AdvancedPIDController gearControl;
	
	ToggleSwitch intakeState;
	ToggleSwitch shooterState;
	
	CSVLogger log;
	
	//declare constants
	//wheel PWM channels
	final int FRONT_LEFT_CHANNEL = 0;
	final int REAR_LEFT_CHANNEL = 1;
	final int FRONT_RIGHT_CHANNEL = 2;
	final int REAR_RIGHT_CHANNEL = 3;
	
	//shooter servo PWM channels
	final int LEFT_SERVO_CHANNEL = 4;
	final int RIGHT_SERVO_CHANNEL = 5;
	
	//gear kicker PWM channel
	final int GEAR_CHANNEL = 6;
	
	//final int kOppFrontLeftChannel = 40;
	//final int kOppRearLeftChannel = 41;
	//final int kOppFrontRightChannel = 42;
	//final int kOppRearRightChannel = 43;
	
	//talon CAN device IDs
	final int TOP_SHOOTER_CHANNEL = 11;
	final int BOTTOM_SHOOTER_CHANNEL = 12;
	final int INTAKE_CHANNEL = 1;
	final int CLIMB1_CHANNEL = 2;
	final int CLIMB2_CHANNEL = 3;
	final int BELT_FEED_CHANNEL = 4;
	final int UPTAKE_CHANNEL = 13;
	
	final int GEAR_KICK_OUT = -90;
	final int GEAR_KICK_IN = 0;
	
	//joystick zero-sensitivity threshold
	final double STICKTHRESHOLD = 0.15;
	final double ROTATIONTHRESHOLD = 0.3;
	
	//intake rotation-sensitivity threshold
	final double INTAKE_ROTATION_THRESHOLD = 0.5;
	
	//distance between left and right encoder wheels
	//final double ENCODER_WHEEL_DISTANCE = 0.4;	//simulator value in meters
	final double ENCODER_WHEEL_DISTANCE = 21.53125;	//real value in inches
	
	//fudge factors to account for encoder imprecisions, this might not be needed for good carpet
	//final double FORWARD_FUDGE_FACTOR = 1.03471;
	
	//shooter speeds
	final double CLOSE_SHOT_SPEED = 3785;	//old 3800, older old 3650
	final double FAR_SHOT_SPEED = 4480;		//old 4175, older old 4175
	
	//shooter servo positions
	final double CLOSE_SERVO = 0.75;
	final double FAR_SERVO = 0.27;
	
	//shooter solenoid positions
	final Value CLOSE_SOLENOID = Value.kForward;
	final Value FAR_SOLENOID = Value.kReverse;
	
	//robot initial rotation on field
	final double FIELD_ROTATION = 90;
	
	final double BELT_FEED_SPEED = -0.4;

	// The channels on the driver station that the joysticks connect to
	final int JOYSTICK1_CHANNEL = 0;
	final int JOYSTICK2_CHANNEL = 1;
	final int JOYSTICKOPP_CHANNEL = 2;
	
	final double SMALL_I = 0.0002;
	
	//PID controller constants for straight and strafe movements
	final double STRAIGHT_P = 0.02;
	final double STRAIGHT_I = 0.000001;
	final double STRAIGHT_D = 1.0;
	
	final double STRAFE_P = 0.16;
	final double STRAFE_I = 0.001;
	final double STRAFE_D = 2.5;
	
	final double STRAIGHT_WITH_I_P = 0.02;
	final double STRAIGHT_WITH_I_I = 0.0001;
	final double STRAIGHT_WITH_I_D = 1.0;
	
	//motion profiling constants
	//final double MAX_FORWARD_VEL = 4.8;		//simulator
	//final double MAX_FORWARD_ACCEL = 19.0;	//simulator
	
	final double MAX_FORWARD_VEL = 100;
	final double MAX_FORWARD_ACCEL = 126.72986;
	
	//final double MAX_DIAGONAL_VEL = 3.5;	//4.6669?
	//final double MAX_DIAGONAL_ACCEL = 19.8;	//0.1071?
	
	//final double MAX_SIDE_VEL = 1.8;
	//final double MAX_SIDE_ACCEL = 9.0;
	
	final double MAX_ROTATIONAL_VEL = 270;
	final double MAX_ROTATIONAL_ACCEL = (1450 / 2) * 0.75;	//works with the shadow, not necessarily the main
	
	//final double FORWARD_KA = 0.0526;		//simulator
	//final double FORWARD_KV = 0.2083;		//simulator
	
	final double FORWARD_KA = 0.0078908 * 0.65;	//fudge factor from Jeremy's office
	final double FORWARD_KV = 0.01;
	
	//final double DIAGONAL_KA = 0.0505;
	//final double DIAGONAL_KV = 0.2823;
	
	//final double SIDE_KA = 0.1111;
	//final double SIDE_KV = 0.5556;
	
	final double ROTATIONAL_KA = /*(1 / MAX_ROTATIONAL_ACCEL) * 0.7*/ 0.0008;	//works with the shadow, may not work on the main
	final double ROTATIONAL_KV = /*(1 / MAX_ROTATIONAL_VEL) * 0.7*/ 0.00355;	//experimental fudge factor
	
	//unit conversion constants
	final double DEG_TO_RAD = Math.PI / 180.0;
	final double RAD_TO_DEG = 180.0 / Math.PI;
	
	final double WHEEL_RADIUS_IN = 1.181;
	final double WHEEL_RADIUS_M = 0.03;
	
	final double ENC_TO_IN = (2.0 * WHEEL_RADIUS_IN * Math.PI) / 2048.0;
	final double IN_TO_ENC = 2048.0 / (2.0 * WHEEL_RADIUS_IN * Math.PI);
	
	final double ENC_TO_M = (2.0 * WHEEL_RADIUS_M * Math.PI) / 2048.0;
	final double M_TO_ENC = 2048.0 / (2.0 * WHEEL_RADIUS_M * Math.PI);
	
	final double DEG_TO_G_ENC = 280.0 / 360.0;
	final double G_ENC_TO_DEG = 360.0 / 280.0;
	
	//gear mode rotation angles
	final double RIGHT_LIFT_ANGLE = -150;
	final double MIDDLE_LIFT_ANGLE = -90;
	final double LEFT_LIFT_ANGLE = -30;
	
	//declare variables
	//drive values
	double driveX;
	double driveY;
	double driveR;
	double driveT;
	
	double initTime;
	
	double allianceMult = 1;
	
	//gyro values
	double navxOrigin;
	double navx2Origin;
	double gyroOrigin;
	
	boolean resettingGyro = false;
	boolean runningFeeder = false;
	boolean farShot = true;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        allianceChooser = new SendableChooser<String>();
        allianceChooser.addObject("Blue", "1.0");
        allianceChooser.addObject("Red", "-1.0");
        SmartDashboard.putData("AllianceString", allianceChooser);
        
    	driverStick = stick1 = new Joystick(JOYSTICK1_CHANNEL);
    	operatorStick = stick2 = new Joystick(JOYSTICK2_CHANNEL);
    	//opponentStick = new Joystick(kJoystickOppChannel);
		
		robotButton = new Button();
		robotIntakeButton = new Button();
		fieldButton = new Button();
		intakeButton = new Button();
		gearButton = new Button();
		shootTrigger = new Button();
    	gyroResetButton1 = new Button();
    	gyroResetButton2 = new Button();
    	gearKickButton = new Button();
		
		climbButton = new Button();
		reverseBeltButton = new Button();
		farShotButton = new Button();
		nearShotButton = new Button();
        intakeState = new ToggleSwitch();
        shooterState = new ToggleSwitch();
	     
		teleopMode = new TeleopMode(this);
        testMode = new TestMode(this);
        SmartDashboard.putBoolean("TestMode", false);
        
		autonomousChooser = new SendableChooser<Autonomous>();
		autonomousChooser.addDefault("Empty: Do Nothing", new EmptyAutonomous(this));
		autonomousChooser.addObject("Cross Baseline", new DriveForwardAutonomous(this));
		autonomousChooser.addObject("Shoot (Far)", new FarShootAutonomous(this));
		autonomousChooser.addObject("Shoot (Close)", new CloseShootAutonomous(this));
		autonomousChooser.addObject("Place Gear (Left)", new GearAutonomous(this, GearAutonomous.Side.LEFT));
		autonomousChooser.addObject("Place Gear (Center)", new GearAutonomous(this, GearAutonomous.Side.CENTER));
		autonomousChooser.addObject("Place Gear (Right)", new GearAutonomous(this, GearAutonomous.Side.RIGHT));
		SmartDashboard.putData("Autonomous Mode", autonomousChooser);
		
        robotDrive = new RobotDrive(FRONT_LEFT_CHANNEL, REAR_LEFT_CHANNEL, FRONT_RIGHT_CHANNEL, REAR_RIGHT_CHANNEL);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
		robotDrive.setExpiration(0.1);
		
		//opponentDrive = new RobotDrive(kOppFrontLeftChannel, kOppRearLeftChannel, kOppFrontRightChannel, kOppRearRightChannel);
		//opponentDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		//opponentDrive.setInvertedMotor(MotorType.kRearLeft, true);
		//opponentDrive.setExpiration(0.1);
		
		gearMotor = new OurVictorSP(GEAR_CHANNEL);
       
        beltFeed = new OurCANTalon(BELT_FEED_CHANNEL);
        uptake = new OurCANTalon(UPTAKE_CHANNEL);
        intake = new OurCANTalon(INTAKE_CHANNEL);
        
        climbMotor1 = new OurCANTalon(CLIMB1_CHANNEL);
        climbMotor2 = new OurCANTalon(CLIMB2_CHANNEL);
        
        climbMotor1.enableBrakeMode(true);
        climbMotor2.enableBrakeMode(true);
        
        topShooter = new OurCANTalon(TOP_SHOOTER_CHANNEL);
        topShooter.absoluteFeedback();
        topShooter.reverseSensor(true); 
		
        topShooter.configNominalOutputVoltage(0.0, 0.0);
        topShooter.configPeakOutputVoltage(0.0, -12.0);
        
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
        
        leftServo = new Servo(LEFT_SERVO_CHANNEL);
        rightServo = new Servo(RIGHT_SERVO_CHANNEL);
        
        leftServo.setBounds(1.8, 0, 1.5, 0, 1.2);	//ms
        rightServo.setBounds(1.8, 0, 1.5, 0, 1.2);	//ms
        
        //compressor = new Compressor(0);
        //compressor.setClosedLoopControl(true);
        
        shooterSolenoid = new DoubleSolenoid(0, 1);
        shooterSolenoid.set(DoubleSolenoid.Value.kForward);
        
        navx = new OurAHRS();
        gyro = new OurADXRS450_Gyro();
        //opponentGyro = new AnalogGyro(40);
        gearEncoder = new Encoder(6, 7, false, EncodingType.k4X);
        
		encoderThread = new EncoderThread(this);
		encoderThread.reset();
		encoderThread.start();
		
		//vSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.V);
		xSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.X);
		ySource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.Y);
		rSource = new GyroPIDSource(navx);
		rMotionSource = new GyroPIDSource(navx);
		gearSource = new GearPIDSource(gearEncoder);
		
		//vOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.V);
		xOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.X);
		yOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.Y);
		rOutput = new GyroPIDOutput(this);
		rMotionOutput = new GyroPIDOutput(this);
		gearOutput = new GearPIDOutput(gearMotor);
		
		//old 0.0045, 0.000001, 0.35
		//vControl = new AdvancedPIDController(0.2, 0.0006, 0.1, vSource, vOutput, 0.01);
		xControl = new AdvancedPIDController(STRAFE_P, STRAFE_I, STRAFE_D, xSource, xOutput, 0.01);
		yControl = new AdvancedPIDController(STRAIGHT_P, STRAIGHT_I, STRAIGHT_D, ySource, yOutput, 0.01);	//I of 0.0001 works better for step PID, but I of 0.000001 works for motion profiling
		rControl = new AdvancedPIDController(0.03, 0.0002, 0.3, rSource, rOutput, 0.01);		//I should be 0.0002 for small-angle moves
		rMotionControl = new AdvancedPIDController(0.04, 0.0002, 1.2, rMotionSource, rMotionOutput, 0.01);
		
		gearControl = new AdvancedPIDController(0.05, 0.0, 0.0, gearSource, gearOutput, 0.01);
		
		//these should be calculated per-move based on robot rotation
		xControl.setKaKv(FORWARD_KA, FORWARD_KV);
		yControl.setKaKv(FORWARD_KA, FORWARD_KV);
		rControl.setKaKv(ROTATIONAL_KA, ROTATIONAL_KV);
		rMotionControl.setKaKv(ROTATIONAL_KA, ROTATIONAL_KV);
		
        SmartDashboard.putNumber("Shooter", 0.0);
        SmartDashboard.putNumber("Intake", 0.0);
        SmartDashboard.putNumber("Uptake", 0.0);
        SmartDashboard.putNumber("Climb 1", 0.0);
        SmartDashboard.putNumber("Climb 2", 0.0);
        
        SmartDashboard.putNumber("P", 0.04);		//0.02
        SmartDashboard.putNumber("I", 0.0001);		//0.000001
        SmartDashboard.putNumber("D", 1.0);			//1.0
        SmartDashboard.putNumber("F", 0.0);
        
        SmartDashboard.putNumber("Ka", FORWARD_KA);
        SmartDashboard.putNumber("Kv", FORWARD_KV);
        SmartDashboard.putNumber("Max a", MAX_FORWARD_ACCEL * 0.8);
        SmartDashboard.putNumber("Max v", MAX_FORWARD_VEL);
        SmartDashboard.putNumber("KaMult", 1.0);
        SmartDashboard.putNumber("KvMult", 1.0);
        
        SmartDashboard.putNumber("XSetpoint", 0.0);
        SmartDashboard.putNumber("YSetpoint", 0.0);
        SmartDashboard.putNumber("RSetpoint", 0.0);
        SmartDashboard.putNumber("Setpoint", 0.0);
        
        //production smartdashboard things
        SmartDashboard.putBoolean("Robot Mode", false);
        SmartDashboard.putBoolean("Intake Mode", false);
        SmartDashboard.putBoolean("Field Mode", false);
        SmartDashboard.putBoolean("Gear Mode", false);
        SmartDashboard.putBoolean("Climber", false);
        SmartDashboard.putBoolean("Intake", false);
        SmartDashboard.putNumber("Trim: ", 0.0);
        SmartDashboard.putNumber("X: ", 0.0);
        SmartDashboard.putNumber("Y: ", 0.0);
        SmartDashboard.putNumber("Y: ", 0.0);
        SmartDashboard.putNumber("L Distance: ", 0.0);
        SmartDashboard.putNumber("R Distance: ", 0.0);
        SmartDashboard.putNumber("C Distance: ", 0.0);
        
        SmartDashboard.putNumber("Speed", 0.0);
        
        //use System.getProperty("user.home") to get path to home directory
        //log = new CSVLogger("/tmp");
        log = new CSVLogger("/home/lvuser");
        
        //wait up to 20 seconds for the navx to calibrate
        double initTime = Timer.getFPGATimestamp();
        while(navx.isCalibrating() && Math.abs(Timer.getFPGATimestamp() - initTime) <= 20)
        	;	//do nothing
        
        System.out.println("calibration took: " + Math.abs(Timer.getFPGATimestamp() - initTime));
        
        //reset gyros
        try{
        	navx.reset();
        	gyro.reset();
        	//opponentGyro.reset();
        }catch(Exception e) {
        	System.out.println("Gyro not work");
        }
        
        //wait one second after resetting navx to instantiate cameras
        initTime = Timer.getFPGATimestamp();
        while(Math.abs(Timer.getFPGATimestamp() - initTime) <= 1)
        	;	//do nothing
        
        //create camera objects
		//CameraServer c = CameraServer.getInstance();
		//c.addAxisCamera("10.28.23.11");
		//c.startAutomaticCapture(1);
        
        UsbCamera c = CameraServer.getInstance().startAutomaticCapture();
        c.setResolution(640, 480);
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
    	
		//vControl.disable();
		xControl.disable();
		yControl.disable();
		rControl.disable();
		
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
		//opponentDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
	
	//PID along a straight line to the given x and y values
	/*public void driveTo_Vector(double x, double y) {
		double dx = x - encoderThread.getX();
		double dy = y - encoderThread.getY();
		double dp = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		
		vSource.setTarget(x, y);
		vOutput.setTarget(x, y);
		
		//vControl.configureGoal(0, MAX_FORWARD_VEL, MAX_FORWARD_ACCEL);
		vControl.setSetpoint(0);
		vControl.enableLog("vControlPID.csv");
		vControl.enable();
	}*/
	
	//PID to the given x and y values using two separate PIDs
	public void driveTo_Cartesian(double x, double y, double vm, double am, String xfile, String yfile) {
		xSource.reset();
		ySource.reset();
		
		xControl.configureGoal(x, MAX_FORWARD_VEL * vm, MAX_FORWARD_ACCEL * 0.8 * am);
		yControl.configureGoal(y, MAX_FORWARD_VEL * vm, MAX_FORWARD_ACCEL * 0.8 * am);
		
		xControl.enableLog(xfile);
		yControl.enableLog(yfile);
		
		xControl.enable();
		yControl.enable();
	}
	
	//PID to the given x and y values without applying a constant multiplier
	public void driveTo_Cartesian(double x, double y, String xfile, String yfile) {
		driveTo_Cartesian(x, y, 1, 0.8, xfile, yfile);
	}
	
	public void driveTo_Cartesian(double x, double y, double vm, double am) {
		driveTo_Cartesian(x, y, vm, am, "xControlPID.csv", "yControlPID.csv");
	}
	
	public void driveTo_Cartesian(double x, double y) {
		driveTo_Cartesian(x, y, 1, 0.8, "xControlPID.csv", "yControlPID.csv");
	}
	
	//PID to the given theta (in degrees) using a single rotation PID
	public void rotateTo(double t, String file) {
		//rControl.setSetpoint(t);
		rControl.reset();
		//rControl.configureGoal(t, MAX_ROTATIONAL_VEL * vm, MAX_ROTATIONAL_ACCEL * am);
		rControl.setSetpoint(t);
		
		/*if(Math.abs(t - ahrs.getAngle()) < 30) {
			rControl.setPID(rControl.getP(), SMALL_I, rControl.getD());
		} else {
			rControl.setPID(rControl.getP(), 0.0, rControl.getD());
		}*/
		
		rControl.enableLog(file);
		rControl.enable();
	}
	
	public void rotateTo(double t) {
		rotateTo(t, "rControlPID.csv");
	}
	
	//PID to the given relative theta (in degrees) using a single PID
	public void rotateTo_Relative(double t, double vm, double am, String file) {
		rMotionControl.reset();
		rMotionSource.reset();
		
		rMotionControl.configureGoal(t, MAX_ROTATIONAL_VEL * vm, MAX_ROTATIONAL_ACCEL * am, true);
		
		rMotionControl.enableLog(file);
		rMotionControl.enable();
	}
	
	//PID to the given theta without applying a constant multiplier
	public void rotateTo_Relative(double t, String file) {
		rotateTo_Relative(t, 0.9, 0.8, file);
	}
	
	public void rotateTo_Relative(double t, double vm, double am) {
		rotateTo_Relative(t, vm, am, "rMotionControlPID.csv");
	}
	
	public void rotateTo_Relative(double t) {
		rotateTo_Relative(t, 0.9, 0.8, "rMotionControlPID.csv");
	}
	
	//find the closest equivalent angle
	public double getCousin(double current, double target){
		double c = current % 360;
		double t = (target + 360) % 360;
		double adjust = t-c;
		
		if(Math.abs(adjust) > 180){
			if(adjust > 0){
				adjust -= 360;
			} else{
				adjust += 360;
			}
		}
		
		return current + adjust;
		
	}
	
	//configure PID controllers differently for X and Y movements
	public void configureStrafe(AdvancedPIDController c) {
		c.setPID(STRAFE_P, STRAFE_I, STRAFE_D);
	}
	
	public void configureStraight(AdvancedPIDController c) {
		c.setPID(STRAIGHT_P, STRAIGHT_I, STRAIGHT_D);
	}
	
	public void configureStraightWithI(AdvancedPIDController c) {
		c.setPID(STRAIGHT_WITH_I_P, STRAIGHT_WITH_I_I, STRAIGHT_WITH_I_D);
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
