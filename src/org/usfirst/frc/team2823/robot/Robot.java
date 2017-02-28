package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
	Button gearButton;
	Button shootTrigger;
	Button gyroResetButton1;
	Button gyroResetButton2;
	Button climbButton;
	Button farShotButton;
	Button nearShotButton;
	
	TeleopMode teleopMode;
	TestMode testMode;
	SendableChooser autonomousChooser;
	
	RobotDrive robotDrive;
	//RobotDrive opponentDrive;
    OurCANTalon bottomShooter;
	OurCANTalon beltFeed;
	OurCANTalon intake;
	OurCANTalon uptake;
	
	OurCANTalon climbMotor1;
	OurCANTalon climbMotor2;
	
	OurCANTalon topShooter;
	
	//Compressor compressor;
	DoubleSolenoid shooterSolenoid;
	
	EncoderThread encoderThread;
	
	OurAHRS ahrs;
	OurADXRS450_Gyro gyro;
	//AnalogGyro opponentGyro;
	
	//EncoderPIDSource vSource;
	EncoderPIDSource xSource;
	EncoderPIDSource ySource;
	GyroPIDSource rSource;
	
	//EncoderPIDOutput vOutput;
	EncoderPIDOutput xOutput;
	EncoderPIDOutput yOutput;
	GyroPIDOutput rOutput;
	
	//AdvancedPIDController vControl;
	AdvancedPIDController xControl;
	AdvancedPIDController yControl;
	AdvancedPIDController rControl;
	
	ToggleSwitch intakeState;
	ToggleSwitch shooterState;
	
	CSVLogger log;
	
	DriverStation driverStation;
	
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
	
	final int TOP_SHOOTER_CHANNEL = 11;
	final int BOTTOM_SHOOTER_CHANNEL = 12;
	final int INTAKE_CHANNEL = 1;
	final int CLIMB1_CHANNEL = 2;
	final int CLIMB2_CHANNEL = 3;
	final int BELT_FEED_CHANNEL = 4;
	final int UPTAKE_CHANNEL = 13;
	
	//joystick zero-sensitivity threshold
	final double STICKTHRESHOLD = 0.15;
	final double ROTATIONTHRESHOLD = 0.25;
	
	//intake rotation-sensitivity threshold
	final double INTAKE_ROTATION_THRESHOLD = 0.5;
	
	//distance between left and right encoder wheels
	//final double ENCODER_WHEEL_DISTANCE = 0.4;	//simulator value in meters
	final double ENCODER_WHEEL_DISTANCE = 21.53125;	//real value in inches
	
	//fudge factors to account for encoder imprecisions, this might not be needed for good carpet
	//final double FORWARD_FUDGE_FACTOR = 1.03471;
	
	//shooter speeds
	final double CLOSE_SHOT_SPEED = 3650;
	final double FAR_SHOT_SPEED = 4175;
	
	//robot initial rotation on field
	final double FIELD_ROTATION = 90;

	// The channels on the driver station that the joysticks connect to
	final int JOYSTICK1_CHANNEL = 0;
	final int JOYSTICK2_CHANNEL = 1;
	final int JOYSTICKOPP_CHANNEL = 2;
	
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
	final double MAX_ROTATIONAL_ACCEL = 1450;
	
	//final double FORWARD_KA = 0.0526;		//simulator
	//final double FORWARD_KV = 0.2083;		//simulator
	
	final double FORWARD_KA = 0.0078908 * 0.65;	//fudge factor from Jeremy's office
	final double FORWARD_KV = 0.01;
	
	//final double DIAGONAL_KA = 0.0505;
	//final double DIAGONAL_KV = 0.2823;
	
	//final double SIDE_KA = 0.1111;
	//final double SIDE_KV = 0.5556;
	
	final double ROTATIONAL_KA = 0.00068966;
	final double ROTATIONAL_KV = 0.0037037;
	
	//unit conversion constants
	final double DEG_TO_RAD = Math.PI / 180.0;
	final double RAD_TO_DEG = 180.0 / Math.PI;
	
	final double WHEEL_RADIUS_IN = 1.181;
	final double WHEEL_RADIUS_M = 0.03;
	
	final double ENC_TO_IN = (2.0 * WHEEL_RADIUS_IN * Math.PI) / 2048.0;
	final double IN_TO_ENC = 2048.0 / (2.0 * WHEEL_RADIUS_IN * Math.PI);
	
	final double ENC_TO_M = (2.0 * WHEEL_RADIUS_M * Math.PI) / 2048.0;
	final double M_TO_ENC = 2048.0 / (2.0 * WHEEL_RADIUS_M * Math.PI);
	
	final double RIGHT_LIFT_ANGLE = -120;
	final double MIDDLE_LIFT_ANGLE = -90;
	final double LEFT_LIFT_ANGLE = -60;
	final double GEAR_IN_ANGLE = -30;
	
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
	boolean nearShot = false;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        driverStation = DriverStation.getInstance();
    	if(driverStation.getAlliance() == DriverStation.Alliance.Red){
    		allianceMult = -1;
    	}
        
    	driverStick = stick1 = new Joystick(JOYSTICK1_CHANNEL);
    	operatorStick = stick2 = new Joystick(JOYSTICK2_CHANNEL);
    	//opponentStick = new Joystick(kJoystickOppChannel);
		
		robotButton = new Button();
		fieldButton = new Button();
		intakeButton = new Button();
		gearButton = new Button();
		shootTrigger = new Button();
    	gyroResetButton1 = new Button();
    	gyroResetButton2 = new Button();
		
		climbButton = new Button();
		farShotButton = new Button();
		nearShotButton = new Button();
        intakeState = new ToggleSwitch();
        shooterState = new ToggleSwitch();
	     
		teleopMode = new TeleopMode(this);
        testMode = new TestMode(this);
        SmartDashboard.putBoolean("TestMode", false);
        
		autonomousChooser = new SendableChooser();
		autonomousChooser.addDefault("Empty: Do Nothing", new EmptyAutonomous(this));
		//autonomousChooser.addObject("Cross Baseline", new DriveForwardAutonomous(this));
		autonomousChooser.addObject("Shoot w/o Gear", new ShootAutonomous(this));
		autonomousChooser.addObject("Place Gear", new GearAutonomous(this));
		SmartDashboard.putData("Autonomous Mode", autonomousChooser);
		
        robotDrive = new RobotDrive(FRONT_LEFT_CHANNEL, REAR_LEFT_CHANNEL, FRONT_RIGHT_CHANNEL, REAR_RIGHT_CHANNEL);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
		robotDrive.setExpiration(0.1);
		
		//opponentDrive = new RobotDrive(kOppFrontLeftChannel, kOppRearLeftChannel, kOppFrontRightChannel, kOppRearRightChannel);
		//opponentDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		//opponentDrive.setInvertedMotor(MotorType.kRearLeft, true);
		//opponentDrive.setExpiration(0.1);
		
       
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
        
        //compressor = new Compressor(0);
        //compressor.setClosedLoopControl(true);
        
        shooterSolenoid = new DoubleSolenoid(0, 1);
        shooterSolenoid.set(DoubleSolenoid.Value.kForward);
        
        ahrs = new OurAHRS();
        gyro = new OurADXRS450_Gyro();
        //opponentGyro = new AnalogGyro(40);
        
		encoderThread = new EncoderThread(this);
		encoderThread.reset();
		encoderThread.start();
		
		//vSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.V);
		xSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.X);
		ySource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.Y);
		rSource = new GyroPIDSource(ahrs);
		
		//vOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.V);
		xOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.X);
		yOutput = new EncoderPIDOutput(this, encoderThread, EncoderPIDOutput.Axis.Y);
		rOutput = new GyroPIDOutput(this);
		
		//old 0.0045, 0.000001, 0.35
		//vControl = new AdvancedPIDController(0.2, 0.0006, 0.1, vSource, vOutput, 0.01);
		xControl = new AdvancedPIDController(0.02, 0.000001, 1.0, xSource, xOutput, 0.01);
		yControl = new AdvancedPIDController(0.02, 0.000001, 1.0, ySource, yOutput, 0.01);
		rControl = new AdvancedPIDController(0.025, 0.0002, 0.3, rSource, rOutput, 0.01);		//I should be 0.0002 for small-angle moves
		
		//these should be calculated per-move based on robot rotation
		xControl.setKaKv(FORWARD_KA, FORWARD_KV);
		yControl.setKaKv(FORWARD_KA, FORWARD_KV);
		rControl.setKaKv(ROTATIONAL_KA, ROTATIONAL_KV);
		
        SmartDashboard.putNumber("Shooter", 0.0);
        SmartDashboard.putNumber("Intake", 0.0);
        SmartDashboard.putNumber("Uptake", 0.0);
        SmartDashboard.putNumber("Climb 1", 0.0);
        SmartDashboard.putNumber("Climb 2", 0.0);
        
        SmartDashboard.putNumber("P", 0.025);		//0.02
        SmartDashboard.putNumber("I", 0.0025);	//0.000001
        SmartDashboard.putNumber("D", 0.3);			//1.0
        SmartDashboard.putNumber("F", 0.0);
        
        SmartDashboard.putNumber("Ka", FORWARD_KA);
        SmartDashboard.putNumber("Kv", FORWARD_KV);
        SmartDashboard.putNumber("Max a", MAX_FORWARD_ACCEL * 0.8);
        SmartDashboard.putNumber("Max v", MAX_FORWARD_VEL);
        SmartDashboard.putNumber("KaMult", 0.7);
        SmartDashboard.putNumber("KvMult", 1.0);
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
        
        //use System.getProperty("user.home") to get path to home directory
        //log = new CSVLogger("/tmp");
        log = new CSVLogger("/home/lvuser");
        
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
	public void driveTo_Cartesian(double x, double y, double vm, double am) {
		xSource.reset();
		ySource.reset();
		
		xControl.configureGoal(x, MAX_FORWARD_VEL * vm, MAX_FORWARD_ACCEL * 0.8 * am);
		yControl.configureGoal(y, MAX_FORWARD_VEL * vm, MAX_FORWARD_ACCEL * 0.8 * am);
		
		xControl.enableLog("xControlPID.csv");
		yControl.enableLog("yControlPID.csv");
		
		xControl.enable();
		yControl.enable();
	}
	
	//PID to the given x and y values without applying a constant multiplier
	public void driveTo_Cartesian(double x, double y) {
		driveTo_Cartesian(x, y, 1, 1);
	}
	
	//PID to the given theta (in degrees) using a single rotation PID
	public void rotateTo(double t, double vm, double am) {
		//rControl.setSetpoint(t);
		rControl.reset();
		rControl.configureGoal(t, MAX_ROTATIONAL_VEL * vm, MAX_ROTATIONAL_ACCEL * am);
		rControl.enableLog("rControlPID.csv");
		rControl.enable();
	}
	
	//PID to the given theta without applying a constant multiplier
	public void rotateTo(double t) {
		rotateTo(t, 1, 1);
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
