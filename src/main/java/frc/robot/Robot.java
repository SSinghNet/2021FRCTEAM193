
package frc.robot;

import java.nio.file.Path;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Default";
    private static final String kMoveFowardAuto = "Move Foward";
    private static final String kMoveShootAuto = "Move and Shoot";
    private static final String kMoveShootIntake = "Move, Shoot, Intake";
    private static final String kSixBallAuto = "Six Ball Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();



    Joystick rightJoystick = new Joystick(0);
    Joystick leftJoystick = new Joystick(1);
    XboxController theController = new XboxController(3);

    TalonFX falcon1 = new TalonFX(1);
    TalonFX falcon2 = new TalonFX(2);
    TalonFX falcon3 = new TalonFX(3);
    TalonFX falcon4 = new TalonFX(4);

    TalonSRX srxTurret775 = new TalonSRX(5);
    TalonSRX srxClimber775_0 = new TalonSRX(6);
    TalonSRX srxClimber775_1 = new TalonSRX(7);

    // DigitalInput climberLimitSwitch0 = new DigitalInput(0);
    // DigitalInput climberLimitSwitch1 = new DigitalInput(1);
    DigitalInput opticalSensor = new DigitalInput(0);

    Compressor theCompressor = new Compressor(8);
    DoubleSolenoid thePiston = new DoubleSolenoid(8, 0, 1);

    Servo hoodServo = new Servo(0);
    Spark RGBController = new Spark(1);

    /*
     * 10 - right shooter 11 - left shooter 12 - intake 13 - feeder 14 - tower
     */

    CANSparkMax sparkMFlywheelNeo = new CANSparkMax(10, MotorType.kBrushless);
    CANSparkMax sparkMFlywheelNeo1 = new CANSparkMax(11, MotorType.kBrushless);
    CANSparkMax sparkMTowerNeo = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax sparkMIntakeNeo = new CANSparkMax(12, MotorType.kBrushless);
    CANSparkMax sparkMFeederNeo = new CANSparkMax(13, MotorType.kBrushless);

    CANEncoder flywheelEncoder;
    CANPIDController flywheelPIDController;
    double turretPos = srxTurret775.getSelectedSensorPosition();
    Timer autoTimer = new Timer();

    int flywheelRPM = 3800;

    ClimberFinalCode theClimber = new ClimberFinalCode(theController, srxClimber775_0, srxClimber775_1);
    FinalDriveTrainCode theDriveTrain = new FinalDriveTrainCode(falcon1, falcon2, falcon3, falcon4, leftJoystick,
            rightJoystick, theController);
    FinalIntakeCode theIntake = new FinalIntakeCode(theController, thePiston, sparkMIntakeNeo, sparkMTowerNeo,
            theCompressor, sparkMFeederNeo);
    // TalonMusic talonMusic = new TalonMusic(falcon1, falcon2, falcon3, falcon4,
    // "StarWarsThroneRoom.chrp", rightJoystick);
    Feeder theFeeder = new Feeder(theController, sparkMFeederNeo, sparkMTowerNeo, opticalSensor);
    Shooter theManualShooter = new Shooter(hoodServo, srxTurret775, flywheelEncoder, flywheelPIDController,
            sparkMFlywheelNeo, theController, flywheelRPM, .8, turretPos);
    FinalAutonomousCode autoCode = new FinalAutonomousCode(falcon1, falcon2, falcon3, falcon4, sparkMFlywheelNeo,
            sparkMFeederNeo, sparkMTowerNeo, sparkMIntakeNeo, thePiston, autoTimer, flywheelEncoder,
            flywheelPIDController, flywheelRPM, hoodServo, sparkMFlywheelNeo, srxTurret775, theController, turretPos,
            RGBController);
    AutoShooterController theAutoShooterController = new AutoShooterController(hoodServo, sparkMFlywheelNeo,
            srxTurret775, theController, turretPos, flywheelPIDController, flywheelEncoder, flywheelRPM, RGBController);

    //pathweaver
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(new WPI_TalonFX(1), new WPI_TalonFX(2));
    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(new WPI_TalonFX(3), new WPI_TalonFX(4));

    // SIMULATOR
    private final Field2d m_field = new Field2d();
    private Gyro m_gyro = new ADIS16470_IMU();
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    private Encoder m_leftEncoder = new Encoder(5, 6);
    private Encoder m_rightEncoder = new Encoder(7, 8);
    private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    private AnalogGyroSim m_gyroSim = new AnalogGyroSim((AnalogGyro) m_gyro);
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2), // 2 Falcon motors on each side of the drivetrain.
        10.4, // gearing reduction.
        7.5, // MOI of 7.5 kg m^2 (from CAD model).
        60.0, // The mass of the robot is 60 kg.
        3.0, // The robot uses 3" radius wheels.
        0.7112, // The track width is 0.7112 meters.
        null);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default: Do Nothing", kDefaultAuto);
        m_chooser.addOption("Move Foward", kMoveFowardAuto);
        m_chooser.addOption("Move Foward and Shoot", kMoveShootAuto);
        m_chooser.addOption("Move Foward, Shoot, and Intake", kMoveShootIntake);
        m_chooser.addOption("Six Ball Auto", kSixBallAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        LiveWindow.disableAllTelemetry();

        sparkMFlywheelNeo1.follow(sparkMFlywheelNeo, true);
        srxClimber775_1.follow(srxClimber775_0);
        srxClimber775_1.setInverted(true);

        hoodServo.setAngle(0);
        theIntake.startCompressor();

        srxTurret775.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 1);
        srxTurret775.set(ControlMode.Position, 502);

        theAutoShooterController.LimeLightInit();

        String trajectoryJSON = "paths/Test.path";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (Exception e) {
            System.out.println("Unable to open trajectory: " + trajectoryJSON + "\n" + e.getStackTrace());
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        SmartDashboard.putData("Field", m_field);
        m_field.setRobotPose(m_odometry.getPoseMeters());
        m_odometry.update(m_gyro.getRotation2d(),
                    m_leftEncoder.getDistance(),
                    m_rightEncoder.getDistance());
        m_field.setRobotPose(m_odometry.getPoseMeters());
        m_leftEncoder.setDistancePerPulse(1);
        m_rightEncoder.setDistancePerPulse(1);

        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kMoveFowardAuto:
                autoCode.AutonomousCodeTwo();
                break;
            case kMoveShootAuto:
                autoCode.AutonomousCodeThree();
                break;
            case kMoveShootIntake:
                autoCode.AutonomousCodeFive();
                break;
            case kSixBallAuto:
                autoCode.SixBallAuto();
                break;
            case kDefaultAuto:
            default:
                autoCode.AutonomousCodeOne();
                break;
        }
    }

    /**
     * This function is called periodically during operator control.
     */

    // boolean isDriving = false;

    @Override
    public void teleopPeriodic() {

        // turret mid ---- 502
        // limelight sees whole thing max - 6'7" -, stops at 5'

        theClimber.RunClimber();
        theIntake.PowerCellArmIntake();
        // theIntake.PowerCellRollerIntake();
        theFeeder.FeederController();
        // theManualShooter.FlywheelSimple();
        theManualShooter.ManualControl();
        theDriveTrain.DriveTrain();
        theAutoShooterController.shooterController();
        theManualShooter.FlywheelVelocityControl();
        theIntake.PowerCellRollerIntakeJoystick();
    }


    public void simulationPeriodic() {

        m_leftEncoder.setDistancePerPulse(2 * Math.PI * 5 /*Wheel Radius*/ / 5/*EncoderResolution*/);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * 5 /*Wheel Radius*/  / 5/*EncoderResolution*/);

        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.
        if (theController.getRawAxis(1) > 0.5 || theController.getRawAxis(1) < -.5) {
            m_driveSim.setInputs(theController.getRawAxis(1)* -1 , 0);
        } else if (theController.getRawAxis(1) < 0.5 && theController.getRawAxis(1) > -.5) {
            m_driveSim.setInputs(0, 0);
        }
        if (theController.getRawAxis(5) > 0.5 || theController.getRawAxis(5) < -.5) {
            m_driveSim.setInputs(0, theController.getRawAxis(1)* -1);
        } else if(theController.getRawAxis(5) < 0.5 && theController.getRawAxis(5) > -.5)  {
            m_driveSim.setInputs(0, 0);
        }
        

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        m_driveSim.update(0.02);

        // Update all of our sensors.
        m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
    }
    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        theIntake.startCompressor();
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry pipe = table.getEntry("pipeline");

        thePiston.set(Value.kForward);

        pipe.setValue(1);

        if (theController.getTriggerAxis(Hand.kRight) > .1) {
            srxClimber775_0.set(ControlMode.PercentOutput, theController.getTriggerAxis(Hand.kRight));
        } else {
            srxClimber775_0.set(ControlMode.PercentOutput, 0);
        }

    }
}
