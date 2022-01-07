/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
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
    10 - right shooter
    11 - left shooter
    12 - intake
    13 - feeder
    14 - tower */

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
    FinalDriveTrainCode theDriveTrain = new FinalDriveTrainCode(falcon1, falcon2, falcon3, falcon4, leftJoystick, rightJoystick);
    FinalIntakeCode theIntake = new FinalIntakeCode(theController, thePiston, sparkMIntakeNeo, sparkMTowerNeo, theCompressor, sparkMFeederNeo);
    //TalonMusic talonMusic = new TalonMusic(falcon1, falcon2, falcon3, falcon4, "StarWarsThroneRoom.chrp", rightJoystick);
    Feeder theFeeder = new Feeder(theController, sparkMFeederNeo, sparkMTowerNeo, opticalSensor);
    Shooter theManualShooter = new Shooter(hoodServo, srxTurret775, flywheelEncoder, flywheelPIDController, sparkMFlywheelNeo, theController, flywheelRPM, .8, turretPos);
    FinalAutonomousCode autoCode = new FinalAutonomousCode(falcon1, falcon2, falcon3, falcon4, sparkMFlywheelNeo, sparkMFeederNeo, sparkMTowerNeo, sparkMIntakeNeo, thePiston, autoTimer, flywheelEncoder, flywheelPIDController, flywheelRPM, hoodServo, sparkMFlywheelNeo, srxTurret775, theController, turretPos, RGBController);
    AutoShooterController theAutoShooterController = new AutoShooterController(hoodServo, sparkMFlywheelNeo, srxTurret775, theController, turretPos, flywheelPIDController, flywheelEncoder, flywheelRPM, RGBController);

    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
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
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
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

    //boolean isDriving = false;

    @Override
    public void teleopPeriodic() {

        //turret mid ----    502
        //limelight sees whole thing max - 6'7" -, stops at 5'

        theClimber.RunClimber();
        theIntake.PowerCellArmIntake();
        //theIntake.PowerCellRollerIntake();
        theFeeder.FeederController();
        //theManualShooter.FlywheelSimple();
        theManualShooter.ManualControl();
        theDriveTrain.DriveTrain();
        theAutoShooterController.shooterController();
        theManualShooter.FlywheelVelocityControl();
        theIntake.PowerCellRollerIntakeJoystick();
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

        if(theController.getTriggerAxis(Hand.kRight) > .1){
            srxClimber775_0.set(ControlMode.PercentOutput, theController.getTriggerAxis(Hand.kRight));
        }else{
            srxClimber775_0.set(ControlMode.PercentOutput, 0);
        }

    }
}
