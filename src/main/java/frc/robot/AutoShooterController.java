package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoShooterController {

    Servo hoodServo;
    CANSparkMax flywheelNeo;
    TalonSRX turret775;
    XboxController theController;

    String isTracking = "false";
    double turretPos;
    boolean startButtonPressed = false;
    boolean trackingStopped = false;

    CANEncoder flywheelEncoder;
    CANPIDController flywheelPIDController;
    NetworkTable table;
    NetworkTableEntry pipe;
    NetworkTableEntry ledMode;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    double x, y, area, isTarget, modeLed, pipeLine, flywheelRPM;

    Spark RGBController;

    public AutoShooterController(Servo hoodServoRobot, CANSparkMax flywheelNeoRobot, TalonSRX turret775Robot,
            XboxController theControllerRobot, double turretPosRobot, CANPIDController flywheelPIDControllerIN,
            CANEncoder flywheelEncoderIN, double flywheelRPMIN, Spark RGBControllerIN) {
        hoodServo = hoodServoRobot;
        flywheelNeo = flywheelNeoRobot;
        turret775 = turret775Robot;
        theController = theControllerRobot;
        turretPos = turretPosRobot;
        flywheelPIDController = flywheelPIDControllerIN;
        flywheelEncoder = flywheelEncoderIN;
        flywheelRPM = flywheelRPMIN;
        RGBController = RGBControllerIN;
    }

    public void shooterController() {

        if (theController.getAButton()) {
            startButtonPressed = true;
        }
        if (theController.getBButton()) {
            startButtonPressed = false;
        }

        if (startButtonPressed) {
            // tracking&flywheel
            startTrackingFlywheel(pipe, x, y);
        }
        if (!startButtonPressed) {
            // INSERT DEFAULT POSITION HERE
            stopTrackingFlywheel(pipe);
            trackingStopped = true;
        }
    }

    public void LimeLightInit() {
        // get values from limelight
        table = NetworkTableInstance.getDefault().getTable("limelight");
        ledMode = table.getEntry("ledMode");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        pipe = table.getEntry("pipeline");

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);

        // read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        isTarget = tv.getDouble(0.0);
        modeLed = ledMode.getDouble(0.0);
        pipeLine = pipe.getDouble(0);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("IsThereATarget", isTarget);
        SmartDashboard.putNumber("ledMode", modeLed);
        SmartDashboard.putNumber("Pipeline", pipeLine);
        SmartDashboard.putString("LimeLightIsTracking", isTracking);
    }

    public void startTrackingFlywheel(NetworkTableEntry pipe, double x, double y) {
        LimeLightInit();

        // set up PIDcontroller
        flywheelPIDController = flywheelNeo.getPIDController();

        // use encoder to display info about flywheel on smart dashboard
        flywheelEncoder = flywheelNeo.getEncoder();
        SmartDashboard.putNumber("ProcessVariable", flywheelEncoder.getVelocity());
        flywheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);

        pipe.setValue(2);
        isTracking = "true";
        // double autoServoAngle = (((-0.08769747858077814318 * (Math.pow(y, 2))) +
        // (0.75133342164053162904 * y)) + 31.25005372095953504186);
        double autoServoAngle = (0.00138821715630382814 * Math.pow(y, 3)) + (-0.12659498575584260394 * Math.pow(y, 2))
                + ((0.87692723545402051499 * y)) + (31.81212577091250892636);
        if (autoServoAngle < 0) {
            autoServoAngle = 0;
        } else if (autoServoAngle > 65) {
            autoServoAngle = 65;
        }

        turret775.set(ControlMode.Position, (turret775.getSelectedSensorPosition() + (x * 3.75)));
        hoodServo.setAngle(autoServoAngle);

        if ((x > -1 && x < 1) && (y > -1 && y < 1)) {
            // lightColorChanges
            RGBController.set(0.71);
            // System.out.println("====ready to shoot====");
        } else if ((x < -1 || x > 1) && (y < -1 || y > 1)) {
            RGBController.set(0.03);
        }
    }

    public void stopTrackingFlywheel(NetworkTableEntry pipe) {
        LimeLightInit();
        if (trackingStopped) {
            turret775.set(ControlMode.Position, 502);
            hoodServo.setAngle(0);
            RGBController.set(0.61);
            // System.out.println("==tracking stopped==");
            trackingStopped = false;
            flywheelNeo.set(0);
        }

        pipe.setValue(1);
        isTracking = "false";
    }

}