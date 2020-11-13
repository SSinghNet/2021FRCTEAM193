package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

public class Shooter {

    Servo hoodServo;
    TalonSRX turret775;
    CANEncoder flywheelEncoder;
    CANPIDController flywheelPIDController;
    CANSparkMax flywheelNeo1;
    CANSparkMax flywheelNeo2;
    XboxController eeController;

    double hoodServoAngle;
    double turretPos;
    int flywheelRPM;
    double flywheelSpeed;

    public Shooter(Servo hoodServoIN, TalonSRX turret775IN, CANEncoder flywheelEncoderIN, CANPIDController flywheelPIDControllerIN, CANSparkMax flywheelNeo1IN, XboxController eeControllerIN, int flywheelRPMIN, double flywheelSpeedIN, double turretPosRobot) {
        hoodServo = hoodServoIN;
        turret775 = turret775IN;
        flywheelEncoder = flywheelEncoderIN;
        flywheelPIDController = flywheelPIDControllerIN;
        flywheelNeo1 = flywheelNeo1IN;
        eeController = eeControllerIN;
        flywheelRPM = flywheelRPMIN;
        flywheelSpeed = flywheelSpeedIN;
        turretPos = turretPosRobot;
    }

	//controls hoodServo and turrert775 with DPad
    public void ManualControl() {

        if (eeController.getPOV() != -1) {
            
            //control hoodServo with DPad up and right
            hoodServoAngle = hoodServo.getAngle();
            if (eeController.getPOV() == 0 && hoodServoAngle < 65) {
                hoodServoAngle += 5;
            } if (eeController.getPOV() == 180 && hoodServoAngle > 0) {
                hoodServoAngle -= 5;
            }
            hoodServo.setAngle(hoodServoAngle);

            //control turret775 with DPad left and right
            //need to use position control for turret
            if (eeController.getPOV() == 270) {
                turretPos+=2;
                turret775.set(ControlMode.Position, turretPos);
            } if (eeController.getPOV() == 90) {
                turretPos-=2;
                turret775.set(ControlMode.Position, turretPos);
            }
        } else if (eeController.getPOV() == -1) {
            turret775.set(ControlMode.PercentOutput, 0);
        }

        System.out.println("hoodServo.getAngle() = " + hoodServo.getAngle() );
        System.out.println("hoodServoAngle = " + (hoodServoAngle * 1.5));
    }

    //set neo2 to follow neo1
    public void FlywheelFollower() {
        flywheelNeo2.follow(CANSparkMax.ExternalFollower.kFollowerSparkMax, 9, true);
    }
    
    //simple flywheel control with A and B buttons
    public void FlywheelSimple() {
        
        if (eeController.getAButton() == true) {
            flywheelNeo1.set(flywheelRPM);
        } if (eeController.getBButton() == true) {
            flywheelNeo1.set(0);
        }
    }

    //flywheel velocity control (measured in RPM)
    public void FlywheelVelocityControl() {
        
        //set up PIDcontroller
        flywheelPIDController = flywheelNeo1.getPIDController();
        
        //use encoder to display info about flywheel on smart dashboard
        flywheelEncoder = flywheelNeo1.getEncoder();
        SmartDashboard.putNumber("ProcessVariable", flywheelEncoder.getVelocity());

        
        if (eeController.getStartButton() == true) {
            flywheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);
        } else if (eeController.getBackButton() == true) {
            flywheelNeo1.set(0);
        }
    }
}