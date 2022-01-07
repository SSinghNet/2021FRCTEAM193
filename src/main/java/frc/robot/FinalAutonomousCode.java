/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class FinalAutonomousCode {
    TalonFX Falcon1;
    TalonFX Falcon2;
    TalonFX Falcon3;
    TalonFX Falcon4;
    CANSparkMax FlyWheel;
    CANSparkMax FeederNeo;
    CANSparkMax TowerNeo;
    CANSparkMax IntakeNeo;
    DoubleSolenoid DoubleSolenoid1;
    Timer AutoTimer;
    boolean IsMotorActive = true;
    CANEncoder flywheelEncoder;
    CANPIDController flywheelPIDController;
    int flywheelRPM;

    Servo hoodServo;
    CANSparkMax sparkMFlyWheel;
    TalonSRX srxTurret775;
    XboxController theController;
    double turretPos;

    Spark RGBController;

    NetworkTable table;
    NetworkTableEntry pipe;
	NetworkTableEntry ledMode;
	NetworkTableEntry tx;
	NetworkTableEntry ty;
	NetworkTableEntry ta;
	NetworkTableEntry tv;
    double x, y, area, isTarget, modeLed, pipeLine;
    

    public FinalAutonomousCode(TalonFX falcon1, TalonFX falcon2, TalonFX falcon3, TalonFX falcon4, CANSparkMax FlyWheel2, CANSparkMax FeederNeo2, CANSparkMax TowerNeo2, CANSparkMax NeoMotor2, DoubleSolenoid Solenoid2, Timer AutoTimer2, CANEncoder flywheelEncoderIN,  CANPIDController flywheelPIDControllerIN, int flywheelRPMIN, Servo hoodServoIN, CANSparkMax sparkMFlyWheelIN, TalonSRX srxTurret775IN, XboxController theControllerIN, double turretPosIN, Spark RGBControllerIN){
        Falcon1 = falcon1;
        Falcon2 = falcon2;
        Falcon3 = falcon3;
        Falcon4 = falcon4;
        FlyWheel = FlyWheel2;
        FeederNeo = FeederNeo2;
        TowerNeo = TowerNeo2;
        IntakeNeo = NeoMotor2;
        DoubleSolenoid1 = Solenoid2;
        AutoTimer = AutoTimer2;
        flywheelEncoder = flywheelEncoderIN;
        flywheelPIDController = flywheelPIDControllerIN;
        flywheelRPM = flywheelRPMIN;
        hoodServo = hoodServoIN;
        sparkMFlyWheel = sparkMFlyWheelIN;
        srxTurret775 = srxTurret775IN;
        theController = theControllerIN;
        turretPos = turretPosIN;
        RGBController = RGBControllerIN;
    }

    AutoShooterController theAutoShooterController = new AutoShooterController(hoodServo, sparkMFlyWheel, srxTurret775, theController, turretPos, flywheelPIDController, flywheelEncoder, flywheelRPM, RGBController);


    public void AutonomousCodeOne(){
    //does nothing
        Falcon1.set(ControlMode.PercentOutput, 0.0);
        Falcon3.set(ControlMode.PercentOutput, 0.0);
    }
    public void AutonomousCodeTwo(){
    //moves forward
        if (IsMotorActive == true){
        Timer.delay(0.8);
        Falcon1.set(ControlMode.PercentOutput, -.35);
        Falcon3.set(ControlMode.PercentOutput, 0.35);
        Timer.delay(0.7);
        IsMotorActive = false;
        } if (IsMotorActive == false){
        Falcon1.set(ControlMode.PercentOutput, 0.0);
        Falcon3.set(ControlMode.PercentOutput, 0.0);
        }
    }
    public void AutonomousCodeThree(){
    //shoot, move
        if (IsMotorActive == true){
            hoodServo.setAngle(30);

            // FlyWheel.set(0.8);

            flywheelPIDController = FlyWheel.getPIDController();
                
            flywheelEncoder = FlyWheel.getEncoder();
            SmartDashboard.putNumber("ProcessVariable", flywheelEncoder.getVelocity());
            flywheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);


            Timer.delay(1);
            TowerNeo.set(-0.3);
            FeederNeo.set(0.3);
            Timer.delay(6.0);
            FlyWheel.set(0.0);
            TowerNeo.set(0.0);
            FeederNeo.set(0.0);
            Falcon1.set(ControlMode.PercentOutput, -0.35);
            Falcon3.set(ControlMode.PercentOutput, 0.35);
            Timer.delay(0.7);
            IsMotorActive = false;
        } if (IsMotorActive == false){
            Falcon1.set(ControlMode.PercentOutput, 0.0);
            Falcon3.set(ControlMode.PercentOutput, 0.0);
        }
    }
    public void AutonomousCodeFour(){
    //shoot, move, pickup powercells
        if(IsMotorActive == true){
            FlyWheel.set(1);
            //tracking code from sumeet
        TowerNeo.set(-0.3);   
        FeederNeo.set(0.3);     
        Timer.delay(4.0);
        FlyWheel.set(0.0);
        TowerNeo.set(0.0);
        FeederNeo.set(0.0);
        Falcon1.set(ControlMode.PercentOutput, -0.3);
        Falcon3.set(ControlMode.PercentOutput, 0.3);
        Timer.delay(3.0);
        DoubleSolenoid1.set(Value.kForward);
        Timer.delay(1.0);
        IntakeNeo.set(-0.3);
        Timer.delay(1.0);
        IsMotorActive = false;
        }if(IsMotorActive == false){   
        Falcon1.set(ControlMode.PercentOutput, 0.0);
        Falcon3.set(ControlMode.PercentOutput, 0.0);
        }
    }
    public void AutonomousCodeFive(){
        //move, shoot, pickup, shoot
        AutoTimer.start();
        //ask sumeet for tracking code
        FeederNeo.set(0.3);
        IntakeNeo.set(0.6);
        Falcon1.set(ControlMode.PercentOutput, -0.2);
        Falcon3.set(ControlMode.PercentOutput, 0.2);
        System.out.println(AutoTimer.get());
        if (AutoTimer.get() > 5.0){
            Falcon1.set(ControlMode.PercentOutput, 0.0);
            Falcon3.set(ControlMode.PercentOutput, 0.0);
        }
    }

    boolean targetSet = false;
    boolean firstRun = true;

    public void SixBallAuto(){
        
        if(firstRun){
            AutoTimer.reset();
            AutoTimer.start();
            firstRun = false;
        }

        RGBController.set(0.53);
        theAutoShooterController.LimeLightInit();
        System.out.println(AutoTimer.get());


                //get values from limelight 
                table = NetworkTableInstance.getDefault().getTable("limelight");
                ledMode = table.getEntry("ledMode");
                tx = table.getEntry("tx");
                ty = table.getEntry("ty");
                ta = table.getEntry("ta");
                tv = table.getEntry("tv");
                pipe = table.getEntry("pipeline");
                pipe.setValue(2);
        
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
        
                //read values periodically
                x = tx.getDouble(0.0);
                y = ty.getDouble(0.0);
                area = ta.getDouble(0.0);
                isTarget = tv.getDouble(0.0);
                modeLed = ledMode.getDouble(0.0);
                pipeLine = pipe.getDouble(0);
        
                //post to smart dashboard periodically
                SmartDashboard.putNumber("LimelightX", x);
                SmartDashboard.putNumber("LimelightY", y);
                SmartDashboard.putNumber("LimelightArea", area);
                SmartDashboard.putNumber("IsThereATarget", isTarget);
                SmartDashboard.putNumber("ledMode", modeLed);
                SmartDashboard.putNumber("Pipeline", pipeLine);
                

        // //=====================================================================================================================================        
            //double autoServoAngle = (((-0.08769747858077814318 * (Math.pow(y, 2))) + (0.75133342164053162904 * y)) + 31.25005372095953504186);
            double autoServoAngle = (0.00138821715630382814 * Math.pow(y, 3)) + (-0.12659498575584260394 * Math.pow(y, 2)) + ((0.87692723545402051499 * y)) + (31.81212577091250892636);
            if(autoServoAngle < 0){
                autoServoAngle = 0;
            }else if(autoServoAngle > 65){
                autoServoAngle = 65;
            }

            srxTurret775.set(ControlMode.Position, (srxTurret775.getSelectedSensorPosition() + (theAutoShooterController.x *3.75)));         
            hoodServo.setAngle(autoServoAngle);

            if((x > -1 && x < 1) && (y > -1 &&y < 1)){
                //lightColorChanges
                RGBController.set(0.71);
                System.out.println("====ready to shoot====");
            }else if((x < -1 || x > 1) && (y < -1 ||y > 1)){
                RGBController.set(0.03);
            }
        // //=====================================================================================================================================

        if(AutoTimer.get() <= 1.5){
        //     //==================================================================================================================================
                 flywheelPIDController = FlyWheel.getPIDController();
                
        //         //use encoder to display info about flywheel on smart dashboard
               flywheelEncoder = FlyWheel.getEncoder();
                SmartDashboard.putNumber("ProcessVariable", flywheelEncoder.getVelocity());
               flywheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);
            //===================================================================================================================================
        //FlyWheel.set(.8);

            DoubleSolenoid1.set(Value.kForward);    
        } else if(AutoTimer.get() > 1.5 && AutoTimer.get() < 9){
            FeederNeo.set(0.8);
            TowerNeo.set(-0.4);
            IntakeNeo.set(0.8);
            Falcon1.set(ControlMode.PercentOutput, -0.2);
            Falcon3.set(ControlMode.PercentOutput, 0.2);
        } else if(AutoTimer.get() > 9 && AutoTimer.get() < 15){
            Falcon1.set(ControlMode.PercentOutput, 0);
            Falcon3.set(ControlMode.PercentOutput, 0);
        }

    }
}
