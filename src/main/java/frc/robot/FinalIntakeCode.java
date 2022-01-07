/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class FinalIntakeCode{
    XboxController eeController;
    DoubleSolenoid DoubleSolenoid1;
    CANSparkMax IntakeMotor;
    CANSparkMax TowerNeo;
    CANSparkMax feederNeo;
    Compressor Compressor1;
    boolean IsPistonActive = true;
    //sarina khera
    public FinalIntakeCode(XboxController eeController2, DoubleSolenoid Solenoid2, CANSparkMax NeoMotor2, CANSparkMax TowerNeo2, Compressor Compressor2, CANSparkMax feederNeoIN){
        eeController = eeController2;
        DoubleSolenoid1 = Solenoid2;
        IntakeMotor = NeoMotor2;
        TowerNeo = TowerNeo2;
        Compressor1 = Compressor2;
        feederNeo = feederNeoIN;
    }

    public void startCompressor(){
        Compressor1.start();
    }

    public void PowerCellRollerIntake(){
        
        //if Y button (yellow button) is pressed, the motor turns on, stays on as long as button is pressed
        //when button not pressed, motor does not turn on
        
        if (eeController.getYButtonPressed()){
            IntakeMotor.set(0.8);
            TowerNeo.set(-0.8);
        }else if (eeController.getYButtonReleased()){
            IntakeMotor.set(0.0);
            TowerNeo.set(0.0);
        }
    }

    public void PowerCellRollerIntakeJoystick(){
        
        if(eeController.getY() >= .1){                                                                      
            //NeoMotor.set(0.8);
            TowerNeo.set(-0.6);

        } else if(eeController.getY() <= -.1){
            //NeoMotor.set(-0.8);
            TowerNeo.set(0.6);
            feederNeo.set(-.8);

        } else if(!eeController.getBumper(Hand.kRight)){
            //NeoMotor.set(0.0);
            TowerNeo.set(0.0);
        }

        if(eeController.getRawAxis(1) >= .1){
            IntakeMotor.set(0.8);
        }else if(eeController.getRawAxis(1) <= -.1){
            IntakeMotor.set(-0.8);
        }else{
            IntakeMotor.set(0);
        }
    }

    public void PowerCellArmIntake(){  
        //if X button (blue button) is pressed once, the piston will go out. if X button is pressed again after that, it will go back in  
        if (IsPistonActive == false){
            if (eeController.getXButtonPressed()){
                DoubleSolenoid1.set(Value.kForward);
                IsPistonActive = true;
            }
        } if (IsPistonActive == true){
            if (eeController.getXButtonPressed()){
                DoubleSolenoid1.set(Value.kReverse);
                IsPistonActive = false;
            }
        }
    }
}


