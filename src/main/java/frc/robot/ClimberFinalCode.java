package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
//Alicia Huynh :)
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

public class ClimberFinalCode{
    //objects
    XboxController eeController;
    TalonSRX climberTalon1;
    TalonSRX climberTalon2;
 
    public ClimberFinalCode(XboxController othereeController, TalonSRX otherClimberTalon1, TalonSRX otherClimberTalon2) {
        eeController = othereeController;
        climberTalon1 = otherClimberTalon1;
        climberTalon2 = otherClimberTalon2;
    }

    public void RunClimber(){
        if(eeController.getTriggerAxis(Hand.kRight) > .1){
            climberTalon1.set(ControlMode.PercentOutput, -eeController.getTriggerAxis(Hand.kRight));
        }else{
            climberTalon1.set(ControlMode.PercentOutput, 0);
        }
    }
}


