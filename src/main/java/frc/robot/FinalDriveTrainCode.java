/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class FinalDriveTrainCode{
    TalonFX Falcon1;
    TalonFX Falcon2;
    TalonFX Falcon3;
    TalonFX Falcon4;
    Joystick LeftJoystick;
    Joystick RightJoystick;
    
    public FinalDriveTrainCode(TalonFX falconn1, TalonFX falconn2, TalonFX falconn3, TalonFX falconn4, Joystick LeftJoystick1, Joystick RightJoystick1){
        Falcon1 = falconn1;
        Falcon2 = falconn2;
        Falcon3 = falconn3;
        Falcon4 = falconn4;
        LeftJoystick = LeftJoystick1;
        RightJoystick = RightJoystick1;    
    }

    public void DriveTrain(){
        //if left/right joystick trigger pressed, robot moves at given speed
        // Falcon2.follow(Falcon1);
        // Falcon4.follow(Falcon3);

        // Falcon1.set(ControlMode.PercentOutput, RightJoystick.getY());
        // Falcon3.set(ControlMode.PercentOutput, -LeftJoystick.getY());

        
        if(RightJoystick.getY() > 0.5 || RightJoystick.getY() < -0.5){
            Falcon1.set(ControlMode.PercentOutput, RightJoystick.getY());

        }else{
            Falcon1.set(ControlMode.PercentOutput, 0);
        }

        if(LeftJoystick.getY() > 0.5 || LeftJoystick.getY() < -.5){
            Falcon3.set(ControlMode.PercentOutput, -LeftJoystick.getY());
        }else{
            Falcon3.set(ControlMode.PercentOutput, 0);
        }

        
    }
}
