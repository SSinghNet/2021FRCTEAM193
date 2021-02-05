/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class FinalDriveTrainCode {
    TalonFX Falcon1;
    TalonFX Falcon2;
    TalonFX Falcon3;
    TalonFX Falcon4;
    Joystick LeftJoystick;
    Joystick RightJoystick;
    XboxController theController;

    public FinalDriveTrainCode(TalonFX falconn1, TalonFX falconn2, TalonFX falconn3, TalonFX falconn4,
            Joystick LeftJoystick1, Joystick RightJoystick1, XboxController theController) {
        Falcon1 = falconn1;
        Falcon2 = falconn2;
        Falcon3 = falconn3;
        Falcon4 = falconn4;
        LeftJoystick = LeftJoystick1;
        RightJoystick = RightJoystick1;
        this.theController = theController;
    }

    public void DriveTrain() {

        // if (RightJoystick.getY() > 0.5 || RightJoystick.getY() < -0.5) {
        // Falcon1.set(ControlMode.PercentOutput, RightJoystick.getY());
        // System.out.println("RightJoystick");
        // } else {
        // Falcon1.set(ControlMode.PercentOutput, 0);
        // }
        // if (LeftJoystick.getY() > 0.5 || LeftJoystick.getY() < -.5) {
        // Falcon3.set(ControlMode.PercentOutput, -LeftJoystick.getY());
        // System.out.println("LeftJoystick");
        // } else {
        // Falcon3.set(ControlMode.PercentOutput, 0);
        // }

        if (theController.getRawAxis(1) > 0.5 || theController.getRawAxis(1) < -.5) {
            Falcon1.set(ControlMode.PercentOutput, -1 * theController.getRawAxis(1));
        } else if (theController.getStickButtonReleased(Hand.kLeft)) {
            Falcon1.set(ControlMode.PercentOutput, 0);
        }
        if (theController.getRawAxis(5) > 0.5 || theController.getRawAxis(5) < -.5) {
            Falcon3.set(ControlMode.PercentOutput, -1 * theController.getRawAxis(5));

        } else if (theController.getStickButtonReleased(Hand.kRight)) {
            Falcon3.set(ControlMode.PercentOutput, 0);
        }

    }
}
