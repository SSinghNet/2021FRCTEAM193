package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

//Joshua 
public class Feeder{

XboxController eeController; 
CANSparkMax feederNeo;
CANSparkMax towerNeo;
DigitalInput opticalSensor;

public Feeder(XboxController eeController1, CANSparkMax feederNeo1, CANSparkMax towerNeo1, DigitalInput opticalSensor){
    eeController = eeController1;
    feederNeo = feederNeo1;
    towerNeo = towerNeo1;
}

boolean bumperPressed = false;
// The feeder turns on when the right bumper is pressed down
    public void FeederController(){
        if(eeController.getBumper(Hand.kRight)){
            towerNeo.set(-0.4);
            feederNeo.set(0.8);
        }else{
            feederNeo.set(0);
            if(eeController.getY() == 0){
                towerNeo.set(0);
            }
        }

    }
}
