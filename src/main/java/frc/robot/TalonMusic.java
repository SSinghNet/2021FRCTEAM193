package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Joystick;

public class TalonMusic{

    Orchestra theMusic;

    TalonFX falcon1;
    TalonFX falcon2;
    TalonFX falcon3;
    TalonFX falcon4;
    String musicFile;
    Joystick rightJoystick;

    public TalonMusic(TalonFX falcon1Robot, TalonFX falcon2Robot, TalonFX falcon3Robot, TalonFX falcon4Robot, String musicFileName, Joystick theRightJoystick){
        falcon1 = falcon1Robot;
        falcon2 = falcon2Robot;
        falcon3 = falcon3Robot;
        falcon4 = falcon4Robot;
        musicFile = musicFileName;
        rightJoystick = theRightJoystick;
    }

    public void PlayMusic(){
    
        // if(rightJoystick.getRawButton(3)){
            theMusic.play();
        // }
        
    }
    
    public void MusicInit(){

        ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>(4);
        _instruments.add(falcon1);
        _instruments.add(falcon2);
        _instruments.add(falcon3);
        _instruments.add(falcon4);
        theMusic = new Orchestra(_instruments);
        
        theMusic.loadMusic(musicFile);

    }

}