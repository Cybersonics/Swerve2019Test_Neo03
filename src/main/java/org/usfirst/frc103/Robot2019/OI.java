package org.usfirst.frc103.Robot2019;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static Joystick leftJoy;
    public static Joystick rightJoy;
    public static XboxController controller;

    public  OI(){
        leftJoy = new Joystick(RobotMap.LEFT_JOYSTICK);
        rightJoy = new Joystick(RobotMap.RIGHT_JOYSTICK);
        controller = new XboxController(RobotMap.CONTROLLER);
    }

    public boolean getAButtonPress(){
        return controller.getAButtonPressed();
    }

    public boolean getAButtonRelease(){
        return controller.getAButtonReleased();
    }

    public boolean getBButtonn(){
        return controller.getBButton();
    }
    
    public boolean getBButtonPress(){
        return controller.getBButtonPressed();
    }

    public boolean getBButtonRelease(){
        return controller.getBButtonReleased();
    }

}
