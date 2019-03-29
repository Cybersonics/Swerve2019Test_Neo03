package org.usfirst.frc103.Swerve2019Test;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;





import edu.wpi.first.wpilibj.Ultrasonic;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import org.usfirst.frc103.Swerve2019Test.subsystems.Drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
    //public static TalonSRX driveLeftFront;
    //public static TalonSRX driveLeftRear;
    //public static TalonSRX driveRightFront;
    //public static TalonSRX driveRightRear;
    /*
    public static TalonSRX steerLeftFront;
    public static TalonSRX steerLeftRear;
    public static TalonSRX steerRightFront;
    public static TalonSRX steerRightRear;

    public static CANSparkMax driveLeftFrontSpark;
    public static CANSparkMax driveLeftRearSpark;
    public static CANSparkMax driveRightFrontSpark;
    public static CANSparkMax driveRightRearSpark;     
    
    public static CANPIDController driveLeftFrontController;
    public static CANPIDController driveLeftRearController;
    public static CANPIDController driveRightFrontController;
    public static CANPIDController driveRightRearController;

    */

    
    public static Ultrasonic ultrasonic;



    public static Joystick leftJoy;
    public static Joystick rightJoy;
    public static XboxController controller;

    public static void init() {
 	
    
        controller = new XboxController(2);
        leftJoy = new Joystick(0);
        rightJoy = new Joystick(1);


        
        
     
    }
    
}
