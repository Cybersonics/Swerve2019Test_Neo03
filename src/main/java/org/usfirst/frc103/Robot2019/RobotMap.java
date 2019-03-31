package org.usfirst.frc103.Robot2019;

import org.usfirst.frc103.Robot2019.subsystems.RangeFinder;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;



import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;

import org.usfirst.frc103.Robot2019.subsystems.Drive;
import org.usfirst.frc103.Robot2019.subsystems.Pneumatics;
import org.usfirst.frc103.Robot2019.subsystems.Intake;
import org.usfirst.frc103.Robot2019.pixy.*;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

 
    public static Ultrasonic ultrasonic;

    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int CONTROLLER = 2;

    public static final int STEER_LEFT_FRONT_TALON = 16;
    public static final int STEER_LEFT_REAR_TALON = 17;
    public static final int STEER_RIGHT_FRONT_TALON = 18;
    public static final int STEER_RIGHT_REAR_TALON = 19;

    public static final int ELEVATOR_FRONT_TALON = 20;
    public static final int ELEVATOR_REAR_TALON = 21;

    public static final int INTAKE_TALON = 22;
    public static final int ARM_TALON = 23;

    public static final int HATCH_PANEL_SOLENOID_FORWARD = 0;
    public static final int HATCH_PANEL_SOLENOID_REVERSE = 1;
    public static final int ARM_LOCK_SOLENOID_FORWARD = 2;
    public static final int ARM_LOCK_SOLENOID_REVERSE = 3;
    public static final int ELEVATOR_REAR_LOCK_FORWARD = 4;
    public static final int ELEVATOR_REAR_LOCK_REVERSE = 5;

    public static Pixy pixy, pixyShooter;
    public static Relay LedRelay;
    public static Relay visionLEDelay;

    public static void init() {
        
        Pixy.enumerate();
        Pixy.ensureAvailable(0xD892D58D);
        pixy = new Pixy(0xD892D58D);
        pixy.startBlockProgram();
        pixy.startFrameGrabber();

        LedRelay = new Relay(0, Direction.kForward);
        LedRelay.set(Value.kOn);

        //visionLEDelay = new Relay(0, Direction.kForward);
        //visionLEDelay.set(Value.kOn);

        /*
        elevator = new Elevators();
        pneumatics = new Pneumatics();
        drive = new Drive();
        intake = new Intake();
        
        

      
        //ultrasonic = new Ultrasonic(8, 9);
        //RangeFinder.start();
*/
        CameraServer.getInstance().startAutomaticCapture();
    }
    
}
