package org.usfirst.frc103.Robot2019;



import java.util.List;
import java.util.stream.Collectors;


import org.usfirst.frc103.Robot2019.commands.ArmControl;
import org.usfirst.frc103.Robot2019.commands.DoNothingAuto;
import org.usfirst.frc103.Robot2019.commands.DriveFieldCentric;
import org.usfirst.frc103.Robot2019.commands.DriveForward;
import org.usfirst.frc103.Robot2019.commands.DriveLeft;
import org.usfirst.frc103.Robot2019.commands.DriveLeft;
//import org.usfirst.frc103.Robot2019.commands.VisionAutoSequence;
import org.usfirst.frc103.Robot2019.commands.SimpleDriveForward;
import org.usfirst.frc103.Robot2019.subsystems.Drive;
import org.usfirst.frc103.Robot2019.subsystems.ElevatorFront;
import org.usfirst.frc103.Robot2019.subsystems.ElevatorRear;
import org.usfirst.frc103.Robot2019.subsystems.Pneumatics;
import org.usfirst.frc103.Robot2019.subsystems.Intake;
import org.usfirst.frc103.Robot2019.subsystems.Arm;
import com.kauailabs.navx.frc.AHRS;
import org.usfirst.frc103.Robot2019.subsystems.Drive;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	
	SendableChooser<Command> autonomousChooser;
    Command autonomousCommand;
    
    public static boolean autonomousEnd;

    public static double zeroHeading;
    public static AHRS navX;
    
    public static Pneumatics pneumatics;
    public static ElevatorFront elevatorFront;
    public static ElevatorRear elevatorRear;
    public static Drive drive;
    public static Intake intake;
    public static Arm arm;
    public static OI oi;

    @Override
	public void robotInit() {
        RobotMap.init();
        pneumatics = new Pneumatics();
        drive = new Drive();
        elevatorFront = new ElevatorFront();
        elevatorRear = new ElevatorRear();
        intake = new Intake();
        arm = new Arm();
        navX = new AHRS(SPI.Port.kMXP);
        
        //OI is always last!!
        oi = new OI();
        
        autonomousChooser = new SendableChooser<Command>();
        //TODO: change distance for Driving
        autonomousChooser.addOption("Drive Forward", new DriveForward(0.0, 4500.0));
        autonomousChooser.addOption("Simple Drive Forward", new SimpleDriveForward(4500.0));
        autonomousChooser.addOption("Drive Left", new DriveLeft(4500, 4500));
        autonomousChooser.setDefaultOption("Do Nothing", new DoNothingAuto());
        
        SmartDashboard.putData("AutonomousCommands", autonomousChooser);

        zeroHeading = navX.getFusedHeading();
    }

    @Override
	public void disabledInit(){
    	//navX.resetDisplacement();
        //navX.zeroYaw();

    }

    @Override
	public void disabledPeriodic() {
        Scheduler.getInstance().run();
        drive.encoderReset();
        
        updateDashboard();
    }

    @Override
	public void autonomousInit() {
        zeroHeading = navX.getFusedHeading();

        autonomousEnd = false;

        // schedule the autonomous command
    	autonomousCommand = (Command) autonomousChooser.getSelected();
    	if (autonomousCommand != null) autonomousCommand.start();
    }

    @Override
	public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        updateDashboard();

        //canceling auton for teleop control during sandstorm
/*        if (OI.leftJoy.getRawButton(3)) {
            if (autonomousCommand != null) autonomousCommand.cancel();
        }
*/ 
    }

    @Override
	public void teleopInit() {
        //zeroHeading = navX.getFusedHeading();
    	// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        autonomousEnd = true;

        if (autonomousCommand != null) {
            autonomousCommand.cancel();

        }
    }

    @Override
	public void teleopPeriodic() {
        Scheduler.getInstance().run();
        updateDashboard();

    	if (OI.leftJoy.getRawButton(10)) zeroHeading = Robot.navX.getFusedHeading();
    }

    @Override
    public void testPeriodic() {
        LiveWindow.run();
    }
    
    private void updateDashboard() {
    	SmartDashboard.putNumber("LF Steer Position", drive.getSteerLFEncoder());
    	SmartDashboard.putNumber("LR Steer Position", drive.getSteerLREncoder());
    	SmartDashboard.putNumber("RF Steer Position", drive.getSteerRFEncoder());
    	SmartDashboard.putNumber("RR Steer Position", drive.getSteerRREncoder());

    	// SmartDashboard.putNumber("LF Drive Position", drive.getDriveLFEncoder());
    	// SmartDashboard.putNumber("LR Drive Position", drive.getDriveLREncoder());
    	// SmartDashboard.putNumber("RF Drive Position", drive.getDriveRFEncoder());
    	// SmartDashboard.putNumber("RR Drive Position", drive.getDriveRREncoder());
    	
    	SmartDashboard.putNumber("NavXHeading", navX.getFusedHeading());
    	SmartDashboard.putNumber("NavX Angle", navX.getAngle());
    	SmartDashboard.putNumber("NavXCompass", navX.getCompassHeading());
        SmartDashboard.putNumber("NavX Yaw", navX.getYaw());
        
        SmartDashboard.putNumber("Angle Testing", Math.abs(navX.getFusedHeading() - zeroHeading));
        SmartDashboard.putBoolean("0 degrees", Math.abs(navX.getFusedHeading() - zeroHeading) >= 355 || Math.abs(navX.getFusedHeading() - zeroHeading) <= 5);
        SmartDashboard.putBoolean("90 degrees", Math.abs(navX.getFusedHeading() - zeroHeading) >= 85 && Math.abs(navX.getFusedHeading() - zeroHeading) <= 95);
        SmartDashboard.putBoolean("180 degrees", Math.abs(navX.getFusedHeading() - zeroHeading) >= 175 && Math.abs(navX.getFusedHeading() - zeroHeading) <= 185);
        SmartDashboard.putBoolean("270 degrees", Math.abs(navX.getFusedHeading() - zeroHeading) >= 265 && Math.abs(navX.getFusedHeading() - zeroHeading) <= 275);
    	//SmartDashboard.putNumber("NavX X Displacement", navX.getDisplacementX());
    	//SmartDashboard.putNumber("NavX Y Displacement", navX.getDisplacementY());
        SmartDashboard.putNumber("ZeroHeading", zeroHeading);
        
        SmartDashboard.putNumber("Front Elevator", elevatorFront.getElevatorFrontEncoder());

        SmartDashboard.putBoolean("Arm Locked", !ArmControl.armLock);

    }    
}