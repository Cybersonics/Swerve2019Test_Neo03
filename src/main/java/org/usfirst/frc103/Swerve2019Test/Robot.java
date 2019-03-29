package org.usfirst.frc103.Swerve2019Test;

import org.usfirst.frc103.Swerve2019Test.RobotMap;

//import static org.usfirst.frc103.Swerve2019Test.RobotMap.navX;

//import java.util.List;
//import java.util.stream.Collectors;

import org.usfirst.frc103.Swerve2019Test.commands.DoNothingAuto;
import org.usfirst.frc103.Swerve2019Test.commands.DriveFieldCentric;

//import org.usfirst.frc103.Swerve2019Test.commands.VisionAutoSequence;

import org.usfirst.frc103.Swerve2019Test.subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
	
	SendableChooser<Command> autonomousChooser;
    Command autonomousCommand;
    
    public static double zeroHeading;
    
    public static Drive drive;
    public static AHRS navX;
    
    @Override
	public void robotInit() {
        RobotMap.init();
        navX = new AHRS(SPI.Port.kMXP);
        
        drive = new Drive();        
        
        autonomousChooser = new SendableChooser<Command>();
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

        // schedule the autonomous command
    	autonomousCommand = (Command) autonomousChooser.getSelected();
    	if (autonomousCommand != null) {
    		autonomousCommand.start();
    	}
    }

    @Override
	public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        //updateDashboard();
 
    }

    @Override
	public void teleopInit() {
        //zeroHeading = navX.getFusedHeading();
    	// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();

        
    }

    @Override
	public void teleopPeriodic() {
        Scheduler.getInstance().run();
        //updateDashboard();

    	if (RobotMap.leftJoy.getRawButton(10)) zeroHeading = navX.getFusedHeading();
    }

    @Override
    public void testPeriodic() {
        LiveWindow.run();
    }
    
    private void updateDashboard() {
    	SmartDashboard.putNumber("LF Steer Position", drive.steerLeftFront.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("LR Steer Position", drive.steerLeftRear.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("RF Steer Position", drive.steerRightFront.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("RR Steer Position", drive.steerRightRear.getSelectedSensorPosition(0));

//    	SmartDashboard.putNumber("LF Drive Position", driveLeftFront.getSelectedSensorPosition(0));
//    	SmartDashboard.putNumber("LR Drive Position", driveLeftRear.getSelectedSensorPosition(0));
//    	SmartDashboard.putNumber("RF Drive Position", driveRightFront.getSelectedSensorPosition(0));
//    	SmartDashboard.putNumber("RR Drive Position", driveRightRear.getSelectedSensorPosition(0));
    	
    	SmartDashboard.putNumber("NavXHeading", navX.getFusedHeading());
    	SmartDashboard.putNumber("NavX Angle", navX.getAngle());
    	SmartDashboard.putNumber("NavXCompass", navX.getCompassHeading());
    	SmartDashboard.putNumber("NavX Yaw", navX.getYaw());
    	//SmartDashboard.putNumber("NavX X Displacement", navX.getDisplacementX());
    	//SmartDashboard.putNumber("NavX Y Displacement", navX.getDisplacementY());
    	SmartDashboard.putNumber("ZeroHeading", zeroHeading);

    }
    
}
