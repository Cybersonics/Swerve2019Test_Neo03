package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc103.Robot2019.OI;

public class FieldCentricSwerveDrive extends Command {
	
	public static final double OMEGA_SCALE = 1.0 / 30.0;
	public static final double DEADZONE = 0.05;

    private double originHeading = 0.0;
    private boolean originLocked = false;
    private double leftPow = 1.0;
	private double rightPow = 1.0;
	
	private double omega;

	public FieldCentricSwerveDrive() {
        requires(Robot.drive);
        //originHeading = RobotMap.navX.getFusedHeading();
    }
	
	@Override
	protected void initialize() {
        originHeading = Robot.zeroHeading;
	}

    @Override
	protected void execute() {
    	if (OI.leftJoy.getRawButton(7)) originHeading = Robot.navX.getFusedHeading();
    	
    	//if (Robot.oi.leftJoy.getRawButton(8)) leftPow = 1.0;
    	//if (Robot.oi.leftJoy.getRawButton(10)) leftPow = 1.5;
    	//if (Robot.oi.leftJoy.getRawButton(12)) leftPow = 2.0;
    	
    	//if (Robot.oi.rightJoy.getRawButton(8)) rightPow = 1.0;
    	//if (Robot.oi.rightJoy.getRawButton(10)) rightPow = 1.5;
    	//if (Robot.oi.rightJoy.getRawButton(12)) rightPow = 2.0;
    		
		double strafe = Math.pow(Math.abs(OI.leftJoy.getX()), leftPow) * Math.signum(OI.leftJoy.getX());
		double forward = Math.pow(Math.abs(OI.leftJoy.getY()), leftPow) * -Math.signum(OI.leftJoy.getY());

//		if(OI.rightJoy.getTrigger()) {
			//if(Math.abs(RobotMap.navX.getFusedHeading() - Robot.zeroHeading) >= 350 || Math.abs(RobotMap.navX.getFusedHeading() - Robot.zeroHeading) <= 10 ||
			//Math.abs(RobotMap.navX.getFusedHeading() - Robot.zeroHeading) >= 80 && Math.abs(RobotMap.navX.getFusedHeading() - Robot.zeroHeading) <= 100 ||
			//Math.abs(RobotMap.navX.getFusedHeading() - Robot.zeroHeading) >= 170 && Math.abs(RobotMap.navX.getFusedHeading() - Robot.zeroHeading) <= 190 ||
			//Math.abs(RobotMap.navX.getFusedHeading() - Robot.zeroHeading) >= 260 && Math.abs(RobotMap.navX.getFusedHeading() - Robot.zeroHeading) <= 280) {
			//	omega = 0.0;
			//} else {
//				omega = Robot.drive.snapTo90();
			//}
//		} else {
			omega = Math.pow(Math.abs(OI.rightJoy.getX()), rightPow) * Math.signum(OI.rightJoy.getX()) * OMEGA_SCALE;
//		}

		/*double strafe = Math.pow(Math.abs(Robot.oi.controller.getX(Hand.kLeft)), leftPow) * Math.signum(Robot.oi.controller.getX(Hand.kLeft));
		double forward = Math.pow(Math.abs(Robot.oi.controller.getY(Hand.kLeft)), leftPow) * -Math.signum(Robot.oi.controller.getY(Hand.kLeft));
        double omega = Math.pow(Math.abs(Robot.oi.controller.getX(Hand.kRight)), rightPow) * Math.signum(Robot.oi.controller.getX(Hand.kRight)) * OMEGA_SCALE;*/
        
        /*if (Robot.oi.leftJoy.getRawButton(8)) {
        	omega += Math.cos(5.0 * Math.PI * Timer.getFPGATimestamp()) * OMEGA_SCALE * 0.5;
        }
        if (Robot.oi.leftJoy.getRawButton(9)) {
        	strafe += Math.cos(5.0 * Math.PI * Timer.getFPGATimestamp()) * 0.15;
        }*/
        /*if (Robot.oi.rightJoy.getTrigger()) {
			strafe += Math.sin(15.0 * Math.PI * Timer.getFPGATimestamp()) * 0.2;
			//omega += Math.sin(15.0 * Math.PI * Timer.getFPGATimestamp()) * 0.02;
        }*/
		
        // Add a small deadzone on the joysticks
        if (Math.abs(strafe) < Math.pow(DEADZONE, leftPow)) strafe = 0.0;
		if (Math.abs(forward) < Math.pow(DEADZONE, leftPow)) forward = 0.0;
		if (Math.abs(omega) < Math.pow(DEADZONE, rightPow) * OMEGA_SCALE) omega = 0.0;
		
		
		// If all of the joysticks are in the deadzone, don't update the motors
		// This makes side-to-side strafing much smoother
		if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
			Robot.drive.setDriveLeftFront(0.0);
			Robot.drive.setDriveLeftRear(0.0);
			Robot.drive.setDriveRightFront(0.0);
			Robot.drive.setDriveRightRear(0.0);
			return;
		}
		
        if (!OI.leftJoy.getTrigger()) {
        	// When the trigger is pressed, we lock the origin heading to the current
        	// orientation of the robot, but only when the trigger is first pressed
    	//	if (!originLocked) {
    	//		//originHeading = RobotMap.navX.getFusedHeading();
    	//		originLocked = true;
    	//	}
    		
    		// Rotate the velocity vector from the joystick by the difference between our
    		// current orientation and the current origin heading
    		double originCorrection = Math.toRadians(originHeading - Robot.navX.getFusedHeading());
    		double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
    		strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
    		forward = temp;
    	//} else if (originLocked) {
    	//	originLocked = false;
    	}
        
        Robot.drive.swerveDrive(strafe, forward, omega);
    }

    @Override
	protected boolean isFinished() {
        return false;
    }

    @Override
	protected void end() {
    }

    @Override
	protected void interrupted() {
    	end();
    }

}
