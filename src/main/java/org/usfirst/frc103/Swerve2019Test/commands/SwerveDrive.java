package org.usfirst.frc103.Swerve2019Test.commands;

import org.usfirst.frc103.Swerve2019Test.Robot;
import org.usfirst.frc103.Swerve2019Test.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class SwerveDrive extends Command {

    public SwerveDrive() {
        requires(Robot.drive);
    }

    @Override
	protected void execute() {
		double vX = RobotMap.leftJoy.getX(), vY = -RobotMap.leftJoy.getY();
        double omega = RobotMap.rightJoy.getX() / 30.0;
        Robot.drive.swerveDrive(vX, vY, omega);
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
