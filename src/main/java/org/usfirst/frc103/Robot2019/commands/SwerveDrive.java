package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc103.Robot2019.OI;

public class SwerveDrive extends Command {

    public SwerveDrive() {
        requires(Robot.drive);
    }

    @Override
	protected void execute() {
		double vX = OI.leftJoy.getX(), vY = -OI.leftJoy.getY();
        double omega = OI.rightJoy.getX() / 30.0;
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
