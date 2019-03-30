package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SimpleDriveForward extends Command {

  boolean isDone;
  double distance;

  public SimpleDriveForward(double distance) {
    requires(Robot.drive);
    this.distance = distance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    isDone = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double forward = 0.0;

    // if ((Math.abs(Robot.drive.getDriveLFEncoder()) < distance) &&
		// 		(Math.abs(Robot.drive.getDriveLREncoder()) < distance) &&
		// 		(Math.abs(Robot.drive.getDriveRFEncoder()) < distance) &&
		// 		(Math.abs(Robot.drive.getDriveRREncoder()) < distance)) {
			
		// 	forward = 0.4;//0.35
		// } else {
		// 	forward = 0.0;
		// 	isDone = true;
    // }
    
    Robot.drive.swerveDrive(0.0, forward, 0.0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.swerveDrive(0.0, 0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
