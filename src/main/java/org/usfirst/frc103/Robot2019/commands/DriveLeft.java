/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc103.Robot2019.Robot;


public class DriveLeft extends Command {
  double strafe;
  boolean isDone;
  double distanceTraveled;
  double forward;
  boolean distanceMet;
  double strafeDistance;
  double distanceForward;

  public DriveLeft(double distanceForward, double strafeDistance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drive);
    this.distanceForward = distanceForward;
    this.strafeDistance = strafeDistance;
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    strafe = 0.0;
    forward = 0.0;
    isDone = false;
    distanceMet = false;
    //distanceTraveled is the average value of all the encoders
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if ((Math.abs(Robot.drive.getDriveLFEncoder()) < distanceForward) &&
		// 		(Math.abs(Robot.drive.getDriveLREncoder()) < distanceForward) &&
		// 		(Math.abs(Robot.drive.getDriveRFEncoder()) < distanceForward) &&
    //     (Math.abs(Robot.drive.getDriveRREncoder()) < distanceForward) &&
    //     distanceMet == false) {
    //   forward = 0.5;
    //   distanceTraveled = (Math.abs(Robot.drive.getDriveLREncoder()) + Math.abs(Robot.drive.getDriveLREncoder()) + Math.abs(Robot.drive.getDriveRFEncoder()) + Math.abs(Robot.drive.getDriveRREncoder())) / 4;
    // } else if (Math.abs(distanceTraveled - Math.abs(Robot.drive.getDriveLFEncoder())) < strafeDistance &&
    // Math.abs(distanceTraveled - Math.abs(Robot.drive.getDriveLREncoder())) < strafeDistance &&
    // Math.abs(distanceTraveled - Math.abs(Robot.drive.getDriveRFEncoder())) < strafeDistance &&
    // Math.abs(distanceTraveled - Math.abs(Robot.drive.getDriveRREncoder())) < strafeDistance) {
    //     distanceMet = true;
    //     strafe = -0.5;
    //     forward = 0.0;
    // } else {
    //   strafe = 0.0;
    //   forward = 0.0;
    //   isDone = true;
    // }

    Robot.drive.swerveDrive(strafe, forward, 0.0);
  }// find a way to make this code include fewer booleans

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
