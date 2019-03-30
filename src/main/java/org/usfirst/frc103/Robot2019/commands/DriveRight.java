/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc103.Robot2019.Robot;


public class DriveRight extends Command {
  boolean isDone;
  boolean secondStep;
  double forwardDistance;
  double strafeDistance;
  double forwardAverage;

  public DriveRight(double forwardDistance, double strafeDistance) {
    requires(Robot.drive);
    this.forwardDistance = forwardDistance;
    this.strafeDistance = strafeDistance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    isDone = false;
    secondStep = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double forward = 0.0;
    double strafe = 0.0; 

    // if ((Math.abs(Robot.drive.getDriveLFEncoder()) < forwardDistance) &&
		// 		(Math.abs(Robot.drive.getDriveLREncoder()) < forwardDistance) &&
		// 		(Math.abs(Robot.drive.getDriveRFEncoder()) < forwardDistance) &&
    //     (Math.abs(Robot.drive.getDriveRREncoder()) < forwardDistance) &&
    //     !secondStep) {
			
    //   forward = 0.4;//0.35
    //   forwardAverage = (Math.abs(Robot.drive.getDriveLFEncoder())+ Math.abs(Robot.drive.getDriveLREncoder()) + Math.abs(Robot.drive.getDriveRFEncoder()) + Math.abs(Robot.drive.getDriveRREncoder())) / 4;
    // }
    // else if((forwardAverage - Math.abs(Robot.drive.getDriveLFEncoder()) < strafeDistance) &&
    //     (forwardAverage - Math.abs(Robot.drive.getDriveLREncoder()) < strafeDistance) &&
    //     (forwardAverage - Math.abs(Robot.drive.getDriveRFEncoder()) < strafeDistance) &&
    //     (forwardAverage - Math.abs(Robot.drive.getDriveRREncoder()) < strafeDistance)) {
    //   secondStep = true;
    //   forward = 0.0;
    //   strafe = 0.5;
      
    // }
    // else {
    //   forward = 0.0;
    //   strafe = 0.0;
    //   isDone = true;
    // }

    Robot.drive.swerveDrive(strafe, forward, 0.0);
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
