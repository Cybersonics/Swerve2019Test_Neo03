/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc103.Robot2019.RobotMap;
import org.usfirst.frc103.Robot2019.OI;
import org.usfirst.frc103.Robot2019.Robot;
import edu.wpi.first.wpilibj.XboxController;


public class Climb extends Command {
  private final static double DOWN_VALUE = 900;
  private static boolean startPressed;
  private static boolean armLocked = false;

  public Climb() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startPressed = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
/*    if(OI.controller.getStartButton()) {
      Robot.elevatorFront.elevatorFrontClimbHeight();
    }

    if(OI.controller.getStartButton() && !startPressed) {
      Robot.elevatorFront.elevatorFrontClimbHeight(); 
      startPressed = true;
    }
*/
    if((Robot.elevatorFront.getElevatorFrontEncoder() == Robot.elevatorFront.CLIMB_HEIGHT) && startPressed==true){
        Robot.arm.armPosition(false, true);
    }

    if(Robot.arm.getArmPositionEncoder() == DOWN_VALUE && startPressed){
      Robot.arm.armPosition(false, false);
      Robot.arm.setArmPin();
      armLocked = true;
    }
    if(armLocked = true){
      Robot.elevatorFront.setElevatorFront(250);
      //this will not work, make a new method to set a constant motor value
      //Finish code!!
    }
    //still needs front and rear elevator commands
    //still needs to run rollers, once the robot is at the climb level
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
