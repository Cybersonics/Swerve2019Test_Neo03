package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.OI;
import org.usfirst.frc103.Robot2019.subsystems.Arm;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class ArmControl extends Command {
  public static boolean armLock;

  public ArmControl() {
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    armLock = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.oi.getBButtonn() && !armLock) {
      //.Robot.arm.setArmLock(Value.kForward);
      Robot.arm.setArmPin();
      if (Robot.oi.getBButtonRelease()){
        armLock = true;
      }
    } else if (Robot.oi.getBButtonn() && armLock) {
      //Robot.arm.setArmLock(Value.kReverse);
      Robot.arm.resetArmPin();
      if (Robot.oi.getBButtonRelease()){
        armLock = false;
      }
    }

    Robot.arm.armPosition(OI.controller.getBumper(Hand.kLeft), OI.controller.getBumper(Hand.kRight));    
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
