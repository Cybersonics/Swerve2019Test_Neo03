package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import org.usfirst.frc103.Robot2019.OI;


public class ElevatorRearControl extends Command {
  boolean rearLiftLocked;
    
  public ElevatorRearControl() {
    requires(Robot.elevatorRear);
  }

  @Override
  protected void initialize() {
    rearLiftLocked = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
//    if (OI.controller.getYButton()) {
      Robot.elevatorRear.setElevatorRear(OI.controller.getY(Hand.kLeft), rearLiftLocked);
//    }
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
