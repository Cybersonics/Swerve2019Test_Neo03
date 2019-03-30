package org.usfirst.frc103.Robot2019.commands;

import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;
import org.usfirst.frc103.Robot2019.subsystems.Intake;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import org.usfirst.frc103.Robot2019.OI;


public class IntakeControl extends Command {
  
  public IntakeControl() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
  }

  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (OI.leftJoy.getRawButton(2)) {
      Robot.intake.driverIntakeIn();
    } else if (OI.rightJoy.getRawButton(2)) {
      Robot.intake.driverIntakeOut();
    } else {
      Robot.intake.intakeRun(OI.controller.getTriggerAxis(Hand.kLeft), OI.controller.getTriggerAxis(Hand.kRight));
    }

    if (OI.leftJoy.getRawButton(3)) {
      Robot.intake.driverBallIntakeIn();
    }

    if (OI.controller.getPOV() == 180) {
      Robot.intake.operatorLowIntakeIn();
    }

    if (OI.controller.getPOV() == 270) {
      Robot.intake.operatorMediumIntakeIn();
    }    
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
