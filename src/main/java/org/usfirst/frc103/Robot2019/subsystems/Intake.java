package org.usfirst.frc103.Robot2019.subsystems;


import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc103.Robot2019.RobotMap;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc103.Robot2019.OI;
import org.usfirst.frc103.Robot2019.Robot;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import org.usfirst.frc103.Robot2019.commands.IntakeControl;

public class Intake extends Subsystem {
  public static TalonSRX intakeMotor;

  final double DEADZONE = 0.05;

  public Intake() {
    intakeMotor = new TalonSRX(RobotMap.INTAKE_TALON);
  }

    public void intakeRun(double intakeIn, double intakeOut) {
      if ((intakeIn > 0 && intakeOut > 0) || (intakeIn < DEADZONE && intakeOut < DEADZONE)) {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
      } else {
        if (intakeIn > 0){
          if (OI.controller.getYButton()) {
            intakeMotor.set(ControlMode.PercentOutput, -intakeIn);
          } else {
            intakeMotor.set(ControlMode.PercentOutput, -(intakeIn * intakeIn));
          }
        }
        if (intakeOut > 0){
          if (OI.controller.getYButton()) {
            intakeMotor.set(ControlMode.PercentOutput, intakeOut*intakeOut);
          } else {
            intakeMotor.set(ControlMode.PercentOutput, intakeOut * 0.25);
            //.25 for hold  .5 for ball eject
          }
        }
      }
    }

  public void operatorLowIntakeIn() {
    intakeMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void operatorMediumIntakeIn() {
    intakeMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void driverIntakeOut() {
    intakeMotor.set(ControlMode.PercentOutput, 1.0);
  }

  public void driverIntakeIn() {
    intakeMotor.set(ControlMode.PercentOutput, -1.0);
  }

  public void driverBallIntakeIn() {
    intakeMotor.set(ControlMode.PercentOutput, -0.6);
  }

  @Override
  protected void initDefaultCommand() { 
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeControl());
  }
}
