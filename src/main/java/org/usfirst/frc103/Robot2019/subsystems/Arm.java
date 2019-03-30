package org.usfirst.frc103.Robot2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;

import org.usfirst.frc103.Robot2019.RobotMap;
import org.usfirst.frc103.Robot2019.commands.ArmControl;

public class Arm extends Subsystem {
  public static TalonSRX armMotor;

  private DoubleSolenoid armLock;

  final double DEADZONE = 0.05;
  private static final double ARM_P = 10.0, ARM_I = 0.02, ARM_D = 0.0, ARM_F = 0.0;
  private static final int STATUS_FRAME_PERIOD = 5;

  public Arm() {
    armMotor = new TalonSRX(RobotMap.ARM_TALON);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    armMotor.config_kP(0, ARM_P, 0);
    armMotor.config_kI(0, ARM_I, 0);
    armMotor.config_kD(0, ARM_D, 0);
    armMotor.config_kF(0, ARM_F, 0);
    armMotor.config_IntegralZone(0, 100, 0);
    armMotor.configAllowableClosedloopError(0, 5, 0);
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

    armLock = new DoubleSolenoid(RobotMap.ARM_LOCK_SOLENOID_FORWARD, RobotMap.ARM_LOCK_SOLENOID_REVERSE);
  }


  public void armPosition(boolean armUp, boolean armDown) {
    if ((armUp && armDown) || (!armUp && !armDown)) {
      armMotor.set(ControlMode.PercentOutput, 0.0);
    } else {
      if (armUp){
        armMotor.set(ControlMode.PercentOutput, 0.50);
      }
      if (armDown){
        armMotor.set(ControlMode.PercentOutput, -0.50);
      }
    }
  }

  public void setArmLock(Value value) {
    armLock.set(value);
  }

  public void setArmPin(){
    armLock.set(Value.kForward);
  }

  public void resetArmPin(){
    armLock.set(Value.kReverse);
  }

  public double getArmPositionEncoder(){
    return armMotor.getSelectedSensorPosition(0); // number needs to be corrected
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArmControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
