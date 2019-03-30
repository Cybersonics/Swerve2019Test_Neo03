package org.usfirst.frc103.Robot2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc103.Robot2019.RobotMap;
import org.usfirst.frc103.Robot2019.commands.ElevatorFrontControl;
import org.usfirst.frc103.Robot2019.OI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc103.Robot2019.Robot;

public class ElevatorFront extends Subsystem {
  private TalonSRX elevatorFront;

  public static final double DEADZONE = 0.05;
  private static final double ELEVATOR_P = 10.0, ELEVATOR_I = 0.02, ELEVATOR_D = 0.0; //10  .02  0
  private static final int STATUS_FRAME_PERIOD = 5;

  private static final double LOWER = 20;
  private static final double HATCH_1_HEIGHT = 500;
  private static final double HATCH_2_HEIGHT = 1000;
  private static final double BALL_1_HEIGHT = 100;
  private static final double BALL_2_HEIGHT = 200;
  private static final double BALL_3_HEIGHT = 300;
  private static final double BALL_CARGO_HEIGHT = 150;
  public final double CLIMB_HEIGHT = 300;

  public ElevatorFront() {
    elevatorFront = new TalonSRX(RobotMap.ELEVATOR_FRONT_TALON);
    elevatorFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
/*    elevatorFront.config_kP(0, ELEVATOR_P, 0);
    elevatorFront.config_kI(0, ELEVATOR_I, 0);
    elevatorFront.config_kD(0, ELEVATOR_D, 0);
    elevatorFront.config_IntegralZone(0, 100, 0);
    elevatorFront.configAllowableClosedloopError(0, 5, 0);
*/    elevatorFront.setNeutralMode(NeutralMode.Brake);
    elevatorFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
    elevatorFront.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }
/*
  public void elevatorFrontHatchPositionUp() {
    if (getElevatorFrontEncoder() < HATCH_1_HEIGHT) {
      setElevatorFrontPosition(HATCH_1_HEIGHT);
    }
    else if (getElevatorFrontEncoder() >= HATCH_1_HEIGHT && getElevatorFrontEncoder() < HATCH_2_HEIGHT) {
      setElevatorFrontPosition(HATCH_2_HEIGHT);
    }
  }
  public void elevatorFrontBallPositionUp() {
    if (getElevatorFrontEncoder() < BALL_1_HEIGHT){
      setElevatorFrontPosition(BALL_1_HEIGHT);
    }
    else if (getElevatorFrontEncoder() < BALL_2_HEIGHT){
      setElevatorFrontPosition(BALL_2_HEIGHT);
    }
    else if (getElevatorFrontEncoder() >= BALL_2_HEIGHT && getElevatorFrontEncoder() < BALL_3_HEIGHT){
      setElevatorFrontPosition(BALL_3_HEIGHT);
    }
  }

  public void elevatorFrontHatchPositionDown() {
    if (getElevatorFrontEncoder() <= HATCH_2_HEIGHT){
      setElevatorFrontPosition(HATCH_1_HEIGHT);
    }
  }
  public void elevatorFrontBallPositionDown() {
    if (getElevatorFrontEncoder() <= BALL_2_HEIGHT ){
      setElevatorFrontPosition(BALL_1_HEIGHT);
    }
    else if (getElevatorFrontEncoder() > BALL_2_HEIGHT){
      setElevatorFrontPosition(BALL_2_HEIGHT);
    }
  }
  public void elevatorFrontBallCargoPosition() {
    setElevatorFrontPosition(BALL_CARGO_HEIGHT);
  }
  public void elevatorFrontClimbHeight(){
    setElevatorFrontPosition(CLIMB_HEIGHT);
  }
  //Ask operator for potential button ideas
*/



  public void setElevatorFront(double frontLift) {
    if (Math.abs(frontLift) < DEADZONE){
      elevatorFront.set(ControlMode.PercentOutput, 0.0);
    } else {
      elevatorFront.set(ControlMode.PercentOutput, -frontLift);
    }
  }

  public double getElevatorFrontEncoder(){
    return elevatorFront.getSelectedSensorPosition(0);
  }

  public void setElevatorFrontPosition(double position) {
    elevatorFront.set(ControlMode.Position, position);
  }

  public void zeroElevatorFrontPosition() {
    elevatorFront.set(ControlMode.Position, 0.0);
  }

  public void elevatorFrontOff() {
    elevatorFront.set(ControlMode.PercentOutput, 0.0);
  }

  public void elevatorFrontZeroEncoder() {
    elevatorFront.setSelectedSensorPosition(0);
  }

  public boolean elevatorFrontLimitHit() {
    return elevatorFront.getSensorCollection().isFwdLimitSwitchClosed();
  }
  
  

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevatorFrontControl());
  }
}
