package org.usfirst.frc103.Robot2019.commands;

import org.usfirst.frc103.Robot2019.OI;
import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.subsystems.ElevatorFront;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class ElevatorFrontControl extends Command {
  public ElevatorFrontControl() {
    requires(Robot.elevatorFront);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.elevatorFront.setElevatorFront(OI.controller.getY(Hand.kRight));

    if (Robot.elevatorFront.elevatorFrontLimitHit()) {
      Robot.elevatorFront.elevatorFrontZeroEncoder();
    }
/*
//balls
    if (OI.controller.getYButton() && OI.controller.getPOV()==1) {
      Robot.elevatorFront.elevatorFrontBallPositionUp();
    }
    else if (OI.controller.getYButton() && OI.controller.getPOV()==5) {
      Robot.elevatorFront.elevatorFrontBallPositionDown();
    }
    else if (OI.controller.getPOV()==1) {
      Robot.elevatorFront.elevatorFrontHatchPositionUp();
    }
    else if (OI.controller.getPOV()==5) {
      Robot.elevatorFront.elevatorFrontHatchPositionDown();
    }
    else if (OI.controller.getXButton()) {
      Robot.elevatorFront.elevatorFrontBallCargoPosition();
    }
*/
    

    
    //needs to be given more to the subsystem, elevator not moving
    //just ask for POV value in command
/*    if (OI.controller.getY(Hand.kRight) > ElevatorFront.DEADZONE) {
      Robot.elevatorFront.setElevatorFront(OI.controller.getY(Hand.kRight));
    }
    else if ((OI.controller.getPOV() == 1) && Robot.elevatorFront.getElevatorFrontEncoder() > 0 && Robot.elevatorFront.getElevatorFrontEncoder()<400) {
      Robot.elevatorFront.setElevatorFrontPosition(500);
    }
    else if ((OI.controller.getPOV()==1) && Robot.elevatorFront.getElevatorFrontEncoder()>450 && Robot.elevatorFront.getElevatorFrontEncoder()<1450){
      Robot.elevatorFront.setElevatorFrontPosition(1500);
    }
    else if ((OI.controller.getPOV()==1) && Robot.elevatorFront.getElevatorFrontEncoder()>1450 && Robot.elevatorFront.getElevatorFrontEncoder()<2000){
      Robot.elevatorFront.setElevatorFrontPosition(2500);
    }
    else if ((OI.controller.getPOV()==5) && Robot.elevatorFront.getElevatorFrontEncoder()<2520 && Robot.elevatorFront.getElevatorFrontEncoder() >1550){
      Robot.elevatorFront.setElevatorFrontPosition(1500);
    }
    else if ((OI.controller.getPOV()==5) && Robot.elevatorFront.getElevatorFrontEncoder()<1520 && Robot.elevatorFront.getElevatorFrontEncoder() >550){
      Robot.elevatorFront.setElevatorFrontPosition(500);
    }
    else if ((OI.controller.getPOV()==5) && Robot.elevatorFront.getElevatorFrontEncoder()<550 && Robot.elevatorFront.getElevatorFrontEncoder() >0){
      Robot.elevatorFront.setElevatorFrontPosition(0);
    } else {
      Robot.elevatorFront.elevatorFrontOff();
    }
*/    
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
