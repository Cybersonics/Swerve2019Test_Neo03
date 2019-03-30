package org.usfirst.frc103.Robot2019.subsystems;


import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc103.Robot2019.commands.PneumaticControl;
import org.usfirst.frc103.Robot2019.OI;
import org.usfirst.frc103.Robot2019.Robot;
import org.usfirst.frc103.Robot2019.RobotMap;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pneumatics extends Subsystem {
  private DoubleSolenoid hatchPanelSolenoid;
  
  public Pneumatics() {
    hatchPanelSolenoid = new DoubleSolenoid(RobotMap.HATCH_PANEL_SOLENOID_FORWARD, RobotMap.HATCH_PANEL_SOLENOID_REVERSE);
  }

  public void setHatchpanel(){
    hatchPanelSolenoid.set(Value.kForward);
  }

  public void resetHatchPanel(){
    hatchPanelSolenoid.set(Value.kReverse);
  }

  /*
  public void hatchPanel() {
    DoubleSolenoid hatchPanelSolenoid2 = hatchPanelSolenoid;
    if (OI.controller.getAButtonPressed()) {
      hatchPanelSolenoid2.set(Value.kForward);
    } else if (OI.controller.getAButtonReleased()) {
      hatchPanelSolenoid2.set(Value.kReverse);
    }
  }
  */

 /* public void armLock() {
    if(OI.controller.getBButton() && !bPressed) {
      armLock.set(Value.kForward);
      if(OI.controller.getBButtonReleased()) {
        bPressed = true;
      }
    }
    else if(OI.controller.getBButton() && bPressed) {
      armLock.set(Value.kReverse);
      if(OI.controller.getBButtonReleased()) {
        bPressed = false;
      }
    }
    else {
      armLock.set(Value.kOff);
    }
  }*/



  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  protected void initDefaultCommand() {
    
    // Set the default command for a subsystem here.
    setDefaultCommand(new PneumaticControl());
  }
}