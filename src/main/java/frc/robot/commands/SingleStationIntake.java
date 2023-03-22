// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

public class SingleStationIntake extends CommandBase {
  LEDModeSubsystem ledModeSubsystem;
  TopArm topArm;
  BottomArm bottomArm;
  Gripper gripper;
  Timer endTimer;
  /** Creates a new SingleStationIntake. */
  public SingleStationIntake(LEDModeSubsystem ledModeSubsystem, TopArm topArm, BottomArm bottomArm, Gripper gripper) {
    this.ledModeSubsystem = ledModeSubsystem;
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.gripper = gripper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(topArm, bottomArm, gripper);
    endTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (ledModeSubsystem.getRobotMode()) {
      topArm.setMotorPosition(0);
      bottomArm.setMotorPosition(50);
      gripper.runGripper(GripperConstants.gripperMotorSpeed);
      endTimer.reset();
      endTimer.start();
    }

    else {
      topArm.setMotorPosition(7);//nose first 3
      bottomArm.setMotorPosition(55);//nose first 57
      gripper.runGripper(-GripperConstants.gripperMotorSpeed);
      endTimer.reset();
      endTimer.start();
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (ledModeSubsystem.getRobotMode()) {
      gripper.gripperHoldCube();
    }

    else {
    gripper.gripperHoldCone();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // cube Mode
    if (ledModeSubsystem.getRobotMode()){
      if (gripper.getBeamBreakSensor() & gripper.getGripperCurrent() > 7){
        return true;
      }
      else {
        return false;
      }
    }
    //cone mode
    else {
      if (gripper.getGripperCurrent() > 50){
        /*if(endTimer.get() > .2){
        return true;
        }
        else {
          return false;
        }*/
        return true;
      }
      else {
        endTimer.reset();
        return false;
      }
    }
}
}