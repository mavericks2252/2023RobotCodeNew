// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

public class ScoreGamePiece extends CommandBase {

  Gripper gripper;
  TopArm topArm;
  BottomArm bottomArm;
  int nodePosition;
  LEDModeSubsystem ledModeSubsystem;
  Double topArmGoal;
  Double topArmError;
  int low = 1;
  int mid = 2;
  int high = 3;

  /** Creates a new ScoreGamePiece. */
  public ScoreGamePiece(Gripper gripper, TopArm topArm, BottomArm bottomArm,LEDModeSubsystem ledModeSubsystem, int nodePosition) {
    this.gripper = gripper;
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.nodePosition = nodePosition;
    this.ledModeSubsystem = ledModeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper, topArm, bottomArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //High node
    if (nodePosition == high){
    
      if (ledModeSubsystem.getRobotMode()){// Cube mode

        topArmGoal = topArm.getMotorEncoderPosition()+10;
      }

      else {// Cone mode
        topArmGoal = topArm.getMotorEncoderPosition()+25;
      }
    }

      // Middle node
      if (nodePosition == mid){
    
      if (ledModeSubsystem.getRobotMode()){// Cube mode

      }

      else {// Cone mode
        topArmGoal = topArm.getMotorEncoderPosition()+15;
      }
    }

      // Low node
      if (nodePosition == low) {
    
        if (ledModeSubsystem.getRobotMode()){// Cube mode

        }

        else {// Cone mode
          
        }
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    topArmError = topArmGoal - topArm.getMotorEncoderPosition();
    topArm.setMotorPosition(topArmGoal);
    if (Math.abs(topArmError) < 1){
    gripper.runGripper(.2);
    }
    if (Math.abs(topArmError) < .5){
      
      bottomArm.setMotorPosition(118);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(topArmError) < .25){
      return true;
    }
    else{
    return false;
    }
  }
}
