// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

public class SwitchNodePosition extends CommandBase {
  TopArm topArm;
  BottomArm bottomArm;
  LEDModeSubsystem ledModeSubsystem;
  int midNode = 2, highNode = 3;
  double bottomReversePosition, topArmGoal, bottomArmGoal, bottomHoldPos, topHoldPos, topArmError, bottomArmError;
  boolean topArmHold, bottomArmHold;
  /** Creates a new SwitchNodePosition. */
  public SwitchNodePosition(TopArm topArm, BottomArm bottomArm, LEDModeSubsystem ledModeSubsystem) {
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.ledModeSubsystem = ledModeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(topArm, bottomArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    if (topArm.getStoredNode() == highNode) {// All placeholders
      topArmGoal = 7;
      bottomArmGoal = 92;
      bottomHoldPos = 2;// Hold the bottom arm until the top reaches this angle
      topArmHold = true;
      bottomArmHold = false;
      topArm.setNodePosition(midNode);
    }
    else if (topArm.getStoredNode() == midNode) {
      topArmGoal = -32;
      bottomArmGoal = 131;
      topHoldPos = 100;// Hold the top arm until the bottom reaches this angle
      topArmHold = false;
      bottomArmHold = true;
      topArm.setNodePosition(highNode);
    }
    else {
      topArmGoal = topArm.getMotorEncoderPosition();
      bottomArmGoal = bottomArm.getMotorEncoderPosition();
      bottomHoldPos = bottomArm.getMotorEncoderPosition();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    topArmError = Math.abs(topArm.getMotorEncoderPosition() - bottomHoldPos);
    bottomArmError = Math.abs(bottomArm.getMotorEncoderPosition() - topHoldPos);

    if (topArmError < 1 && topArm.getStoredNode() == midNode) {
      bottomArmHold = false;
    }
    else if (bottomArmError < 1 && topArm.getStoredNode() == highNode) {
      topArmHold = false;
    }

    if (topArm.getStoredNode() == midNode) {
      topArm.setMotorPosition(topArmGoal);
      if (!bottomArmHold) {
        bottomArm.setMotorPosition(bottomArmGoal);
      }
    }
    else if (topArm.getStoredNode() == highNode) {
      bottomArm.setMotorPosition(bottomArmGoal);
      if (!topArmHold) {
        topArm.setMotorDownPosition(topArmGoal);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (topArm.getStoredNode() == highNode) {
      topArm.storeNode(midNode);
    }
    else if (topArm.getStoredNode() == midNode) {
      topArm.storeNode(highNode);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ledModeSubsystem.getRobotMode()) {
      return true;
    }
    else if (Math.abs(topArm.getMotorEncoderPosition() - topArmGoal) < 1 
    && Math.abs(bottomArm.getMotorEncoderPosition() - bottomArmGoal) < 1) {
    return true;
    }
    else {
      return false;
    }
  }
}
