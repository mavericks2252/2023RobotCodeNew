// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BottomArmConstants;
import frc.robot.Constants.TopArmConstants;
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
   
    if (topArm.getNodePosition() == highNode) {// Going to mid node
      topArmGoal = TopArmConstants.kMidNodeScorePosCone;
      bottomArmGoal = BottomArmConstants.kMidNodeScorePosCone;
      bottomHoldPos = 100;// Hold the bottom arm until the top reaches this angle
      topArmHold = true;
      bottomArmHold = false;
      bottomArm.setMotorPosition(bottomArmGoal);
    }
    else if (topArm.getNodePosition() == midNode) {// Going to high node
      topArmGoal = TopArmConstants.kHighNodeScorePosCone;
      bottomArmGoal = BottomArmConstants.kHighNodeScorePosCone;
      topHoldPos = 4;// Hold the top arm until the bottom reaches this angle
      topArmHold = false;
      bottomArmHold = true;
      topArm.setMotorPosition(topArmGoal);
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
    topArmError = Math.abs(topArm.getMotorEncoderPosition() - topHoldPos);
    bottomArmError = Math.abs(bottomArm.getMotorEncoderPosition() - bottomHoldPos);
    SmartDashboard.putBoolean("Top Arm Hold", topArmHold);
    SmartDashboard.putBoolean("Bottom Arm Hold", bottomArmHold);

    // is at mid node heading to high node move top arm up and hold bottom arm until hold position reached
    if (topArmError < 1 && topArm.getNodePosition() == midNode) { 
      bottomArmHold = false;
      
    }

    // is at hight node going to mid node.  Hold top arm until bottom arm retracts to hold position
    if (bottomArmError < 1 && topArm.getNodePosition() == highNode) {
      topArmHold = false;
      
    }

    if (!bottomArmHold) {
        bottomArm.setMotorPosition(bottomArmGoal);
      }
    
    
    if (!topArmHold) {
        topArm.setMotorDownPosition(topArmGoal);
      }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (topArm.getNodePosition() == midNode){
      topArm.setNodePosition(highNode);
    }

    else if (topArm.getNodePosition() == highNode){
      topArm.setNodePosition(midNode);
    }
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ledModeSubsystem.getRobotMode()) {
      return true;
    }

    else if ( Math.abs(topArm.getMotorEncoderPosition() - topArmGoal) < 1 
              && Math.abs(bottomArm.getMotorEncoderPosition() - bottomArmGoal) < 1) {
      return true;
    }

    else {
      return false;
    }
  }
}
