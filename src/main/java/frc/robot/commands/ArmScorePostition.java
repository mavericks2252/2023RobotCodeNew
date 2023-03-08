// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

public class ArmScorePostition extends CommandBase {
  /** Creates a new ArmScorePostition. */
  TopArm topArm;
  BottomArm bottomArm;
  LEDModeSubsystem ledModeSubsystem;
  double bottomGoalPosition;
  double topGoalPosition;
  double bottomArmError;
  Boolean topArmHold;
  Boolean bottomArmHold;
  Gripper gripper;
  GamePieceRelease coneRelease;
  int node;
  int low = 1;
  int mid = 2;
  int high = 3;
  int scoringNode;
  double bottomHoldPosition;
  

  public ArmScorePostition( int node, LEDModeSubsystem ledModeSubsystem, BottomArm bottomArm, TopArm topArm) {
    this.ledModeSubsystem = ledModeSubsystem;
    this.node = node;
    this.topArm = topArm;
    this.bottomArm = bottomArm;

    addRequirements(bottomArm, topArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    topArm.setMotorPosition(56);
    bottomArm.setMotorPosition(ArmConstants.kBottomReversePosition); // Set the lower arm angle back
    

    topArmHold = true;
    bottomArmHold = true;


    if(node == high) {
      if (ledModeSubsystem.getRobotMode()) {// Cube mode
        topGoalPosition = -5;
        bottomGoalPosition = 125;
      }

      else {// Cone mode
        topGoalPosition = -30;
        bottomGoalPosition = 130;
        bottomHoldPosition = 50;
      }
      scoringNode = high;
    }

    else if(node == mid) {

      if (ledModeSubsystem.getRobotMode()) {// Cube mode
        topGoalPosition = 20;
        bottomGoalPosition = 75;
      }

      else {// Cone mode
        topGoalPosition = 0;
        bottomGoalPosition = 93;
        bottomHoldPosition = 35;
      }
      scoringNode = mid;
    }

    else{  // VALUES NEED SET
      topGoalPosition = 90;
      bottomGoalPosition = 90;
      scoringNode = low;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putBoolean("Top Arm Hold", topArmHold);
    SmartDashboard.putBoolean("Bottom Arm hold", bottomArmHold);
    SmartDashboard.putNumber("Scoring Node", scoringNode);
    //SmartDashboard.putNumber("Bottom Arm Error", Math.abs(bottomArmError));*/

    bottomArmError = ArmConstants.kBottomReversePosition - bottomArm.getMotorEncoderPosition();

    

    if (Math.abs(bottomArmError) < 1) {
      topArmHold = false;
    }
    if (topArm.getMotorEncoderPosition() < bottomHoldPosition) {
      bottomArmHold = false;
    }

    if (!topArmHold) {
      topArm.setMotorPosition(topGoalPosition);
    }
    if (!bottomArmHold) {
      bottomArm.setMotorPosition(bottomGoalPosition);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  
  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

      if (Math.abs(bottomArm.getMotorEncoderPosition() - bottomGoalPosition) < .25 & Math.abs(topArm.getMotorEncoderPosition() - topGoalPosition) < .25){
        return true;
      }
      else {
        return false;
      }
    
  }
}
