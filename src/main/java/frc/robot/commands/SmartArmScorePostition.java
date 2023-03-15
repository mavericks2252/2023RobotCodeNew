// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopArm;

public class SmartArmScorePostition extends CommandBase {
  /** Creates a new ArmScorePostition. */
  TopArm topArm;
  BottomArm bottomArm;
  LEDModeSubsystem ledModeSubsystem;
  SwerveSubsystem swerveSubsystem;
  Intake intake;
  double bottomGoalPosition;
  double topGoalPosition;
  double bottomArmError;
  double reversePosition;
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
  

  public SmartArmScorePostition( int node, LEDModeSubsystem ledModeSubsystem, BottomArm bottomArm, TopArm topArm, SwerveSubsystem swerveSubsystem, Intake intake) {
    this.ledModeSubsystem = ledModeSubsystem;
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.swerveSubsystem = swerveSubsystem;
    this.intake = intake;
    this.node = node;
    addRequirements(bottomArm, topArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (swerveSubsystem.getHeading() < -90 || swerveSubsystem.getHeading() > 90) {
      topArm.setReverseScoring();
      intake.extendIntake();
    }

    // Sets reverse position depending on robot orientation 
    topArm.setMotorPosition(56);
    if (topArm.getScoringPosition()) {
      reversePosition = 100;
    }
    else{
     reversePosition = ArmConstants.kBottomReversePosition;
    }

    bottomArm.setMotorPosition(reversePosition);// Set the lower arm angle back

    topArmHold = true;
    bottomArmHold = true;


    if(node == high) {
      if (ledModeSubsystem.getRobotMode()) {// Cube mode
        
        topGoalPosition = -5;
        bottomGoalPosition = 105;
        bottomHoldPosition = 15;
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
        topGoalPosition = 5;
        bottomGoalPosition = 75;
        bottomHoldPosition = 35;
      }

      else {// Cone mode
        // Sets position depending on robot orientation
        if (topArm.getScoringPosition()){
          topGoalPosition = 210;
          bottomGoalPosition = 58;  
          bottomHoldPosition = 160;
        }
        else {
        topGoalPosition = 0;
        bottomGoalPosition = 93;
        bottomHoldPosition = 35;
        }
      }
      scoringNode = mid;
    }

    else{ 
      topGoalPosition = 75;
      bottomGoalPosition = 98;
      scoringNode = low;
      bottomHoldPosition = 75;
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
    if (topArm.getScoringPosition()){
      if (topArm.getMotorEncoderPosition() > bottomHoldPosition) {
        bottomArmHold = false;
      }
    }
    else {
      if (topArm.getMotorEncoderPosition() < bottomHoldPosition) {
        bottomArmHold = false;
      }
    }

    if (!topArmHold) {
      topArm.setMotorPosition(topGoalPosition);
    }
    if (!bottomArmHold) {
      bottomArm.setMotorPosition(bottomGoalPosition);
    }
    SmartDashboard.putBoolean("Score Command Running", true);
    
    SmartDashboard.putNumber("Top Arm Error", (Math.abs(topArm.getMotorEncoderPosition()) - Math.abs(topGoalPosition)));
    SmartDashboard.putNumber("Bottom Arm Error", (Math.abs(bottomArm.getMotorEncoderPosition()) - Math.abs(bottomGoalPosition)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    SmartDashboard.putBoolean("Score Command Running", false);
    topArm.setNodePosition(node);

    
  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

      if (Math.abs(bottomArm.getMotorEncoderPosition() - bottomGoalPosition) < 1 && Math.abs(topArm.getMotorEncoderPosition() - topGoalPosition) < 1){
        ledModeSubsystem.stopBlinking();
        return true;
      }
      else {
        return false;
      }
    
  }
}
