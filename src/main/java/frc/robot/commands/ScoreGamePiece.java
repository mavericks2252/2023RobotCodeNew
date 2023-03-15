// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

public class ScoreGamePiece extends CommandBase {

  Gripper gripper;
  TopArm topArm;
  BottomArm bottomArm;
  double nodePosition;
  LEDModeSubsystem ledModeSubsystem;
  double topArmGoal;
  double topArmError;
  double bottomArmGoal;
  double bottomArmError;
  int low = 1;
  int mid = 2;
  int high = 3;
  Timer endTimer;

  /** Creates a new ScoreGamePiece. */
  public ScoreGamePiece(Gripper gripper, TopArm topArm, BottomArm bottomArm,LEDModeSubsystem ledModeSubsystem) {
    this.gripper = gripper;
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.ledModeSubsystem = ledModeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper, topArm, bottomArm);
    endTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTimer.reset();
    endTimer.start();
    nodePosition = topArm.getNodePosition();
    
    //High node
    if (nodePosition == high || nodePosition == 4){
    
      if (ledModeSubsystem.getRobotMode()){// Cube mode

        topArmGoal = topArm.getMotorEncoderPosition();
        
      }

      else {// Cone mode
        topArmGoal = topArm.getMotorEncoderPosition()+25;
        bottomArmGoal = 118;
      }
    }

      // Middle node
      if (nodePosition == mid){
    
        if (ledModeSubsystem.getRobotMode()){// Cube mode
          topArmGoal = topArm.getMotorEncoderPosition();
        }

        else {// Cone mode
          if (topArm.getScoringPosition()) {// Reverse scoring
            topArmGoal = topArm.getMotorEncoderPosition()-30;
          }
          else{
          topArmGoal = topArm.getMotorEncoderPosition()+33;
          bottomArmGoal = 90;
          }
      }
    }

      // Low node
      if (nodePosition == low) {
    
        if (ledModeSubsystem.getRobotMode()){// Cube mode
          
        }

        else {// Cone mode
          gripper.runGripper(1);
          topArmGoal = 75;
        }
      }

      
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      topArm.setMotorPosition(topArmGoal);

    if (ledModeSubsystem.getRobotMode()) {
      gripper.reverseGripper(1);
      
    }

    else if (!ledModeSubsystem.getRobotMode() && nodePosition != low){ // Cone mode
      topArmError = topArmGoal - topArm.getMotorEncoderPosition();
      
      if (Math.abs(topArmError) < 4){
      gripper.runGripper(.4);
      bottomArm.setMotorPosition(bottomArmGoal);//118
      }
      if (Math.abs(topArmError) < .5){
        
        
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledModeSubsystem.stopBlinking();
    
    new Thread(() -> {
      try {
        Thread.sleep(500);
        gripper.stopGripper();
      } catch (Exception e) {
      }
      }).start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  // Cube mode
    if (ledModeSubsystem.getRobotMode() || nodePosition == low ){// If cube mode or in low node position and the end timer is greater than .25 sec
      if(endTimer.get() > .5){// If the beam break is not tripped return true
        return true;
      }
      else {// If the beam break is tripped return false
        return false;
      }
    }

  // Cone mode
    else {
      if (Math.abs(topArmError) < .25){// If the error of the top arm is within .25 of a degree return true
        return true;
      }
      else{
      return false;
      }
    }
  }
}
