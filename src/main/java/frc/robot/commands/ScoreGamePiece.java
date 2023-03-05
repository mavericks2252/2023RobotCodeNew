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
  Integer nodePosition;
  LEDModeSubsystem ledModeSubsystem;
  Double topArmGoal;
  Double topArmError;

  /** Creates a new ScoreGamePiece. */
  public ScoreGamePiece(Gripper gripper, TopArm topArm, BottomArm bottomArm,LEDModeSubsystem ledModeSubsystem, Integer nodePosition) {
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
    
    if (ledModeSubsystem.getRobotMode()){

      topArmGoal = topArm.getMotorEncoderPosition()+10;
      topArm.setMotorDownPosition(topArmGoal);
    }

    else {
      
    }
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    topArmError = topArmGoal - topArm.getMotorEncoderPosition();
    if (Math.abs(topArmError) <.5){

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
