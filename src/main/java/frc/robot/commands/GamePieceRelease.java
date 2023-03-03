// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LEDModeSubsystem;

public class GamePieceRelease extends CommandBase {
  Gripper gripper;
  ArmStowPosition armStowPosition;
  LEDModeSubsystem ledModeSubsystem;
  /** Creates a new ConeRelease. */
  public GamePieceRelease(Gripper gripper, LEDModeSubsystem ledModeSubsystem) {
    this.gripper = gripper;
    this.ledModeSubsystem = ledModeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ledModeSubsystem.getRobotMode()){// Cube mode
      gripper.reverseGripper(1);
    }
    else {// Cone mode
      gripper.openGripper();
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
    return false;//(gripper.rangsensorGetVoltage() < 2.5) ? true : false;
  }
}
