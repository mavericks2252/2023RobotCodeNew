// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class CubeRelease extends CommandBase {
  Gripper gripper;
  ArmStowPosition armStowPosition;
  /** Creates a new CubeRelease. */
  public CubeRelease(Gripper gripper) {
    this.gripper = gripper;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //gripper.openGripper();
    gripper.reverseGripper(1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
    //gripper.closeGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (gripper.rangsensorGetVoltage() < 2.5) ? true : false;
  }
}
