// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */

  SwerveSubsystem swerveSubsystem;
  Double balanceAngle;
  public AutoBalanceCommand(SwerveSubsystem ss) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSubsystem = ss;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    balanceAngle = swerveSubsystem.getBalanceAngle();

    if (balanceAngle < 10 & balanceAngle > -10) {
      swerveSubsystem.stopModules();
    }
    //else if()
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
