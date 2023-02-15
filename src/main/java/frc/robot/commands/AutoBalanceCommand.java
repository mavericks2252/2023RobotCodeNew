// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */

  SwerveSubsystem swerveSubsystem;
  Double balanceAngle;
  Double lastAngle;
  private double xSpeed;
  SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

  public AutoBalanceCommand(SwerveSubsystem ss) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSubsystem = ss;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (lastAngle < 10 & lastAngle > -10) {
      xSpeed = 0;
    }
    else if(lastAngle > swerveSubsystem.getBalanceAngle() & lastAngle > balanceAngle){
      xSpeed = DriveConstants.kAutoBalanceSpeed;
    }
    //else if(lastAngle)

    
    
    
    
    lastAngle = swerveSubsystem.getBalanceAngle();

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
