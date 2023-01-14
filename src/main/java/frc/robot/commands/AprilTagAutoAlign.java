// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class AprilTagAutoAlign extends CommandBase {
  Vision vision;
  SwerveSubsystem swerveSubsystem;

  private double tv;
  private double headingError;
  private ChassisSpeeds chassisSpeeds;

  /** Creates a new VisionTracking. */
  public AprilTagAutoAlign(SwerveSubsystem ss) {
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

    tv = vision.limeLightTargetCheck();

    if(tv == 1) {
      headingError = swerveSubsystem.getHeading() - 90;
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        0, 0, headingError, swerveSubsystem.getRotation2d());
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      swerveSubsystem.setModuleStates(moduleStates);
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
