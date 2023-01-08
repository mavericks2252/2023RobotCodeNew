// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  Supplier<Boolean> fieldOrientedFunction;
  SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, 
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, 
      Supplier<Boolean> fieldOrientedFunction) {

      this.swerveSubsystem = swerveSubsystem;
      this.xSpdFunction = xSpdFunction;
      this.ySpdFunction = ySpdFunction;
      this.turningSpdFunction = turningSpdFunction;
      this.fieldOrientedFunction = fieldOrientedFunction;
      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

      addRequirements(swerveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    // Apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadBand ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadBand ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadBand ? turningSpeed : 0.0;

    // Make driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
    SmartDashboard.putNumber("TurningSpeedValue", turningSpeed);

    // Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else {
        // Relative to robot
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
