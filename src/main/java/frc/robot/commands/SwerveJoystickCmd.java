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
  Supplier<Boolean> fieldOrientedFunction,  xStanceFunction;
  SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(
      SwerveSubsystem swerveSubsystem, // required subsystem
      Supplier<Double> xSpdFunction, //supplier for the x speed from joycestick
      Supplier<Double> ySpdFunction, //supplier for y speed from joycestick
      Supplier<Double> turningSpdFunction, //supplier for turning from joycestick
      Supplier<Boolean> fieldOrientedFunction, // button supplier to take robot out of field releative mode
      Supplier<Boolean> xStanceFunction) { // supplier for xstance to put the robot in xstance mode

      this.swerveSubsystem = swerveSubsystem;
      this.xSpdFunction = xSpdFunction;
      this.ySpdFunction = ySpdFunction;
      this.turningSpdFunction = turningSpdFunction;
      this.fieldOrientedFunction = fieldOrientedFunction;
      this.xStanceFunction = xStanceFunction;
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

        

    // Make driving smoother
    xSpeed = xLimiter.calculate(joystickDeadband(xSpeed)) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(joystickDeadband(ySpeed)) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(joystickDeadband(turningSpeed))
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
    
    if (xStanceFunction.get()){
    swerveSubsystem.setXStanceMode();
    
    }

    else {
    // Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
    }
  }

  public double joystickDeadband(Double speed) {
    if (Math.abs(speed) < Math.abs(OIConstants.kDeadBand)) return 0.0;
    return Math.signum(speed) * ((Math.abs(speed) - OIConstants.kDeadBand) / (1.0 - OIConstants.kDeadBand));}
    
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


// Old code to apply deadband
    /*    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadBand ? xSpeed : 0.0;
          ySpeed = Math.abs(ySpeed) > OIConstants.kDeadBand ? ySpeed : 0.0;
          turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadBand ? turningSpeed : 0.0;
        
           if (Math.abs(xSpeed) < Math.abs(OIConstants.kDeadBand)) return 0.0;
          return OIConstants.kDeadBand * Math.signum(xSpeed) + ((xSpeed - OIConstants.kDeadBand) / (1.0 - OIConstants.kDeadBand));
          if (Math.abs(ySpeed) < Math.abs(OIConstants.kDeadBand)) return 0.0;
          return OIConstants.kDeadBand * Math.signum(ySpeed) + ((ySpeed - OIConstants.kDeadBand) / (1.0 - OIConstants.kDeadBand));

          if (Math.abs(turningSpeed) < Math.abs(OIConstants.kDeadBand)) return 0.0;
          return OIConstants.kDeadBand * Math.signum(turningSpeed) + ((turningSpeed - OIConstants.kDeadBand) / (1.0 - OIConstants.kDeadBand));
    */  