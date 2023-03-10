// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class AprilTagAutoAlign extends CommandBase {
  Vision vision;
  SwerveSubsystem swerveSubsystem;

  private double tv;
  private double tx;
  private double ta;
  private double headingError;
  private ChassisSpeeds chassisSpeeds;
  private double ySpeed;
  private double xSpeed;
  private double turningSpeed;
  private double targetHeading = 180;
  private double targetArea = 1.75;
  SlewRateLimiter turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
  SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);


  /** Creates a new VisionTracking. */
  public AprilTagAutoAlign(SwerveSubsystem ss, Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSubsystem = ss;
    vision = v;
    
    addRequirements(swerveSubsystem, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = vision.limeLightTargetCheck();
    tx = vision.limeLightHorizontalPosition();
    ta = vision.limeLightTargetArea();

    if(tv == 1) {//
      headingError = targetHeading - Math.abs(swerveSubsystem.getHeading());
      headingError = Math.copySign(headingError, swerveSubsystem.getHeading());
      //If heading error is greater than assceptable error 
      //mupliply the heading error by a kP to get a turning speed
      
      if (Math.abs(headingError) > 1.5) {

        turningSpeed = headingError * AutoConstants.kPAlignmentTheta;
        
          
          if (Math.abs(turningSpeed) > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond) {
            
            turningSpeed = Math.copySign(DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond, turningSpeed);
          }

          else if (Math.abs(turningSpeed) < DriveConstants.kMinimumTurningSpeed) {

            turningSpeed = Math.copySign(DriveConstants.kMinimumTurningSpeed, turningSpeed);
          }
        
      }

      else {
        turningSpeed = 0;
      }

      
      if (Math.abs(tx) > 1) {

        ySpeed = tx * AutoConstants.kPAlignmentY;

          if (Math.abs(ySpeed) > DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) {

            ySpeed = Math.copySign(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, ySpeed);
          }
      }

      else {
        ySpeed = 0;
      }


      if (ta < targetArea) {

        xSpeed = (targetArea - ta) * AutoConstants.kPAlignmentX;

          if (xSpeed > DriveConstants.kTeleDriveMaxSpeedMetersPerSecond) {
            xSpeed = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
          }

          else if (xSpeed < DriveConstants.kMinimumDriveSpeed) {

            xSpeed = DriveConstants.kMinimumDriveSpeed;
          }
      }

      else {
        xSpeed = 0;
      }

      
    } 

    else {
      turningSpeed = 0;
      ySpeed = 0;
      xSpeed = 0;
    }
    

    //Need an else statement for setting all the speeds to 0
      SmartDashboard.putNumber("Heading Error", headingError);

    //Moved this out here so it assigns a value to the drive motors everytime through the program
    turningSpeed = turningLimiter.calculate(turningSpeed);
    ySpeed = ySpeedLimiter.calculate(ySpeed);
    xSpeed = xSpeedLimiter.calculate(xSpeed);

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        //*******Then use this turning speed to calculate the chassis speeds
        -xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop modules when it ends so we dont leave any old speeds in there while tranisitoning to joystick command
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
