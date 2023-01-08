// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule(
      PortConstants.kFrontLeftDriveMotorPort,
      PortConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveMotorInverted,
      DriveConstants.kFrontLeftTurningMotorInverted, 
      PortConstants.kFrontLeftAbsoluteEncoderPort, 
      DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad, 
      DriveConstants.kFrontLeftAbsoluteEncoderReversed);
      
  private final SwerveModule frontRight = new SwerveModule(
      PortConstants.kFrontRightDriveMotorPort, 
      PortConstants.kFrontRightTurningMotorPort, 
      DriveConstants.kFrontRightDriveMotorInverted, 
      DriveConstants.kFrontRightTurningMotorInverted, 
      PortConstants.kFrontRightAbsoluteEncoderPort, 
      DriveConstants.kFrontRightAbsoluteEncoderOffsetRad, 
      DriveConstants.kFrontRightAbsoluteEncoderReversed);
      
  private final SwerveModule backLeft = new SwerveModule(
      PortConstants.kbackLeftDriveMotorPort, 
      PortConstants.kbackLeftTurningMotorPort, 
      DriveConstants.kbackLeftDriveMotorInverted, 
      DriveConstants.kbackLeftTurningMotorInverted, 
      PortConstants.kbackLeftAbsoluteEncoderPort, 
      DriveConstants.kbackLeftAbsoluteEncoderOffsetRad, 
      DriveConstants.kbackLeftAbsoluteEncoderReversed);

  private final SwerveModule backRight = new SwerveModule(
      PortConstants.kbackRightDriveMotorPort, 
      PortConstants.kbackRightTurningMotorPort, 
      DriveConstants.kbackRightDriveMotorInverted, 
      DriveConstants.kbackRightTurningMotorInverted, 
      PortConstants.kbackRightAbsoluteEncoderPort, 
      DriveConstants.kbackRightAbsoluteEncoderOffsetRad, 
      DriveConstants.kbackRightAbsoluteEncoderReversed);

  private Pigeon2 gyro = new Pigeon2(PortConstants.kPigeonPort);
  

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> {
    try {
      Thread.sleep(1000);
    } catch (Exception e) {
    }
    }).start();


  }

    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   SmartDashboard.putNumber("RobotHeading", getHeading());
   SmartDashboard.putNumber("FL CANCoder", frontLeft.absoluteEncoder.getAbsolutePosition());
   SmartDashboard.putNumber("FR CANCoder", frontRight.absoluteEncoder.getAbsolutePosition());
   SmartDashboard.putNumber("BL CANCoder", backLeft.absoluteEncoder.getAbsolutePosition());
   SmartDashboard.putNumber("BR CANCoder", backRight.absoluteEncoder.getAbsolutePosition());
   SmartDashboard.putNumber("FL Turning Position", Units.radiansToDegrees(frontLeft.getTurningPosition()));
   SmartDashboard.putNumber("FR Turning Position", Units.radiansToDegrees(frontRight.getTurningPosition()));
   SmartDashboard.putNumber("BL Turning Position", Units.radiansToDegrees(backLeft.getTurningPosition()));
   SmartDashboard.putNumber("BR Turning Position", Units.radiansToDegrees(backRight.getTurningPosition()));
   SmartDashboard.putNumber("BR CANCoder Rads", backRight.getAbsoluteEncoderRad());
   SmartDashboard.putNumber("FR CANCoder Rads", frontRight.getAbsoluteEncoderRad());
   SmartDashboard.putNumber("FL CANCoder Rads", frontLeft.getAbsoluteEncoderRad());
   SmartDashboard.putNumber("BL CANCoder Rads", backLeft.getAbsoluteEncoderRad());
  }

  public void zeroHeading() {
    gyro.setYaw(0);
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

}
