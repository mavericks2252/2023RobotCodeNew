// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class ModuleConstants {

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.4285714;
        public static final double kDriveEncoderRotToMeter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRotToRad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPMToMeterPerSec = kDriveEncoderRotToMeter / 60;
        public static final double kTurningEncoderRPMToRadPerSec = kTurningEncoderRotToRad / 60;
        public static final double kPTurning = 0.5; //Will need tuning

    }

    public static final class MotorConstants {

        public static final double kFalconTicksPerRev = 2048;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(24.25); //Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24.25); //Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
         new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
         new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
         new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back Left
         new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back Right

        
        public static final boolean kFrontLeftDriveMotorInverted = false;
        public static final boolean kFrontLeftTurningMotorInverted = true;
        public static final double kFrontLeftAbsoluteEncoderOffsetRad = Units.degreesToRadians(322.822266);
        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;

        public static final boolean kFrontRightDriveMotorInverted = true;
        public static final boolean kFrontRightTurningMotorInverted = true;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = Units.degreesToRadians(347.607422);
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;

        public static final boolean kbackRightDriveMotorInverted = true;
        public static final boolean kbackRightTurningMotorInverted = true;
        public static final double kbackRightAbsoluteEncoderOffsetRad = Units.degreesToRadians(252.158203);
        public static final boolean kbackRightAbsoluteEncoderReversed = false;

        public static final boolean kbackLeftDriveMotorInverted = false;
        public static final boolean kbackLeftTurningMotorInverted = true;
        public static final double kbackLeftAbsoluteEncoderOffsetRad = Units.degreesToRadians(19.3359);
        public static final boolean kbackLeftAbsoluteEncoderReversed = false;

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(16.3);
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 15;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 10;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 15;
    }

    public static final class OIConstants {

        public static final double kDeadBand = 0.15;
        public static final int kDriverControllerPort = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 4;
  

    }

    public static final class PortConstants {

    // Controllers
        public static final int kOperatorControllerPort = 0;
        public static final int kDriverControllerPort = 1;

    // Swerve
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontLeftAbsoluteEncoderPort = 3;
        
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kFrontRightAbsoluteEncoderPort = 6;

        public static final int kbackLeftDriveMotorPort = 7;
        public static final int kbackLeftTurningMotorPort = 8;
        public static final int kbackLeftAbsoluteEncoderPort = 9;

        public static final int kbackRightDriveMotorPort = 10;
        public static final int kbackRightTurningMotorPort = 11;
        public static final int kbackRightAbsoluteEncoderPort = 12;


        public static final int kPigeonPort = 13;

    // Controller Buttons
        public static final int xButton = 3;
        public static final int aButton = 1;
        public static final int bButton = 2;
        public static final int yButton = 4;
    
    }

}
