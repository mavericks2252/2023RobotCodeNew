// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // May want to measure the wheels to see if they are exactly 4" diameter
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 21.4285714;
        public static final double kDriveEncoderRotToMeter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRotToRad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kPTurning = 0.825; //Will need tuning

        // What are these two?  Are we using them?
        public static final double kDriveEncoderRPMToMeterPerSec = kDriveEncoderRotToMeter / 60; 
        public static final double kTurningEncoderRPMToRadPerSec = kTurningEncoderRotToRad / 60;
        
        

    }

    public static final class MotorConstants {

        public static final double kFalconTicksPerRev = 2048;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(24.25); //Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(21.50); //Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
         new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
         new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
         new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back Left
         new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back Right

        
        public static final boolean kFrontLeftDriveMotorInverted = false;
        public static final boolean kFrontLeftTurningMotorInverted = true;
        public static final double kFrontLeftAbsoluteEncoderOffsetRad = Units.degreesToRadians(108.984);
        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;// Changed to False from True 2/16/23 ... Motor Not following direction it should

        public static final boolean kFrontRightDriveMotorInverted = false;
        public static final boolean kFrontRightTurningMotorInverted = true;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = Units.degreesToRadians(53.523);
        public static final boolean kFrontRightAbsoluteEncoderReversed = true;

        public static final boolean kbackLeftDriveMotorInverted = false;
        public static final boolean kbackLeftTurningMotorInverted = true;
        public static final double kbackLeftAbsoluteEncoderOffsetRad = Units.degreesToRadians(162.509);
        public static final boolean kbackLeftAbsoluteEncoderReversed = true;

        public static final boolean kbackRightDriveMotorInverted = true;
        public static final boolean kbackRightTurningMotorInverted = true;
        public static final double kbackRightAbsoluteEncoderOffsetRad = Units.degreesToRadians(76.816);
        public static final boolean kbackRightAbsoluteEncoderReversed = true;

        

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(16.3);
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 15;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 4;//10
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 5;//15

        public static final double kMinimumTurningSpeed = .75;
        public static final double kMinimumDriveSpeed = .3;
    }

    public static final class IntakeConstants {

        public static final double kIntakeMotorSpeedtop = -.6;
        public static final double kIntakeMotorSpeedbottom = .7;
    }

    public static final class TopArmConstants {

        public static final double kP = 0.165;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kIntegralZone = 0;
        public static final double kFeedForward = 0;
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;
        public static final int kSmartMotionSlot = 0;
        public static final double kAllowedError = 5;
        public static final double kMaxAcceleration = 5;
        public static final double kMinVelocity = 0;
        public static final double kMaxVelocity = 5;

        public static final double kGearRatio = 72 / 10 * 68 / 18 * 60 / 26 * 36 / 15;
        public static final Double kStowPosition = null;


    }

    public static final class BottomArmConstants {
        
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kIntegralZone = 0;
        public static final double kFeedForward = 0;
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;
        public static final int smartMotionSlot = 0;
        public static final double allowedError = 0;
        public static final double maxAcceleration = 500;
        public static final double minVelocity = 0;
        public static final double maxVelocity = 60;
        
        public static final double kGearRatio = 64 * 72 * 80 * 48 / 10 / 18 / 24 / 18;
        public static final Double kStowPosition = null;


    }

    public static final class GripperConstants {

        public static final double gripperMotorSpeed = 0.75;

    }

    public static final class OIConstants {

        public static final double kDeadBand = .15;
        public static final int kOperatorControllerPort = 0;
        public static final int kDriverControllerPort = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 4;

        // Controller Buttons
        public static final int xButton = 3;
        public static final int aButton = 1;
        public static final int bButton = 2;
        public static final int yButton = 4;
        public static final int rbButton = 6;
        public static final int lbButton = 5;
    }         

    public static final class AutoConstants {

        
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        
        public static final double kPThetaController = 4;
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
                    new TrapezoidProfile.Constraints
                        (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                         DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        public static final PIDConstants kXYController = new PIDConstants(2, 0, 0);
        public static final PIDConstants kThetaController = new PIDConstants(kPThetaController, 0, 0);
        public static final double kPAlignmentTheta = 0.115;
        public static final double kPAlignmentY = 0.05;
        public static final double kPAlignmentX = 0.8;
        public static final double kAutoBalanceSpeed = .75;

        
        /*public static final ProfiledPIDController kThetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);*/
        //public static final double kPXController = 2;
        //public static final double kPYController = 2.5;
        //public static final PIDController kxController = new PIDController(kPXController, 0, 0);
        //public static final PIDController kyController = new PIDController(kPYController, 0, 0);
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

        public static final int kIntakeMotorPort = 14;

        public static final int kIndexerMotorPort = 15;


        public static final int kPigeonPort = 13;
        public static final int kIntakeTopMotorPort = 16;
        public static final int kIntakeBottomMotorPort = 17;
        
       
        public static final int kBottomArmMasterMotorPort = 0;
        public static final int kBottomArmSlaveMotorPort = 19;
        public static final int kTopArmMotorPort = 21;//18
        public static final int kGripperMotorPort = 18;//21
        public static final int kGripperSolenoidPort = 0;

    
    
    }

}
