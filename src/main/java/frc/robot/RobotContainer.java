// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprilTagAutoAlign;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SpinIndexer;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Vision vision = new Vision();
  private final AprilTagAutoAlign aprilTagAutoAlign = new AprilTagAutoAlign(swerveSubsystem, vision);
  private final Intake intake = new Intake();
  private final RunIntake runIntake = new RunIntake(intake);
  private final Indexer indexer = new Indexer();
  private final SpinIndexer spinIndexer = new SpinIndexer(indexer);

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
          swerveSubsystem,
          () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
          () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
          () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis), 
          //Change the button to yButton and delete the constant that is used here for consistancy
          () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    // Configure the button bindings
    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      new JoystickButton(driverJoystick, OIConstants.bButton).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
      new JoystickButton(driverJoystick, OIConstants.aButton).whileTrue(aprilTagAutoAlign);
      new JoystickButton(driverJoystick, OIConstants.xButton).whileTrue(runIntake);
      new JoystickButton(driverJoystick, OIConstants.yButton).whileTrue(spinIndexer);

      
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      
      /*
        ***DO NOT DELETE THIS LINE
        ***Place auto Commands AFTER this line
        Enabling Continuous Input on Rotational PID Controler to pass through -180 to 180 degrees (in Radians)
      */
      AutoConstants.kThetaController.enableContinuousInput(-Math.PI, Math.PI);
      
      
      SmartDashboard.putBoolean("Passed Marker 1", false); // Used for testing passing a marker


      PathPlannerTrajectory testPath = PathPlanner.loadPath("New Path", // Configuring name of the path
      new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, //Max speed for path
        AutoConstants.kMaxAccelerationMetersPerSecondSquared //Max acceleration for path
        ));// New path constraints

     // Sampling the velocity at a given time in program
      PathPlannerState exampleState = (PathPlannerState) testPath.sample(1.2);
      SmartDashboard.putNumber("Path Velocity", exampleState.velocityMetersPerSecond);
     
      
     //Creating the eventmap for markers in auto program
      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("marker1", new InstantCommand (() -> SmartDashboard.putBoolean("Passed Marker 1", true)));

      eventMap.put("event", new WaitCommand(1));

      FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
        new PPSwerveControllerCommand (
        testPath,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        AutoConstants.kxController,
        AutoConstants.kyController,
        AutoConstants.kThetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem), 
        testPath.getMarkers(), 
        eventMap);


      return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(testPath.getInitialHolonomicPose())),
        pathWithEvents,
        new InstantCommand(() -> swerveSubsystem.stopModules()));


  }
}

/* Code used while learning Path Planning

// Create trajectory settings
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
      AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

      // Generate trajectory
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
               List.of(
                  new Translation2d(1, 0),
                  new Translation2d(1,-1)
               ), new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig); 
                
    // Construct command to follow trajectory
       SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                  testPath, 
                  swerveSubsystem::getPose, 
                  DriveConstants.kDriveKinematics, 
                  AutoConstants.kxController, 
                  AutoConstants.kyController, 
                  AutoConstants.kThetaController, 
                  swerveSubsystem::setModuleStates, 
                  swerveSubsystem);
      return null;

      // Add some init and wrap-up, and return evrything
      
      );
      ;
      
      
      PPSwerveControllerCommand testAuto = new PPSwerveControllerCommand (
        testPath,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        AutoConstants.kxController,
        AutoConstants.kyController,
        AutoConstants.kThetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);*/
