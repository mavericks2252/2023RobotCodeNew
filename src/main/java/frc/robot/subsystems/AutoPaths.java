// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

public class AutoPaths extends SubsystemBase {
  

  SendableChooser<List <PathPlannerTrajectory>> autoChooser;
  /** Creates a new AutoPaths. */
  public AutoPaths() {
    List<PathPlannerTrajectory> doNothing = 
        PathPlanner.loadPathGroup(
          "do nothing", 
          new PathConstraints(0, 0));

    List<PathPlannerTrajectory> twoPieceRightSide = 
        PathPlanner.loadPathGroup(
          "Path 1", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));

      List<PathPlannerTrajectory> threePieceRightSide = 
        PathPlanner.loadPathGroup(
          "Path 2", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));
            
      List<PathPlannerTrajectory> twoPieceLeftSide = 
        PathPlanner.loadPathGroup(
          "Path 3", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));

      List<PathPlannerTrajectory> twoPiecePlusLeftSide = 
        PathPlanner.loadPathGroup(
          "Path 4", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));




      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("Do Nothing", doNothing);
      autoChooser.addOption("Path 1", twoPieceRightSide);
      autoChooser.addOption("Path 2", threePieceRightSide);
      autoChooser.addOption("Path 3", twoPieceLeftSide);
      autoChooser.addOption("Path 4", twoPiecePlusLeftSide);

      SmartDashboard.putData(autoChooser);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public List<PathPlannerTrajectory> getSelectedAuto() {
    return autoChooser.getSelected();
  }
  
}
