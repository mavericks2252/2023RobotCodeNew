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

    List<PathPlannerTrajectory> onePiecePlus = 
        PathPlanner.loadPathGroup(
          "One Piece Plus", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));
            
      List<PathPlannerTrajectory> twoPieceLevel = 
        PathPlanner.loadPathGroup(
          "Two Piece Level", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));

      List<PathPlannerTrajectory> twoPiecePlus = 
        PathPlanner.loadPathGroup(
          "Two Piece Plus", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            3.2));
      
      List<PathPlannerTrajectory> threePiece = 
        PathPlanner.loadPathGroup(
          "Two Piece Level Copy", 
          new PathConstraints(
            3.05, 
            3.5));

      




      autoChooser = new SendableChooser<>();
      autoChooser.addOption("Do Nothing", doNothing);
      autoChooser.addOption("One Piece Plus", onePiecePlus);
      autoChooser.addOption("Two Piece Level", twoPieceLevel);
      autoChooser.addOption("Two Piece Plus", twoPiecePlus);
      autoChooser.addOption("Three Piece", threePiece);
      autoChooser.setDefaultOption("Do Nothing", doNothing);

      SmartDashboard.putData("Selected Auto", autoChooser);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public List<PathPlannerTrajectory> getSelectedAuto() {
    return autoChooser.getSelected();
  }
  
}
