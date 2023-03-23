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
            2, 
            2));
            
      List<PathPlannerTrajectory> twoPieceLevel = 
        PathPlanner.loadPathGroup(
          "Two Piece Level", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));
      
      List<PathPlannerTrajectory> threePiece = 
        PathPlanner.loadPathGroup(
          "Three Piece Score", 
          new PathConstraints(
            3.05, 
            3.5));

      List<PathPlannerTrajectory> threeAndHalfPiece = 
        PathPlanner.loadPathGroup(
          "Three Piece Drive", 
          new PathConstraints(
            3.25, 
            3.5));

    List<PathPlannerTrajectory> threePieceLevel = 
        PathPlanner.loadPathGroup(
          "Three Piece Level", 
          new PathConstraints(
            3.3, 
            4));


       List<PathPlannerTrajectory> twoAndHalfPlus= 
        PathPlanner.loadPathGroup(
          "2.5 Piece Level", 
          new PathConstraints(
            3.05, 
            3.5));

      List<PathPlannerTrajectory> onlyScoreMid = 
        PathPlanner.loadPathGroup(
          "Only Score Mid", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));

      List<PathPlannerTrajectory> twoPieceScoreRight = 
        PathPlanner.loadPathGroup(
          "Two Piece Score Cable", 
          new PathConstraints(
            2, 
            2));
      
      List<PathPlannerTrajectory> twoPiecePlusCable = 
        PathPlanner.loadPathGroup(
          "Two Piece Plus Cable", 
          new PathConstraints(
            2, 
            2));

      List<PathPlannerTrajectory> onePiecePlusMidBalance = 
        PathPlanner.loadPathGroup(
          "One Peice Plus Mid", 
          new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared));




      autoChooser = new SendableChooser<>();
      autoChooser.addOption("Do Nothing", doNothing);
      autoChooser.addOption("One Piece Plus", onePiecePlus);
      autoChooser.addOption("Two Piece Level", twoPieceLevel);
      autoChooser.addOption("Three Piece Score", threePiece);
      autoChooser.addOption("3.5 Piece Score", threeAndHalfPiece);
      autoChooser.addOption("Three Piece Level", threePieceLevel);
      autoChooser.addOption("Only Score Mid", onlyScoreMid);
      autoChooser.addOption("2.5 Piece Level", twoAndHalfPlus);
      autoChooser.addOption("Two Piece Score Cable", twoPieceScoreRight);
      autoChooser.addOption("Two Piece Plus Cable", twoPiecePlusCable);
      autoChooser.addOption("One Piece Plus Mid", onePiecePlusMidBalance);
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
