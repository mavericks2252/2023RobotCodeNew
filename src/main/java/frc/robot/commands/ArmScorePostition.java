// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.TopArm;

public class ArmScorePostition extends CommandBase {
  /** Creates a new ArmScorePostition. */
  TopArm topArm;
  BottomArm bottomArm;
  double bottomGoalPostition;
  double topGoalPosition;
  Double bottomArmError;
  Boolean topArmHold;
  Boolean bottomArmHold;
  

  public ArmScorePostition(double bottomGoalPosition, double topGoalPosition, BottomArm bottomArm, TopArm topArm) {
    this.bottomGoalPostition = bottomGoalPosition;
    this.topGoalPosition = topGoalPosition;
    this.topArm = topArm;
    this.bottomArm = bottomArm;

    addRequirements(bottomArm, topArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    topArm.setMotorPosition(56);
    bottomArm.setMotorPosition(ArmConstants.kBottomReversePosition); // Set the lower arm angle back
    

    topArmHold = true;
    bottomArmHold = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    bottomArmError = ArmConstants.kBottomReversePosition - bottomArm.getMotorEncoderPosition();

    SmartDashboard.putBoolean("Top Arm Hold", topArmHold);
    SmartDashboard.putBoolean("Bottom Arm hold", bottomArmHold);
    //SmartDashboard.putNumber("Bottom Arm Error", Math.abs(bottomArmError));*/

    if (Math.abs(bottomArmError) < 1) {
      topArmHold = false;
    }
    if (topArm.getMotorEncoderPosition() < (bottomGoalPostition * .4)) {//
      bottomArmHold = false;
    }

    if (!topArmHold) {
      topArm.setMotorPosition(topGoalPosition);
    }
    if (!bottomArmHold) {
      bottomArm.setMotorPosition(bottomGoalPostition);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    topArm.stopMotors();
    bottomArm.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
