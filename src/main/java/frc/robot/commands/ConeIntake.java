// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BottomArmConstants;
import frc.robot.Constants.TopArmConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TopArm;

public class ConeIntake extends CommandBase {
  Intake intake;
  Gripper gripper;
  TopArm topArm;
  BottomArm bottomArm;
  double topArmError;
  double bottomArmError;
  /** Creates a new ConeIntake. */
  public ConeIntake(Intake intake, Gripper gripper, TopArm topArm, BottomArm bottomArm) {
    this.intake = intake;
    this.gripper = gripper;
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, gripper, topArm, bottomArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake();

    gripper.runGripper();
    
    topArm.setMotorPosition(100);
    bottomArm.setMotorPosition(75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    topArm.setMotorPosition(TopArmConstants.kStowPosition);
    bottomArm.setMotorPosition(BottomArmConstants.kStowPosition);
    topArmError = TopArmConstants.kStowPosition - topArm.encoderPositionAngle();
    bottomArmError = BottomArmConstants.kStowPosition - bottomArm.encoderPositionAngle();

    intake.stopIntake();
    gripper.stopGripper();

    if (Math.abs(topArmError) < 1) {
      topArm.stopMotors();
      intake.retractIntake();

    }
    if (Math.abs(bottomArmError) < 1) {
      bottomArm.stopMotors();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (gripper.rangsensorGetVoltage() > 2.0) ? true : false;
  }
}
