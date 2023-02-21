// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BottomArmConstants;
import frc.robot.Constants.TopArmConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.TopArm;

public class ArmStowPosition extends CommandBase {
  /** Creates a new ArmStowPosition. */
  TopArm topArm;
  BottomArm bottomArm;
  Boolean topArmHold;
  Boolean bottomArmHold;
  Double bottomArmError;
  public ArmStowPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(topArm, bottomArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bottomArm.setMotorPosition(45.0);
    topArmHold = true;
    bottomArmHold = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bottomArmError = BottomArmConstants.kStowPosition - bottomArm.encoderPositionAngle();

    if (Math.abs(bottomArmError) < 1) {
      topArmHold = false;
    }
    if (topArm.encoderPositionAngle() < 30) {
      bottomArmHold = false;
    }

    if (!topArmHold) {
      topArm.setMotorPosition(TopArmConstants.kStowPosition);
    }
    if (!bottomArmHold) {
      bottomArm.setMotorPosition(BottomArmConstants.kStowPosition);
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
