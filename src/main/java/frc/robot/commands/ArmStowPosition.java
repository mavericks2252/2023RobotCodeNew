// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.TopArm;

public class ArmStowPosition extends CommandBase {
  /** Creates a new ArmStowPosition. */
  TopArm topArm;
  BottomArm bottomArm;
  Boolean topArmHold;
  Boolean bottomArmHold;
  Double bottomArmError;
  public ArmStowPosition(BottomArm bottomArm, TopArm topArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    addRequirements(topArm, bottomArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bottomArm.setMotorPosition(ArmConstants.kBottomReversePosition);
    topArm.setMotorPosition(topArm.getMotorEncoderPosition()-10);
    topArmHold = true;
    bottomArmHold = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bottomArmError = ArmConstants.kBottomReversePosition - bottomArm.encoderPositionAngle();

    SmartDashboard.putBoolean("Stow Top Arm Hold", topArmHold);
    SmartDashboard.putBoolean("Stow Bottom Arm Hold", bottomArmHold);

    if (bottomArm.getMotorEncoderPosition() < 90) {
      topArmHold = false;
    }
    if (topArm.getMotorEncoderPosition() > 25) {
      bottomArmHold = false;
    }

    if (!topArmHold) {
      topArm.setMotorDownPosition(ArmConstants.kTopStowPosition);
    }
    if (!bottomArmHold) {
      bottomArm.setMotorPosition(ArmConstants.kBottomStowPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!topArmHold && !bottomArmHold){
      return true;
    }
    
    else {
    return false;
    }
  }
}
