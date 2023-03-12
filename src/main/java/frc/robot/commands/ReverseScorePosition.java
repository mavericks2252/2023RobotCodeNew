// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

public class ReverseScorePosition extends CommandBase {
  TopArm topArm;
  BottomArm bottomArm;
  Intake intake;
  LEDModeSubsystem ledModeSubsystem;

  double topArmGoal;
  double bottomArmGoal;
  double bottomHoldPosition;
  double reversePosition = 100;
  double bottomArmError;

  int node;
  int mid = 2;

  boolean topArmHold;
  boolean bottomArmHold;

  Timer intakeTimer;
  /** Creates a new ReverseScorePosition. */
  public ReverseScorePosition(int node, TopArm topArm, BottomArm bottomArm, Intake intake, LEDModeSubsystem ledModeSubsystem) {
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.intake = intake;
    this.ledModeSubsystem = ledModeSubsystem;
    this.node = node;
    intakeTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, topArm, bottomArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    topArm.setReverseScoring();
    intakeTimer.reset();
    intakeTimer.start();
    intake.extendIntake();
    bottomArm.setMotorPosition(reversePosition);

    topArmHold = true;
    bottomArmHold = true;

    if (node == mid && !ledModeSubsystem.getRobotMode()){// Mid node and in cone mode
      topArmGoal = 195;
      bottomArmGoal = 58;  
      bottomHoldPosition = 160;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bottomArmError = reversePosition - bottomArm.getMotorEncoderPosition();

    if (Math.abs(bottomArmError) < 1 && intakeTimer.get() > 1) {
      topArmHold = false;
    }
    if (topArm.getMotorEncoderPosition() > bottomHoldPosition) {
      bottomArmHold = false;
    }

    if (!topArmHold) {
      topArm.setMotorPosition(topArmGoal);
    }
    if (!bottomArmHold) {
      bottomArm.setMotorPosition(bottomArmGoal);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(bottomArm.getMotorEncoderPosition() - bottomArmGoal) < 1 && Math.abs(topArm.getMotorEncoderPosition() - topArmGoal) < 1){
      return true;
    }
    else {
      return false;
    }
  }
}
