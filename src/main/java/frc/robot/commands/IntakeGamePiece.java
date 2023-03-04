// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

public class IntakeGamePiece extends CommandBase {
  Intake intake;
  Gripper gripper;
  TopArm topArm;
  BottomArm bottomArm;
  LEDModeSubsystem ledModeSubsystem;
  double topArmError;
  double bottomArmError;
  Timer endTimer;
  /** Creates a new CubeIntake. */
  public IntakeGamePiece(Intake intake, Gripper gripper, TopArm topArm, BottomArm bottomArm, LEDModeSubsystem ledModeSubsystem) {
    this.intake = intake;
    this.gripper = gripper;
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.ledModeSubsystem = ledModeSubsystem;
    endTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, gripper, topArm, bottomArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTimer.reset();
    endTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake();
  
    // Cube mode
    if (ledModeSubsystem.getRobotMode()){
    gripper.runGripper();
    topArm.setMotorPosition(130);
    bottomArm.setMotorPosition(85);
    }

    // Cone mode
    else {
      topArm.setMotorPosition(90);// Placeholder value
      bottomArm.setMotorPosition(90);// Placeholder value
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //send arm to stow position
    topArm.setMotorPosition(ArmConstants.kStowPosition);
    bottomArm.setMotorPosition(ArmConstants.kStowPosition);
    intake.stopIntake();
    gripper.stopGripper();

    // Starting a new thread and waiting a period of time to retract the intake
    new Thread(() -> {
      try{
        Thread.sleep(500);
        intake.retractIntake(); 
    } catch(Exception e){}
    }).start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //end command if gripper motor exceeds current limit
    if(gripper.getGripperCurrent() > 7 & endTimer.get() > 1){
      return true;
    }
    else {
      return false;
    }
  }
}
