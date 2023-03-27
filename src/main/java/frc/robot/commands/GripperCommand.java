// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LEDModeSubsystem;

public class GripperCommand extends CommandBase {
  Supplier<Double> gripperSpeed;
  Gripper gripper;
  LEDModeSubsystem ledModeSubsystem;
  /** Creates a new Gripper. */
  public GripperCommand(Supplier<Double> gripperSpeed, Gripper gripper, LEDModeSubsystem ledModeSubsystem) {
    this.gripperSpeed = gripperSpeed;
    this.gripper = gripper;
    this.ledModeSubsystem = ledModeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ledModeSubsystem.getRobotMode()){
      if (gripperSpeed.get() < .1){
        gripper.gripperHoldCube();
      }

      else{
        gripper.runGripper(gripperSpeed.get());
      }
    }
    else{
      if(gripperSpeed.get() <.1){
        gripper.gripperHoldCone();
      }
      else{
        gripper.reverseGripper(gripperSpeed.get());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
