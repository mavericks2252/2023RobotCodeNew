// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TopArm;

public class RaiseTopArm extends CommandBase {
  /** Creates a new RaiseTopArm. */
  TopArm topArm;
  double topArmGoal;
  double topArmError;
  public RaiseTopArm(TopArm topArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.topArm = topArm;
    addRequirements(topArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    topArmGoal = topArm.getMotorEncoderPosition() - 20; //set goal for top arm 20 degrees higher than current position

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    topArmError = topArmGoal - topArm.getMotorEncoderPosition();
    topArm.setMotorPosition(topArmGoal);// Sets the top arm to the position
    SmartDashboard.putNumber("raise arm error", topArmError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(topArmError) < 3){// If the arm is within 3 degrees of the goal it will stop the command
      return true;
  }
  else {
    return false;
  }
}

}
