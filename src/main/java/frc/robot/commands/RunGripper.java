// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class RunGripper extends CommandBase {
  /** Creates a new RunGripper. */
  Gripper gripper;
  Timer endTimer;

  public RunGripper(Gripper g) {
    gripper = g;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
    endTimer = new Timer();

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

    gripper.runGripper(1);
    //gripper.openGripper();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
   // gripper.closeGripper();
    SmartDashboard.putBoolean("Intake Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return (gripper.rangsensorGetVoltage() > 2) ? true : false;
    if(gripper.getGripperCurrent() > 7 & endTimer.get() > 1){
      return true;
    }
    else {
      return false;
    }
  }
}
