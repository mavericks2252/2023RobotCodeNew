// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

public class ConeFloorReverse extends CommandBase {
  Intake intake;
  Gripper gripper;
  Floor floor;
  TopArm topArm;
  BottomArm bottomArm;
  LEDModeSubsystem ledModeSubsystem;
  Timer endTimer;
  Timer reverseTimer;
  /** Creates a new ConeFloorReverse. */
  public ConeFloorReverse(Intake intake, Gripper gripper, Floor floor, TopArm topArm, BottomArm bottomArm, LEDModeSubsystem ledModeSubsystem) {
    this.intake = intake;
    this.gripper = gripper;
    this.floor = floor;
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.ledModeSubsystem = ledModeSubsystem;
    endTimer = new Timer();
    reverseTimer = new Timer();
    addRequirements(intake, gripper, topArm, bottomArm, floor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTimer.reset();
    endTimer.start();
    reverseTimer.reset();
    reverseTimer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ledModeSubsystem.getRobotMode() && reverseTimer.get() > .5){
      
      floor.reverseFloorMotor();
      intake.onlyRunIntake();
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    topArm.setMotorPosition(ArmConstants.kTopStowPosition);
    bottomArm.setMotorPosition(ArmConstants.kBottomStowPosition);
    intake.stopIntake();
    intake.retractIntake();
    floor.stopFloorMotor();

    if (!ledModeSubsystem.getRobotMode()) {
      gripper.gripperHoldCone();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ledModeSubsystem.getRobotMode()) {
      return true;
    }

    else {
      // Only look at current
      if(gripper.getGripperCurrent() > 15){
        //wait .5 seconds after current threshold to end    
        if(endTimer.get() > .5){
            ledModeSubsystem.startBlinking();
            return true;
            
        }
        //timer not expired return false
        else{
          return false;
        }
          }
      //keep reseting timer if current limit isnt hit
      else {
        endTimer.reset();
        return false;
        }
    }
    
  }
}
