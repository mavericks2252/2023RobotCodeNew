// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TopArm;

public class ArmStowPosition extends CommandBase {
  /** Creates a new ArmStowPosition. */
  TopArm topArm;
  BottomArm bottomArm;
  Boolean topArmHold;
  Boolean bottomArmHold;
  Double bottomArmError;
  Intake intake;
  public ArmStowPosition(BottomArm bottomArm, TopArm topArm, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.intake = intake;
    addRequirements(topArm, bottomArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // Sets the position back at the beginning
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

    if (bottomArm.getMotorEncoderPosition() < 90) {// Once the bottom goes under 90 degrees it will let the top move
      topArmHold = false;
    }
    if (topArm.getMotorEncoderPosition() > 25) {// Once the top goes above 25 it will let the bottom arm move
      bottomArmHold = false;
    }

    // Once it is no longer holding the arm they will set it to the stow position
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

     if (topArm.getScoringPosition()) {// Waits half of a secong to rettract the intake
      new Thread(() -> {
        try {
          Thread.sleep(500);
          intake.retractIntake();
        } catch (Exception e) {}
        }).start();
      
    }
    topArm.setTrueStowPosition();
    topArm.setRegularScoring();
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Will end the command once it is no longer holding the arm
    if (!topArmHold && !bottomArmHold){
      
      return true;
    }
    
    else {
    return false;
    }
  }
}
