// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetScorePosition extends SequentialCommandGroup {
  SwerveSubsystem swerveSubsystem;
  TopArm topArm;
  BottomArm bottomArm;
  LEDModeSubsystem ledModeSubsystem;
  Intake intake;
  int node;
  /** Creates a new SetScorePosition. */
  public SetScorePosition(int node, TopArm topArm, BottomArm bottomArm, SwerveSubsystem swerveSubsystem, LEDModeSubsystem ledModeSubsystem, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    if (swerveSubsystem.getHeading() > -90 && swerveSubsystem.getHeading() < 90){// Intake facing away from driver station (forwards)
      addCommands(new ArmScorePostition(node, ledModeSubsystem, bottomArm, topArm));
    }
    else {// Intake facing towards driver station (backwards)
      addCommands(new ReverseScorePosition(node, topArm, bottomArm, intake, ledModeSubsystem));
    }
    
  }
}
