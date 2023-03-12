// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreGamePieceCommand extends SequentialCommandGroup {
  /** Creates a new ScoreGamePieceCommand. */
  public ScoreGamePieceCommand(Gripper gripper, TopArm topArm, BottomArm bottomArm, LEDModeSubsystem ledModeSubsystem, Intake intake) {
     
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreGamePiece(gripper, topArm, bottomArm, ledModeSubsystem), 
      new ArmStowPosition(bottomArm, topArm, intake));
  }
}
