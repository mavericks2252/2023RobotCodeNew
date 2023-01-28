// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PortConstants;

public class Intake extends SubsystemBase {
  /** Creates a new SparkMax. */
  WPI_TalonFX topIntakeMotor;
  WPI_TalonFX bottomIntakeMotor;

  public Intake() {

    topIntakeMotor = new WPI_TalonFX(PortConstants.kIntakeTopMotorPort);
    bottomIntakeMotor = new WPI_TalonFX(PortConstants.kIntakeBottomMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake() {

    topIntakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.kIntakeMotorSpeedtop);
    bottomIntakeMotor.set(ControlMode.PercentOutput, IntakeConstants.kIntakeMotorSpeedbottom);
  }

  public void stopIntake() {

    topIntakeMotor.stopMotor();
    bottomIntakeMotor.stopMotor();
  }

}
