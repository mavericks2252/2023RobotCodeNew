// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PortConstants;

public class Intake extends SubsystemBase {
  /** Creates a new SparkMax. */
  CANSparkMax intakeMotor;

  public Intake() {

    intakeMotor = new CANSparkMax(PortConstants.kIntakeMotorPort, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake() {

    intakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
  }

  public void stopIntake() {

    intakeMotor.stopMotor();
  }

}
