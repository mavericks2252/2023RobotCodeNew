// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  TalonFX indexerMotor;
  public Indexer() {
    indexerMotor = new TalonFX(PortConstants.kIndexerMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIndexer() {

    indexerMotor.set(ControlMode.PercentOutput, .1);
  }

  public void stopIndexer() {

    indexerMotor.set(ControlMode.PercentOutput, 0);
  }
}
