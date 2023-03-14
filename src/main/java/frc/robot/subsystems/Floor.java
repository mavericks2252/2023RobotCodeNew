// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;


public class Floor extends SubsystemBase {
  CANSparkMax floorMotor;
  /** Creates a new Floor. */
  public Floor() {
    floorMotor = new CANSparkMax(PortConstants.kFloorMotorPort, MotorType.kBrushless);
    floorMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runFloorMotor() {
    floorMotor.set(.4);
  }

  public void reverseFloorMotor() {
    floorMotor.set(-.6);
  }

  public void stopFloorMotor() {
    floorMotor.stopMotor();
  }
}
