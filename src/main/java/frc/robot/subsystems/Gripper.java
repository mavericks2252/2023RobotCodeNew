// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.PortConstants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  CANSparkMax gripperMotor;
  /** Creates a new Gripper. */
  public Gripper() {

    gripperMotor = new CANSparkMax(PortConstants.kGripperMotorPort, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runGripper() {

    gripperMotor.set(GripperConstants.gripperMotorSpeed);
  }

  public void reverseGripper() {

    gripperMotor.set(-GripperConstants.gripperMotorSpeed);
  }

  public void stopGripper() {
    gripperMotor.stopMotor();
  }
}
