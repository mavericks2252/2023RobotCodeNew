// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.PortConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  CANSparkMax gripperMotor;
  AnalogInput rangeSensor;


  /** Creates a new Gripper. */
  public Gripper() {

    gripperMotor = new CANSparkMax(PortConstants.kGripperMotorPort, MotorType.kBrushless);
    rangeSensor = new AnalogInput(1);
    
    


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Range Sensor Value", rangsensorGetVoltage());
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

  public double rangsensorGetVoltage() {
    return rangeSensor.getVoltage();
  }
}
