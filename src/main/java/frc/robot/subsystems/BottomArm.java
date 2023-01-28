// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PortConstants;

public class BottomArm extends SubsystemBase {
  /** Creates a new BottomArm. */
  CANSparkMax bottomArmMasterMotor;
  CANSparkMax bottomArmSlaveMotor;
  SparkMaxPIDController armPidController;
  public BottomArm() {
  bottomArmMasterMotor = new CANSparkMax(PortConstants.kBottomArmMasterMotorPort, MotorType.kBrushless);
  bottomArmSlaveMotor = new CANSparkMax(PortConstants.kBottomArmSlaveMotorPort, MotorType.kBrushless);
  bottomArmSlaveMotor.follow(bottomArmMasterMotor);

  bottomArmMasterMotor.restoreFactoryDefaults();
  bottomArmSlaveMotor.restoreFactoryDefaults();

  armPidController = bottomArmMasterMotor.getPIDController();

  armPidController.setP(ArmConstants.kPBottom);
  armPidController.setI(ArmConstants.kIBottom);
  armPidController.setD(ArmConstants.kDBottom);
  armPidController.setIZone(ArmConstants.kIntegralZoneBottom);
  armPidController.setFF(ArmConstants.kFeedForwardBottom);
  armPidController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

  armPidController.setSmartMotionMaxVelocity(ArmConstants.maxVelocity, ArmConstants.smartMotionSlot);
  armPidController.setSmartMotionMinOutputVelocity(ArmConstants.minVelocity, ArmConstants.smartMotionSlot);
  armPidController.setSmartMotionMaxAccel(ArmConstants.maxAcceleration, ArmConstants.smartMotionSlot);
  armPidController.setSmartMotionAllowedClosedLoopError(ArmConstants.allowedError, ArmConstants.smartMotionSlot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
