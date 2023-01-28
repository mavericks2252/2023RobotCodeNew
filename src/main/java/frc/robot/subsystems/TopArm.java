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


public class TopArm extends SubsystemBase {
  /** Creates a new TopArm. */
  CANSparkMax topArmMotor;
  SparkMaxPIDController armPIDController;
 
  public TopArm() {
    topArmMotor = new CANSparkMax(PortConstants.kTopArmMotorPort, MotorType.kBrushless);

    topArmMotor.restoreFactoryDefaults();

    armPIDController = topArmMotor.getPIDController();

    armPIDController.setP(ArmConstants.kPTop);
    armPIDController.setI(ArmConstants.kITop);
    armPIDController.setD(ArmConstants.kDTop);
    armPIDController.setIZone(ArmConstants.kIntegralZoneTop);
    armPIDController.setFF(ArmConstants.kFeedForwardTop);
    armPIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

    armPIDController.setSmartMotionMaxVelocity(ArmConstants.maxVelocity, ArmConstants.smartMotionSlot);
    armPIDController.setSmartMotionMinOutputVelocity(ArmConstants.minVelocity, ArmConstants.smartMotionSlot);
    armPIDController.setSmartMotionMaxAccel(ArmConstants.maxAcceleration, ArmConstants.smartMotionSlot);
    armPIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.allowedError, ArmConstants.smartMotionSlot);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
