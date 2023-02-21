// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BottomArmConstants;
import frc.robot.Constants.PortConstants;

public class BottomArm extends SubsystemBase {
  /** Creates a new BottomArm. */
  CANSparkMax bottomArmMasterMotor;
  CANSparkMax bottomArmSlaveMotor;
  SparkMaxPIDController armPidController;
  RelativeEncoder relativeEncoder;
  DutyCycleEncoder absDutyCycleEncoder;
  
  
  
  public BottomArm() {
  bottomArmMasterMotor = new CANSparkMax(PortConstants.kBottomArmMasterMotorPort, MotorType.kBrushless);
  bottomArmSlaveMotor = new CANSparkMax(PortConstants.kBottomArmSlaveMotorPort, MotorType.kBrushless);
  bottomArmSlaveMotor.follow(bottomArmMasterMotor);
  

  bottomArmMasterMotor.restoreFactoryDefaults();
  bottomArmSlaveMotor.restoreFactoryDefaults();

  armPidController = bottomArmMasterMotor.getPIDController();

  //alternateEncoder = bottomArmMasterMotor.getAbsoluteEncoder(Type.kDutyCycle);
  absDutyCycleEncoder = new DutyCycleEncoder(2);

  relativeEncoder = bottomArmMasterMotor.getEncoder();
  
  relativeEncoder.setPositionConversionFactor(360 / BottomArmConstants.kGearRatio);


  armPidController.setP(BottomArmConstants.kP);
  armPidController.setI(BottomArmConstants.kI);
  armPidController.setD(BottomArmConstants.kD);
  armPidController.setIZone(BottomArmConstants.kIntegralZone);
  armPidController.setFF(BottomArmConstants.kFeedForward);
  armPidController.setOutputRange(BottomArmConstants.kMinOutput, BottomArmConstants.kMaxOutput);

  armPidController.setSmartMotionMaxVelocity(BottomArmConstants.maxVelocity, BottomArmConstants.smartMotionSlot);
  armPidController.setSmartMotionMinOutputVelocity(BottomArmConstants.minVelocity, BottomArmConstants.smartMotionSlot);
  armPidController.setSmartMotionMaxAccel(BottomArmConstants.maxAcceleration, BottomArmConstants.smartMotionSlot);
  armPidController.setSmartMotionAllowedClosedLoopError(BottomArmConstants.allowedError, BottomArmConstants.smartMotionSlot);
  
  new Thread(() -> {
    try {
      Thread.sleep(1000);
      resetEncoderPosition();
    } catch (Exception e) {
    }
    }).start();

  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //armPidController.setReference(0, CANSparkMax.ControlType.kSmartMotion);

    SmartDashboard.putNumber("Bottom Motor Encoder Position", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Bottom Absolute Encoder Position", encoderPositionAngle());
    SmartDashboard.putNumber("Bottom Absolute Position", absDutyCycleEncoder.getAbsolutePosition());
  }

  public Double encoderPositionAngle() {

    return absDutyCycleEncoder.getAbsolutePosition() * 360;
  }

  public void resetEncoderPosition() {
    
    relativeEncoder.setPosition(encoderPositionAngle());
  }

  public void setMotorPosition(Double angle) {
    
    armPidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void stopMotors() {
    bottomArmMasterMotor.stopMotor();
  }

  
}
