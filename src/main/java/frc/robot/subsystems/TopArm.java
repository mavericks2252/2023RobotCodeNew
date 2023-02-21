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
import frc.robot.Constants.TopArmConstants;
import frc.robot.Constants.PortConstants;


public class TopArm extends SubsystemBase {
  /** Creates a new TopArm. */
  CANSparkMax topArmMotor;
  SparkMaxPIDController armPIDController;
  DutyCycleEncoder absDutyCycleEncoder;
  RelativeEncoder relativeEncoder;
  Double armGoalPos;
 
  public TopArm() {
    topArmMotor = new CANSparkMax(PortConstants.kTopArmMotorPort, MotorType.kBrushless);

    topArmMotor.restoreFactoryDefaults();

    armPIDController = topArmMotor.getPIDController();

    absDutyCycleEncoder = new DutyCycleEncoder(1);
    
    relativeEncoder = topArmMotor.getEncoder();
    relativeEncoder.setPositionConversionFactor(360 / TopArmConstants.kGearRatio);

    armPIDController.setP(TopArmConstants.kP);
    armPIDController.setI(TopArmConstants.kI);
    armPIDController.setD(TopArmConstants.kD);
    armPIDController.setIZone(TopArmConstants.kIntegralZone);
    armPIDController.setFF(TopArmConstants.kFeedForward);
    armPIDController.setOutputRange(TopArmConstants.kMinOutput, TopArmConstants.kMaxOutput);

    armPIDController.setSmartMotionMaxVelocity(TopArmConstants.kMaxVelocity, TopArmConstants.kSmartMotionSlot);
    armPIDController.setSmartMotionMinOutputVelocity(TopArmConstants.kMinVelocity, TopArmConstants.kSmartMotionSlot);
    armPIDController.setSmartMotionMaxAccel(TopArmConstants.kMaxAcceleration, TopArmConstants.kSmartMotionSlot);
    armPIDController.setSmartMotionAllowedClosedLoopError(TopArmConstants.kAllowedError, TopArmConstants.kSmartMotionSlot);

    SmartDashboard.putNumber("Arm Goal Position", 0);
    

    new Thread(() -> {
      try {
        Thread.sleep(2000);
        resetEncoderPosition();
      } catch (Exception e) {
      }
      }).start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Motor Encoder Position", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Top Absolute Encoder Position", encoderPositionAngle());
    SmartDashboard.putNumber("Top Absolute Position", absDutyCycleEncoder.getAbsolutePosition());
    armGoalPos = SmartDashboard.getNumber("Arm Goal Position", 0);
  }

  public Double encoderPositionAngle() {

    return absDutyCycleEncoder.getAbsolutePosition() * 360;
  }

  public void resetEncoderPosition() {

    relativeEncoder.setPosition(encoderPositionAngle());
  }

  public void setMotorPosition(Double angle) {
    
    armPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void stopMotors() {
    topArmMotor.stopMotor();
  }

  
}
