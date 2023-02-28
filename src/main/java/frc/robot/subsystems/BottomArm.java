// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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
  
  

  bottomArmMasterMotor.restoreFactoryDefaults();
  bottomArmSlaveMotor.restoreFactoryDefaults();
  
  bottomArmSlaveMotor.follow(bottomArmMasterMotor);
  bottomArmMasterMotor.setIdleMode(IdleMode.kBrake);
  bottomArmSlaveMotor.setIdleMode(IdleMode.kBrake);
  //bottomArmMasterMotor.setClosedLoopRampRate(BottomArmConstants.kClosedLoopRampRate);
  bottomArmMasterMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  bottomArmMasterMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  bottomArmMasterMotor.setSoftLimit(SoftLimitDirection.kForward, 160);
  bottomArmMasterMotor.setSoftLimit(SoftLimitDirection.kReverse, 60);



  armPidController = bottomArmMasterMotor.getPIDController();

  //alternateEncoder = bottomArmMasterMotor.getAbsoluteEncoder(Type.kDutyCycle);
  absDutyCycleEncoder = new DutyCycleEncoder(0);
  

  relativeEncoder = bottomArmMasterMotor.getEncoder();
  
  relativeEncoder.setPositionConversionFactor(360 / BottomArmConstants.kGearRatio);


  armPidController.setP(BottomArmConstants.kP);
  armPidController.setI(BottomArmConstants.kI);
  armPidController.setD(BottomArmConstants.kD);
  armPidController.setIZone(BottomArmConstants.kIntegralZone);
  armPidController.setFF(BottomArmConstants.kFeedForward);
  armPidController.setOutputRange(BottomArmConstants.kMinOutput, BottomArmConstants.kMaxOutput);

  SmartDashboard.putNumber("Bottom Arm kP", armPidController.getP());
  SmartDashboard.putNumber("Bottom Arm kI", armPidController.getI());
  SmartDashboard.putNumber("Bottom Arm IZone", armPidController.getIZone());
  SmartDashboard.putNumber("Bottom Arm kD", armPidController.getD());

  
  
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
    SmartDashboard.putNumber("Bottom Absolute Position", absDutyCycleEncoder.getAbsolutePosition() * 360);
    SmartDashboard.putNumber("Bottom Arm Percent Output", bottomArmMasterMotor.getAppliedOutput());


    armPidController.setP(SmartDashboard.getNumber("Bottom Arm kP", 0));
    armPidController.setI(SmartDashboard.getNumber("Bottom Arm kI", 0));
    armPidController.setIZone(SmartDashboard.getNumber("Bottom Arm IZone", 0));
    armPidController.setD(SmartDashboard.getNumber("Bottom Arm kD", 0));
  }

  public Double encoderPositionAngle() {
    double angle = absDutyCycleEncoder.getAbsolutePosition() * 360;
    angle -= BottomArmConstants.kAbsEncoderOffset;
    return angle * (BottomArmConstants.kAbsEncoderReversed ? -1 : 1);

    //return absDutyCycleEncoder.getAbsolutePosition() * 360;
  }

  public void resetEncoderPosition() {
    
    relativeEncoder.setPosition(encoderPositionAngle());
  }

  public void setMotorPosition(double angle) {
    
    armPidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void stopMotors() {
    bottomArmMasterMotor.stopMotor();
  }

  
}
