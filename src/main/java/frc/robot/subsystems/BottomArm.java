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
  // Setting motor equal to a new sparkmax 
  bottomArmMasterMotor = new CANSparkMax(PortConstants.kBottomArmMasterMotorPort, MotorType.kBrushless);
  bottomArmSlaveMotor = new CANSparkMax(PortConstants.kBottomArmSlaveMotorPort, MotorType.kBrushless);
  
  
  // Configuring the motors
  bottomArmMasterMotor.restoreFactoryDefaults();
  bottomArmSlaveMotor.restoreFactoryDefaults();
  bottomArmSlaveMotor.follow(bottomArmMasterMotor);
  bottomArmMasterMotor.setIdleMode(IdleMode.kBrake);
  bottomArmSlaveMotor.setIdleMode(IdleMode.kBrake);
  bottomArmMasterMotor.setClosedLoopRampRate(BottomArmConstants.kClosedLoopRampRate);
  bottomArmMasterMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  bottomArmMasterMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  bottomArmMasterMotor.setSoftLimit(SoftLimitDirection.kForward, 160);
  bottomArmMasterMotor.setSoftLimit(SoftLimitDirection.kReverse, 55);

  armPidController = bottomArmMasterMotor.getPIDController();

  

 
  absDutyCycleEncoder = new DutyCycleEncoder(0);
  

  relativeEncoder = bottomArmMasterMotor.getEncoder();
  
  relativeEncoder.setPositionConversionFactor(360 / BottomArmConstants.kGearRatio);

  // Configures pids on the motors
  armPidController.setP(BottomArmConstants.kP);
  armPidController.setI(BottomArmConstants.kI);
  armPidController.setD(BottomArmConstants.kD);
  armPidController.setIZone(BottomArmConstants.kIntegralZone);
  armPidController.setFF(BottomArmConstants.kFeedForward);
  armPidController.setOutputRange(BottomArmConstants.kMinOutput, BottomArmConstants.kMaxOutput);

  

  
  // Waits one second before running the method
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
    SmartDashboard.putNumber("Bottom Motor Encoder Position", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Bottom Absolute Encoder Position", encoderPositionAngle());
    SmartDashboard.putNumber("Bottom Absolute Position", absDutyCycleEncoder.getAbsolutePosition() * 360);
  }

  // Gets the position of the motor as an angle
  public Double encoderPositionAngle() {
    double angle = absDutyCycleEncoder.getAbsolutePosition() * 360;
    angle -= BottomArmConstants.kAbsEncoderOffset;
    return angle * (BottomArmConstants.kAbsEncoderReversed ? -1 : 1);

    
  }

  // Sets the motor encoder equal to the position of the absolute encoder
  public void resetEncoderPosition() {
    
    relativeEncoder.setPosition(encoderPositionAngle());
  }

  // Sets the motor to an angle
  public void setMotorPosition(double angle) {
    
    armPidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public double getMotorEncoderPosition() {
    return relativeEncoder.getPosition();
  }

  public void stopMotors() {
    bottomArmMasterMotor.stopMotor();
  }

  
}


/*alternateEncoder = bottomArmMasterMotor.getAbsoluteEncoder(Type.kDutyCycle);
  SmartDashboard.putNumber("Bottom Arm kP", armPidController.getP());
  SmartDashboard.putNumber("Bottom Arm kI", armPidController.getI());
  SmartDashboard.putNumber("Bottom Arm IZone", armPidController.getIZone());
  SmartDashboard.putNumber("Bottom Arm kD", armPidController.getD());
  armPidController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
  SmartDashboard.putNumber("Bottom Arm Percent Output", bottomArmMasterMotor.getAppliedOutput());
  armPidController.setP(SmartDashboard.getNumber("Bottom Arm kP", 0));
  armPidController.setI(SmartDashboard.getNumber("Bottom Arm kI", 0));
  armPidController.setIZone(SmartDashboard.getNumber("Bottom Arm IZone", 0));
  armPidController.setD(SmartDashboard.getNumber("Bottom Arm kD", 0));
  return absDutyCycleEncoder.getAbsolutePosition() * 360;
  */