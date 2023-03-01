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
import frc.robot.Constants.TopArmConstants;
import frc.robot.Constants.PortConstants;


public class TopArm extends SubsystemBase {
  /** Creates a new TopArm. */
  CANSparkMax topArmMotor;
  SparkMaxPIDController armPIDController;
  SparkMaxPIDController armDownPIDController;
  DutyCycleEncoder absDutyCycleEncoder;
  RelativeEncoder relativeEncoder;
  double armGoalPos;
  
 
  public TopArm() {
    topArmMotor = new CANSparkMax(PortConstants.kTopArmMotorPort, MotorType.kBrushless);

    topArmMotor.restoreFactoryDefaults();
    topArmMotor.setInverted(true);
    topArmMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    topArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    topArmMotor.setSoftLimit( SoftLimitDirection.kReverse, -35);
    topArmMotor.setSoftLimit(SoftLimitDirection.kForward, 200);
    topArmMotor.setIdleMode(IdleMode.kBrake);
    topArmMotor.setClosedLoopRampRate(TopArmConstants.kClosedLoopRampRate);
    //armPIDController = topArmMotor.getPIDController();

    absDutyCycleEncoder = new DutyCycleEncoder(1);
    
    
    relativeEncoder = topArmMotor.getEncoder();
    relativeEncoder.setPositionConversionFactor(360 / TopArmConstants.kGearRatio);
    
    armPIDController = topArmMotor.getPIDController();
    armPIDController.setP(TopArmConstants.kP);
    armPIDController.setI(TopArmConstants.kI);
    armPIDController.setD(TopArmConstants.kD);
    armPIDController.setIZone(TopArmConstants.kIntegralZone);
    armPIDController.setFF(TopArmConstants.kFeedForward);
    armPIDController.setOutputRange(TopArmConstants.kMinOutput, TopArmConstants.kMaxOutput);

   


    /*SmartDashboard.putNumber("Arm Goal Position", 0);

    SmartDashboard.putNumber("Top Arm kP", armPIDController.getP());
    SmartDashboard.putNumber("Top Arm kI", armPIDController.getI());
    SmartDashboard.putNumber("Top Arm kD", armPIDController.getD());
    SmartDashboard.putNumber("Top Arm IZone", armPIDController.getIZone());*/
    

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
    SmartDashboard.putNumber("Top Absolute Position", absDutyCycleEncoder.getAbsolutePosition() * 360);
    armGoalPos = SmartDashboard.getNumber("Arm Goal Position", 0);

    /*armPIDController.setP(SmartDashboard.getNumber("Top Arm kP", 0));
    armPIDController.setI(SmartDashboard.getNumber("Top Arm kI", 0));
    armPIDController.setD(SmartDashboard.getNumber("Top Arm kD", 0));
    armPIDController.setIZone(SmartDashboard.getNumber("Top Arm IZone", 0));

    SmartDashboard.putNumber("Acutal Top Arm kP", armPIDController.getP());*/

    


  }

  public Double encoderPositionAngle() {
    double angle = absDutyCycleEncoder.getAbsolutePosition() * 360;
    angle -= TopArmConstants.kAbsEncoderOffset;
    return angle * (TopArmConstants.kAbsEncoderReversed ? -1 : 1);
  }

  public void resetEncoderPosition() {

    relativeEncoder.setPosition(encoderPositionAngle());
  }

  public void setMotorPosition(double angle) {
    armPIDController = topArmMotor.getPIDController();
    
    armPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void setMotorDownPosition(double angle) {
    armDownPIDController = topArmMotor.getPIDController();
    armDownPIDController.setP(0.013);
    armDownPIDController.setI(0.00008);
    armDownPIDController.setD(0.0);
    armDownPIDController.setIZone(1.5);
    armDownPIDController.setFF(0);
    armDownPIDController.setOutputRange(-0.5, 0.5);
    

    armDownPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public double getMotorEncoderPosition() {
    return relativeEncoder.getPosition();
  }

  public void stopMotors() {
    topArmMotor.stopMotor();
  }

  
}
