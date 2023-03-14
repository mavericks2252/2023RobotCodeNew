// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PortConstants;

public class Intake extends SubsystemBase {
  /** Creates a new SparkMax. */
  WPI_TalonFX intakeMotor;
  DoubleSolenoid intakeSolenoid;
  StatorCurrentLimitConfiguration currentLimit = new StatorCurrentLimitConfiguration(true, 20, 40, .5);

  public Intake() {

    intakeMotor = new WPI_TalonFX(PortConstants.kIntakeMotorPort);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PortConstants.kIntakeSolenoidForwardChannel, PortConstants.kIntakeSolenoidReverseChannel);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {

    intakeMotor.set(ControlMode.PercentOutput, speed);
    extendIntake();
  }

  public void onlyRunIntake()  {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.kIntakeMotorSpeed);
  }

  public void extendIntake() {

    intakeSolenoid.set(Value.kForward);
  }

  public void retractIntake() {

    intakeSolenoid.set(Value.kReverse);
  }

  public void stopIntake() {

    intakeMotor.stopMotor();
  }

}
