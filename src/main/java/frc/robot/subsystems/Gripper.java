// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import frc.robot.Constants.PortConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  CANSparkMax gripperMotor;
  DigitalInput beamBreakSensor;
  DoubleSolenoid gripperSolenoid;


  /** Creates a new Gripper. */
  public Gripper() {

    gripperMotor = new CANSparkMax(PortConstants.kGripperMotorPort, MotorType.kBrushless);
    gripperMotor.setIdleMode(IdleMode.kBrake);
    gripperMotor.setInverted(false);
    beamBreakSensor = new DigitalInput(2);
    
        
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Beam Break Sensor Value", getBeamBreakSensor());
    SmartDashboard.putNumber("Gripper Motor Curent", gripperMotor.getOutputCurrent());
    SmartDashboard.putNumber("gripper motor output", gripperMotor.get());

  }

  public void runGripper(double percentOutput) {

    gripperMotor.set(percentOutput);
   
  }


  public void reverseGripper(double percentOutput) {

    gripperMotor.set(-percentOutput);
  }

 

  public void stopGripper() {
    gripperMotor.stopMotor();
   
  }

  public double getGripperCurrent() {
    return gripperMotor.getOutputCurrent();
  }

  public boolean getBeamBreakSensor() {
    return !beamBreakSensor.get();
  }

  public void gripperHoldCone(){
    reverseGripper(.075);;
  }

  public void gripperHoldCube(){
    runGripper(.075);
  }
}
