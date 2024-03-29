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
  DigitalInput coneBeamBreak;
  DoubleSolenoid gripperSolenoid;
  LEDModeSubsystem ledModeSubsystem;


  /** Creates a new Gripper. */
  public Gripper() {


    gripperMotor = new CANSparkMax(PortConstants.kGripperMotorPort, MotorType.kBrushless);// Setting the motor equal to a new sparkmax
    gripperMotor.setIdleMode(IdleMode.kBrake);// Sets the motor to either brake or coast mode
    gripperMotor.setInverted(false);
    beamBreakSensor = new DigitalInput(2);// Sets the beam break sensor to digital input port 2
    coneBeamBreak = new DigitalInput(3);
    //this.ledModeSubsystem = ledModeSubsystem;
    
        
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Beam Break Sensor Value", getBeamBreakSensor());
    SmartDashboard.putBoolean("Cone Beam Break", getConeBeamBreak());

    SmartDashboard.putNumber("Gripper Motor Curent", gripperMotor.getOutputCurrent());
    SmartDashboard.putNumber("gripper motor output", gripperMotor.get());

  }

  public void runGripper(double percentOutput) {// Sets the motor to an output percentage 

    gripperMotor.set(percentOutput);
   
  }


  public void reverseGripper(double percentOutput) {// Sets the motor to an output percentage reversed

    gripperMotor.set(-percentOutput);
  }


  public void stopGripper() {
    gripperMotor.stopMotor();
   
  }

  public double getGripperCurrent() {// Returns the current being sent to the motors
    return gripperMotor.getOutputCurrent();
  }

  public boolean getBeamBreakSensor() {// Returns the status of the beam break sensor
    return !beamBreakSensor.get();
  }

  public boolean getConeBeamBreak() {
    return !coneBeamBreak.get();
  }

  public void gripperHoldCone(){
    reverseGripper(.075);;
  }

  public void gripperHoldCube(){
    runGripper(.075);
  }
}
