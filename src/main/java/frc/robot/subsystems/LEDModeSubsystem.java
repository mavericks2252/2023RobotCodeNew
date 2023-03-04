// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PortConstants;

public class LEDModeSubsystem extends SubsystemBase {
  boolean cubeMode = false;
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  AddressableLEDBuffer ledBufferOff;
  Gripper gripper;

  AddressableLEDBuffer yellowLedBuffer;
  AddressableLEDBuffer purpleLedBuffer;
  Timer ledTimerOff;
  Timer ledTimerOn;
  boolean blink = false;
  /** Creates a new LEDModeSubsystem. */
  public LEDModeSubsystem(Gripper gripper) {
    this.gripper = gripper;
    
    ledTimerOff = new Timer();
    ledTimerOn = new Timer();
    
    led = new AddressableLED(PortConstants.kLEDStripPort);
    ledBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    ledBufferOff = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    purpleLedBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    yellowLedBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    led.setLength(ledBuffer.getLength());
    led.start();
    for (var i = 0; i < purpleLedBuffer.getLength(); i++){
      purpleLedBuffer.setRGB(i, 255, 0, 255);
    }
    for (var i = 0; i < yellowLedBuffer.getLength(); i++){
      yellowLedBuffer.setRGB(i, 255, 255, 0);
    }
    for (var i = 0; i < ledBufferOff.getLength(); i++){
      ledBufferOff.setRGB(i, 0, 0, 0);// off
    }
    for (var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 255, 0, 0);
    }
    led.setData(yellowLedBuffer);
    ledTimerOn.start();
    ledTimerOff.reset();
    ledTimerOn.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Robot Mode", cubeMode);
    SmartDashboard.putNumber("Led On Timer", ledTimerOn.get());
    SmartDashboard.putNumber("Led Off Timer", ledTimerOff.get());
    
      /*if (!gripper.getBeamBreakSensor()){ //if beam break broken set to blink mode
        blink = true;
      }
      else{
        blink = false;
      }*/
      if (blink){
        if (ledTimerOn.get() >= .25){
          ledTimerOn.reset();
          ledTimerOn.stop();      
          ledTimerOff.start();
          setLEDColor();        
        }

        else if (ledTimerOff.get() >= .25){
          ledTimerOff.reset();
          ledTimerOff.stop();
          ledTimerOn.start();
          ledOff();
        }
      }
      else {
        setLEDColor();
      }
                
  }


  public void cubeMode() {
    cubeMode = true;
    
    led.setData(purpleLedBuffer);
  }

  public void coneMode() {
    cubeMode = false;
    
    led.setData(yellowLedBuffer);
  }

  public boolean getRobotMode() {
    return cubeMode;
  }

  public void setLEDColor() {
    if(cubeMode) {
      led.setData(purpleLedBuffer);
    }
    else{
      led.setData(yellowLedBuffer);
    }
  }

  public void ledOff(){
    led.setData(ledBufferOff);
  }
}
