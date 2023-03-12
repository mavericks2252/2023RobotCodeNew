// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PortConstants;

public class LEDModeSubsystem extends SubsystemBase {
  boolean cubeMode = false;
  AddressableLED led;
  AddressableLEDBuffer ledBufferOff;
  Gripper gripper;
  AddressableLEDBuffer yellowLedBuffer;
  AddressableLEDBuffer purpleLedBuffer;
  AddressableLEDBuffer redLEDBuffer;
  Timer ledTimerOff;
  Timer ledTimerOn;
  boolean blink = false; // sets robot to cone mode by default
  
  /** Creates a new LEDModeSubsystem. */
  public LEDModeSubsystem(Gripper gripper) {
    this.gripper = gripper;
    
    //create led blink timers
    ledTimerOff = new Timer();
    ledTimerOn = new Timer();
    
    // initialize LED's LED's
    led = new AddressableLED(PortConstants.kLEDStripPort);
    ledBufferOff = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    purpleLedBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    yellowLedBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    redLEDBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    led.setLength(yellowLedBuffer.getLength());
    led.start();
    
    // create buffers for each color
    for (var i = 0; i < purpleLedBuffer.getLength(); i++){
      purpleLedBuffer.setRGB(i, 255, 0, 255);
    }
    for (var i = 0; i < yellowLedBuffer.getLength(); i++){
      //yellowLedBuffer.setRGB(i, 255, 255, 0);
      yellowLedBuffer.setLED(i, Color.kYellow);
    }
    for (var i = 0; i < ledBufferOff.getLength(); i++){
      ledBufferOff.setRGB(i, 0, 0, 0);// off
    }
    for (var i = 0; i < redLEDBuffer.getLength(); i++){
      redLEDBuffer.setRGB(i, 255, 0, 0);// red
    }

    
    // defualt robot to Cone Mode
    led.setData(yellowLedBuffer);
    
    //led blinking timers
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
    
     
      //blink mode - Blink LED's for given time
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
      
      // No game piece set led color based on mode
      else { 
        setLEDColor();
      }
                
  }


  //set robot to cubme mode
  public void cubeMode() {
    cubeMode = true;
    
    led.setData(purpleLedBuffer);
  }

  // set robot to cone mode
  public void coneMode() {
    cubeMode = false;
    
    led.setData(yellowLedBuffer);
  }

  // Get robot Mode true = cube false = cone
  public boolean getRobotMode() {
    return cubeMode;
  }

  // Set LED Color based on mode
  public void setLEDColor() { 
    
    if (DriverStation.isDisabled()){
      led.setData(redLEDBuffer);
    }
    
    else if(cubeMode) {
      led.setData(purpleLedBuffer);
    }
    
    else{
      led.setData(yellowLedBuffer);
    }
    
    
  }

  // turn LED's off
  public void ledOff(){
    led.setData(ledBufferOff);
  }

  public void setRedLEDs () {
    led.setData(redLEDBuffer);
  }

  public void startBlinking() {// To be called elsewhere to blink the lights
    blink = true;
  }

  public void stopBlinking() {// To be called elsewhere to return the lights to a solid color
    blink = false;
  }
}
