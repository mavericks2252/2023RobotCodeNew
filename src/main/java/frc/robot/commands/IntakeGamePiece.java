// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.TopArm;

public class IntakeGamePiece extends CommandBase {
  Intake intake;
  Gripper gripper;
  Floor floor;
  TopArm topArm;
  BottomArm bottomArm;
  LEDModeSubsystem ledModeSubsystem;
  double topArmError;
  double bottomArmError;
  Timer endTimer;
  /** Creates a new CubeIntake. */
  public IntakeGamePiece(Intake intake, Gripper gripper, Floor floor, TopArm topArm, BottomArm bottomArm, LEDModeSubsystem ledModeSubsystem) {
    this.intake = intake;
    this.gripper = gripper;
    this.floor = floor;
    this.topArm = topArm;
    this.bottomArm = bottomArm;
    this.ledModeSubsystem = ledModeSubsystem;
    endTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, gripper, topArm, bottomArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTimer.reset();
    endTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    
    floor.runFloorMotor();
    
    // Cube mode
    if (ledModeSubsystem.getRobotMode()){
      gripper.runGripper(GripperConstants.gripperMotorSpeed);
      topArm.setMotorPosition(95);//95
      bottomArm.setMotorPosition(65);
      intake.runIntake(IntakeConstants.kIntakeMotorSpeed);
    }

  
    // Cone mode
    else {
      intake.runIntake(.6);
      topArm.setMotorPosition(115);//119
      bottomArm.setMotorPosition(78);
      gripper.reverseGripper(.5);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //send arm to stow position
    topArm.setMotorPosition(ArmConstants.kTopStowPosition);
    bottomArm.setMotorPosition(ArmConstants.kBottomStowPosition);
    intake.stopIntake();
        
    if (ledModeSubsystem.getRobotMode()){
      gripper.gripperHoldCube();
    }
    else{
      gripper.gripperHoldCone();
    }
   

    // Starting a new thread and waiting a period of time to retract the intake
    new Thread(() -> {
      try{
        Thread.sleep(500);
        intake.retractIntake();
        floor.stopFloorMotor();
        
    } catch(Exception e){}
    }).start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    
    
   //Cube Mode
   if(ledModeSubsystem.getRobotMode()){

    // if beamBreak sensor broken end
    if(gripper.getBeamBreakSensor() && gripper.getGripperCurrent() > 12 ){
          ledModeSubsystem.startBlinking();
          return true;

      }    

      //not broken keep running
      else {
        
        return false;
      }

    }
    
    // cone mode
    else{
      
      // Only look at current
      if(gripper.getGripperCurrent() > 15){
        //wait .5 seconds after current threshold to end    
        if(endTimer.get() > .5){
            ledModeSubsystem.startBlinking();
            return true;
            
        }
        //timer not expired return false
        else{
          return false;
        }
          }
      //keep reseting timer if current limit isnt hit
      else {
        endTimer.reset();
        return false;
        }
   }
  }
    
   
  
}
