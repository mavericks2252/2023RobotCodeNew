// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private double tv;
  private double tx;
  private double ty;
  private double ta;
  private double ry;

  /** Creates a new Vision. */
  public Vision() {
SmartDashboard.putBoolean("target check", false);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    tv = limeLightTargetCheck();
    tx = limeLightHorizontalPosition();
    ty = limeLightVerticalPosition();
    ta = limeLightTargetArea();
    ry = limeLightTurnAngle();

    SmartDashboard.putNumber("Vision Target Found", tv);
    SmartDashboard.putNumber("Horizontal Position", tx);
    SmartDashboard.putNumber("Vertical Postion", ty);
    SmartDashboard.putNumber("Target Distance", ta);
    SmartDashboard.putNumber("Turning Position", ry);


  }

  public double limeLightTargetCheck() {
            SmartDashboard.putBoolean("target check", true);
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(5);
  }

  public double limeLightHorizontalPosition() {

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double limeLightVerticalPosition() {

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double limeLightTargetArea() {

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  public double limeLightTurnAngle() {

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ry").getDouble(0);
  }
}
