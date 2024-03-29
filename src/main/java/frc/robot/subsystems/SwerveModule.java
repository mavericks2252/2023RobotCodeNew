package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.MotorConstants;



public class SwerveModule {

    WPI_TalonFX drivingMotor;
    WPI_TalonFX turningMotor;

    PIDController turningPIDController;

    WPI_CANCoder absoluteEncoder;
    Boolean absoluteEncoderReversed;
    Double absoluteEncoderOffsetRad;

    StatorCurrentLimitConfiguration drivStatorCurrentLimitConfiguration = new StatorCurrentLimitConfiguration(true, 60, 40, .5);
    SupplyCurrentLimitConfiguration driveSupplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 60, 40, .5);

    public SwerveModule(int drivingMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
         
        
        // Comp Robot
        absoluteEncoder = new WPI_CANCoder(absoluteEncoderId, "Drive System");
        drivingMotor = new WPI_TalonFX(drivingMotorId, "Drive System");
        turningMotor = new WPI_TalonFX(turningMotorId, "Drive System");

         //Practice Robot
        /*absoluteEncoder = new WPI_CANCoder(absoluteEncoderId);
        drivingMotor = new WPI_TalonFX(drivingMotorId);
        turningMotor = new WPI_TalonFX(turningMotorId);*/

        drivingMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        drivingMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        drivingMotor.configStatorCurrentLimit(drivStatorCurrentLimitConfiguration);
        drivingMotor.configSupplyCurrentLimit(driveSupplyCurrentLimitConfiguration);

        turningMotor.configStatorCurrentLimit(drivStatorCurrentLimitConfiguration);
        turningMotor.configSupplyCurrentLimit(driveSupplyCurrentLimitConfiguration);




        turningPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        drivingMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setNeutralMode(NeutralMode.Brake);

        new Thread(() -> {
            try {
              Thread.sleep(1000);
              resetEncoders();
            } catch (Exception e) {
            }
            }).start();
        

    }

    public double getDrivePosition() {
        return ticksToMeters(drivingMotor.getSelectedSensorPosition());

    }

    public double getTurningPosition() {
        
        return ticksToRads(turningMotor.getSelectedSensorPosition());

    }

    public double getDriveVelocity() {
        return drivingMotor.getSelectedSensorVelocity() * 10 / MotorConstants.kFalconTicksPerRev / ModuleConstants.kDriveMotorGearRatio * ModuleConstants.kWheelCircumferenceMeters;

    }

    public double getTurningVelocity() {
        double angle = turningMotor.getSelectedSensorVelocity() * 10  / MotorConstants.kFalconTicksPerRev / ModuleConstants.kTurningMotorGearRatio * 360;
        return Units.degreesToRadians(angle);

    }
    
    public double getAbsoluteEncoderRad() {
        double angle = Units.degreesToRadians(absoluteEncoder.getAbsolutePosition());
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double radsToTicks(double rads){
        double degrees = Units.radiansToDegrees(rads);
        return degrees * MotorConstants.kFalconTicksPerRev * ModuleConstants.kTurningMotorGearRatio / 360;

        
        
       // return rads * MotorConstants.kFalconTicksPerRev * ModuleConstants.kTurningMotorGearRatio / Math.PI * 2;
    }

    public double ticksToRads(double ticks){
        return ticks / MotorConstants.kFalconTicksPerRev / ModuleConstants.kTurningMotorGearRatio * Math.PI * 2;
    }

    public double ticksToMeters(double ticks){
        return ticks / MotorConstants.kFalconTicksPerRev / ModuleConstants.kDriveMotorGearRatio * ModuleConstants.kWheelCircumferenceMeters;
    }

    public double metersToTicks(double meters){
        return meters * MotorConstants.kFalconTicksPerRev * ModuleConstants.kDriveMotorGearRatio / ModuleConstants.kWheelCircumferenceMeters;
    }

    public void resetEncoders() {

        drivingMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition( radsToTicks( getAbsoluteEncoderRad() ) );
        SmartDashboard.putBoolean("Reset Encoders", true);
       
    }

    public SwerveModuleState getState(){
       return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        drivingMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state" , state.toString());
    }

    public void setXstanceAngle(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getState().angle);
        drivingMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state" , state.toString());
    }

    public void stop() {
        drivingMotor.set(0);
        turningMotor.set(0);
    }

    public SwerveModulePosition getModulePosition( ){

        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }
    

}
