// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
 

public class SwerveModule extends SubsystemBase {
  private final SparkMax driveMotor;
  private final SparkMax rotationMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANcoder canCoder;
  private final double canCoderOffsetRadians;

  private final PIDController rotationPIDController;

  /** Creates a new SwerveModule. */
  public SwerveModule(
    int driveID,
    int rotationID,
    int canCoderID,
    double canCoderOffsetRadians,
    boolean isDriveInverted
    /*boolean isRotationInverted */) {
    driveMotor = new SparkMax(driveID, MotorType.kBrushless);
    rotationMotor = new SparkMax(rotationID, MotorType.kBrushless);

    // DRIVE motor configuration
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    
    driveConfig
    .inverted(isDriveInverted)
    .idleMode(IdleMode.kBrake);
    driveConfig.encoder
    .positionConversionFactor(DriveConstants.driveEncoderPositionConversionFactor)
    .velocityConversionFactor(DriveConstants.driveEncoderVelocityConversionFactor);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ROTATION motor configuration
    SparkMaxConfig rotationConfig = new SparkMaxConfig();
    
    rotationConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    rotationConfig.encoder
    .positionConversionFactor(DriveConstants.rotationEncoderPositionConversionFactor)
    .velocityConversionFactor(DriveConstants.rotationEncoderVelocityConversionFactor);

    rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // PID stuff
    rotationPIDController = new PIDController(0.25, 0, 0);
    rotationPIDController.setTolerance(0.01);
    rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Initalizations
    canCoder = new CANcoder(canCoderID);
    this.canCoderOffsetRadians = canCoderOffsetRadians;
    driveEncoder = driveMotor.getEncoder();
    rotationEncoder = rotationMotor.getEncoder();



  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getRotationPosition() {
    return rotationEncoder.getPosition();
  }

  public double getRotationVelocity() {
    return rotationEncoder.getVelocity();
  }

  public SparkMax getDriveMotor() {
    return driveMotor;
  }

  public SparkMax getRotationMotor() {
    return rotationMotor;
  }

  public CANcoder getCANcoder() {
    return canCoder;
  }

  public double getCANCoderRad() {
    double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
    double angle = (2 * Math.PI * absolutePosition) - canCoderOffsetRadians;

    return angle % (2 * Math.PI);
  }

  public void initRotationOffset() {
    rotationEncoder.setPosition(getCANCoderRad());
}

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(getCANCoderRad());
  }

  // Maybe Change getRotationPosition to getCANCoderRad
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getCANCoderRad()));
  }

  public void setDesiredState(SwerveModuleState state) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    if((Math.abs(state.speedMetersPerSecond)< 0.001)){
      stop();
      return;
    }

    // state = optimizeModule(state, new Rotation2d(getCANCoderRad()));
    state = optimize(state, getState().angle);
    



    // Set the drive motor speed
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.maxSpeed);

    // Calculate the desired rotation position
    double desiredRotation = state.angle.getRadians();

    // Debugging: Print the desired rotation angle
    System.out.println("Desired Rotation (Radians): " + desiredRotation);

    // Set the rotation motor position using the PID controller
    double rotationOutput = rotationPIDController.calculate(getCANCoderRad(), desiredRotation);
    rotationMotor.set(rotationOutput);

    driveMotor.setVoltage(DriveConstants.driveFF.calculate(state.speedMetersPerSecond));
  }

  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90 ){
      return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }
  // public static SwerveModuleState optimizeModule(SwerveModuleState state, Rotation2d angle){
  //   double targetAngle = placeInRange(angle.getRadians(), state.angle.getRadians());
  //   double targetSpeed = state.speedMetersPerSecond;
  //   double currentAngle = angle.getRadians();
  //   double delta = targetAngle - currentAngle;

  //   if(Math.abs(delta) > Math.PI / 2){
  //     targetAngle = delta > Math.PI ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
  //     targetSpeed *= -1;
  //   }
  //   return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
  // }

  // public static double placeInRange(double current, double expected){
  //   double low = 0;
  //   double high = 0;
  //   final double twopi = 2 * Math.PI;
  //   double offset = current % twopi;

  //   if (offset >= 0){
  //     low = current - offset;
  //     high = current + (twopi - offset);

  //   } else{
  //     high = current - offset;
  //     low = current + (twopi - offset);
  //   }
  //   while(expected < low){
  //     expected += twopi;
  //   }
  //   while(expected > high){
  //     expected -= twopi;
  //   }
  //   if(expected - current > Math.PI){
  //     expected -= twopi;
  //   } else if (expected - current < -Math.PI){
  //     expected += twopi;
  //   }

  //   return expected;
  // }

  public void stop(){
    driveMotor.set(0);
    rotationMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
