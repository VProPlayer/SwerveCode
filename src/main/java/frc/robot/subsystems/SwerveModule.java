// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
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
  private final SparkFlex driveMotor;
  private final SparkFlex rotationMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANcoder canCoder;
  private final double canCoderOffsetRadians;

  private final PIDController rotationPidController;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveID, int rotationID, int canCoderID, double canCoderOffsetRadians, boolean isDriveInverted) {
    driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
    rotationMotor = new SparkFlex(rotationID, MotorType.kBrushless);

    // DRIVE motor configuration
    SparkFlexConfig driveConfig = new SparkFlexConfig();
    
    driveConfig
    .inverted(isDriveInverted)
    .idleMode(IdleMode.kBrake);
    driveConfig.encoder
    .positionConversionFactor(DriveConstants.driveEncoderPositionConversionFactor)
    .velocityConversionFactor(DriveConstants.driveEncoderVelocityConversionFactor);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ROTATION motor configuration
    SparkFlexConfig rotationConfig = new SparkFlexConfig();
    
    rotationConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    rotationConfig.encoder
    .positionConversionFactor(DriveConstants.rotationEncoderPositionConversionFactor)
    .velocityConversionFactor(DriveConstants.rotationEncoderVelocityConversionFactor);

    driveMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    PIDController rotationPIDController = new PIDController(0, 0, 0);

    rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

    canCoder = new CANcoder(canCoderID);


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

  public SparkFlex getDriveMotor() {
    return driveMotor;
  }

  public SparkFlex getRotationMotor() {
    return rotationMotor;
  }

  public CANcoder getCANcoder() {
    return canCoder;
  }

  public double getCANCoderRad() {
    double angle = (Math.PI * 2 * canCoder.getAbsolutePosition().getValueAsDouble()) - canCoderOffsetRadians % (2* Math.PI);
    return angle;
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(getCANCoderRad());
  }

  // Maybe Change getRotationPosition to getCANCoderRad
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotationPosition()));
  }

  public void setDesiredStates

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
