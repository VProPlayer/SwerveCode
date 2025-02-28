// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int JoystickResetHeading = 5;
  }

  public static class DriveConstants {

    // Given Motor Rotations, convert to Meters traveled
    // (1 rev / Gear Ratio) * ((2 * PI * r) / (1 Rev)) = 
    // (2 * PI * r) / (Gear Ratio) = 
    public static final double driveEncoderPositionConversionFactor = 0.0540992905;

    
    // dx/dt
    // Given RPM, convert to m/s
    public static final double driveEncoderVelocityConversionFactor = driveEncoderPositionConversionFactor / 60.0;

    // Given Motor Rotations, convert to Radians travelled
    // (1 rev / Gear Ratio) * ((2 * PI) RAD / (1 Rev))
    // (2 * PI) RAD / (Gear Ratio)
    public static final double rotationEncoderPositionConversionFactor = 0.335103217;

    // Given RPM, convert to radians/seconds
    public static final double rotationEncoderVelocityConversionFactor = rotationEncoderPositionConversionFactor / 60.0;

    // Global
    public static final double maxSpeed = 5; // meters/sec
    public static final double maxAcceleration = 10; // meters/sec^2
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/sec
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/sec^2
    // Teleop max speeds
    public static final double kTeleDriveMaxSpeed = 7.5 / 4.0;
    public static final double kTeleDriveMaxAngularSpeed = 3;
    
    // Drivetrain Feedforward Constant (not final)
    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.2, 2.5, 0.0);

    // Drivetrain

     // front left 
     public static final int frontLeftDriveMotorId = 1;
     public static final int frontLeftRotationMotorId = 2;
     public static final int frontLeftCanCoderId = 11;
     public static final double frontLeftOffsetRad = 0.867188 * 2 * Math.PI;
     // front right 
     public static final int frontRightDriveMotorId = 8;
     public static final int frontRightRotationMotorId = 7;
     public static final int frontRightCanCoderId = 12;
     public static final double frontRightOffsetRad = 0.038330 * 2 * Math.PI;
     // back left
     public static final int backLeftDriveMotorId = 5;
     public static final int backLeftRotationMotorId = 6;
     public static final int backLeftCanCoderId = 14;
     public static final double backLeftOffsetRad = 0.245361 * 2 * Math.PI;
     // back right
     public static final int backRightDriveMotorId = 4;
     public static final int backRightRotationMotorId = 3;
     public static final int backRightCanCoderId = 13;
     public static final double backRightOffsetRad = 0.473633 * 2 * Math.PI;
     
     public static final double wheelBase = Units.inchesToMeters(25.125); // distance between front wheels (like train track)
    public static final double trackWidth = Units.inchesToMeters(21.25); // distance from center of wheels on side

     public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // front left (-,+)
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front right (+,+)
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // back left (-,-)
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0) // back right (+,-)
    );
  }
}
