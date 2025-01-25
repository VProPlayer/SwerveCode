// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }
}
