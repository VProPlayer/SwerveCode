package frc.robot.commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;



public class SwerveJoystick extends Command{
    private final DoubleSupplier forwardX, forwardY, rotation, slider;
    private final SwerveSubsystem swerveSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;
    public SwerveJoystick(SwerveSubsystem swerveSubsystem, DoubleSupplier forwardX, DoubleSupplier forwardY, DoubleSupplier rotation, DoubleSupplier slider){
        this.swerveSubsystem = swerveSubsystem;
        this.forwardX = forwardX;
        this.forwardY = forwardY;
        this.rotation = rotation;
        this.slider = slider;

        this.xLimiter = new SlewRateLimiter(DriveConstants.maxAcceleration);
        this.yLimiter = new SlewRateLimiter(DriveConstants.maxAcceleration);
        this.rotLimiter = new SlewRateLimiter(DriveConstants.maxAngularAcceleration);
        addRequirements(swerveSubsystem);
    }


     // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -forwardX.getAsDouble();
    double ySpeed = -forwardY.getAsDouble();
    double rot = -rotation.getAsDouble();

    xSpeed = Math.abs(xSpeed) > 0.25 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.35 ? ySpeed : 0.0;
    rot = Math.abs(rot) > 0.4 ? rot : 0.0;

    xSpeed = xLimiter.calculate(xSpeed)* DriveConstants.kTeleDriveMaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed)* DriveConstants.kTeleDriveMaxSpeed;
    rot = rotLimiter.calculate(rot)* DriveConstants.kTeleDriveMaxAngularSpeed;

    ChassisSpeeds chassisSpeeds;
    swerveSubsystem.drive(
      xSpeed,
      ySpeed, 
      rot, 
      swerveSubsystem.fieldRelativeStatus
    );


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
