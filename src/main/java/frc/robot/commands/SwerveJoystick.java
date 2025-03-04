package frc.robot.commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;



public class SwerveJoystick extends Command{
    private final DoubleSupplier forwardX, forwardY, rotation, slider;
    private final SwerveSubsystem swerveSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

    private static final double X_DEADBAND = 0.25;
    private static final double Y_DEADBAND = 0.35;
    private static final double ROT_DEADBAND = 0.4;

    
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
    double xSpeed = -forwardX.getAsDouble(); // un negate?
    double ySpeed = -forwardY.getAsDouble();
    double rot = -rotation.getAsDouble();

    SmartDashboard.putNumber("Processed Rotation Value", rot);


    xSpeed = applyDeadbandAndLimiter(xSpeed, X_DEADBAND, xLimiter, DriveConstants.maxSpeed);
    ySpeed = applyDeadbandAndLimiter(ySpeed, Y_DEADBAND, yLimiter, DriveConstants.maxSpeed);
    rot = applyDeadbandAndLimiter(rot, ROT_DEADBAND, rotLimiter, DriveConstants.maxAngularVelocity);

    double sliderVal = (-slider.getAsDouble() + 1) / 2;
    sliderVal = Math.max(sliderVal, 0.15);
    xSpeed *= sliderVal;
    ySpeed *= sliderVal;
    rot *= sliderVal;

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.1, 1);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.1, 1);
    rot = MathUtil.applyDeadband(rot, 0.3, 1);

    swerveSubsystem.drive(
      xSpeed,
      ySpeed, 
      rot, 
      swerveSubsystem.fieldRelativeStatus
    );


  }

  private double applyDeadbandAndLimiter(
    double value, 
    double deadband, 
    SlewRateLimiter limiter, 
    double maxSpeed) {

    value = Math.abs(value) > deadband ? value : 0.0;
    return limiter.calculate(value) * maxSpeed;
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
