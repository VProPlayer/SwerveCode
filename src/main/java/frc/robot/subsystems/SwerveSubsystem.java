package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class SwerveSubsystem extends SubsystemBase{
    public static boolean fieldRelativeStatus = true;
    private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.frontLeftDriveMotorId, 
    DriveConstants.frontLeftRotationMotorId, 
    DriveConstants.frontLeftCanCoderId, 
    DriveConstants.frontLeftOffsetRad,
    false);
  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.frontRightDriveMotorId, 
    DriveConstants.frontRightRotationMotorId, 
    DriveConstants.frontRightCanCoderId, 
    DriveConstants.frontRightOffsetRad,
    true);
  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.backLeftDriveMotorId, 
    DriveConstants.backLeftRotationMotorId, 
    DriveConstants.backLeftCanCoderId, 
    DriveConstants.backLeftOffsetRad,
    false);
  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.backRightDriveMotorId, 
    DriveConstants.backRightRotationMotorId, 
    DriveConstants.backRightCanCoderId, 
    DriveConstants.backRightOffsetRad,
    true);

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kinematics, new Rotation2d(), getModulePosition());
    
    public SwerveSubsystem(){
      new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyro.reset();
        odometry.resetPosition(new Rotation2d(), getModulePosition(), new Pose2d());
      } catch (Exception e) {
      }
    }).start();

    // initialize CANcoder offsets
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    backLeft.initRotationOffset();
    backRight.initRotationOffset();

    // reset encoders upon each start
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
    }

     public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds chassisSpeeds = DriveConstants.kinematics.toChassisSpeeds(getStates());
        return chassisSpeeds;
    }

    public void stopDrive(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    public AHRS getNavX() {
        return gyro;
      }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeft.getState();
        states[1] = frontRight.getState();
        states[2] = backLeft.getState();
        states[3] = backRight.getState();

        return states;
    }
    

    public void setDrive(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getModulePosition() {
    SwerveModulePosition[] positions = {
      new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getCANCoderRad())),
      new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getCANCoderRad())),
      new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getCANCoderRad())),
      new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getCANCoderRad()))
    };

    

    return positions;
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // makes it never go above specified max velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
    // Sets the speed and rotation of each module
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getYaw());
  }
  

  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */

    ChassisSpeeds speeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed);

    setModuleStates(states);

  }

  @Override
  public void periodic(){
    double[] swerve = new double[]{
      frontLeft.getDrivePosition(),
      frontRight.getDrivePosition(),
      backLeft.getDrivePosition(),
      backRight.getDrivePosition(), 
      frontLeft.getRotationPosition(),
      frontRight.getRotationPosition(),
      backLeft.getRotationPosition(),
      backRight.getRotationPosition()
    };
    double[] swerveStates = new double[]{
      frontLeft.getState().speedMetersPerSecond,
      frontRight.getState().speedMetersPerSecond,
      backLeft.getState().speedMetersPerSecond,
      backRight.getState().speedMetersPerSecond,
      frontLeft.getState().angle.getDegrees(),
      frontRight.getState().angle.getDegrees(),
      backLeft.getState().angle.getDegrees(),
      backRight.getState().angle.getDegrees()
    };

    SmartDashboard.putNumberArray("Swerve", swerve);
    SmartDashboard.putNumberArray("Swerve States", swerveStates);

  }
  
}
