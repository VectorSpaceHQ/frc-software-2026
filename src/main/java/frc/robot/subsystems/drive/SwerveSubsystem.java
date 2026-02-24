// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.Constants;
import frc.robot.configuration.configs.SwerveSubsysConfig;

import static edu.wpi.first.units.Units.Meter;
import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import swervelib.SwerveInputStream;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private SwerveDrive swerveDrive;

  private enum Orientation {
    FIELD(0),
    ROBOT(1);

    private int value;

    private Orientation(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  private SwerveSubsysConfig swerveConfig = null;
  private SwerveInputStream driveDirectAngle = null;
  private Command driveFieldOrientedDirectAngle = null;
  private Command driveFieldOrientedAnglularVelocity = null;
  private SwerveInputStream driveAngularVelocity = null;

  private Orientation driveOrientation = Orientation.FIELD;
  private double speedLimiter = 0.5;

  public SwerveSubsystem(SwerveSubsysConfig config) {

    this.swerveConfig = config;
    if (swerveConfig.getIsPresent()) {
      try {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.SwerveConstants.maxSpeed * speedLimiter,
            new Pose2d());
        zeroGyro();
        resetOdometry();
        // Alternative method if you don't want to supply the conversion factor via JSON
        // files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
        // angleConversionFactor, driveConversionFactor);
      } catch (Exception e) {
        throw new RuntimeException(e);
      }
      driveAngularVelocity = SwerveInputStream.of(swerveDrive,
          () -> swerveConfig.getController().getY() * -1,
          () -> swerveConfig.getController().getX() * -1)
          .withControllerRotationAxis(swerveConfig.getController()::getTwist)
          .deadband(swerveConfig.getDeadband())
          .scaleTranslation(1)
          .allianceRelativeControl(() -> isFieldOriented())
          .robotRelative(() -> isRobotOriented());
      /**
       * Clone's the angular velocity input stream and converts it to a fieldRelative
       * input stream.
       */
      driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(swerveConfig.getController()::getX,
          swerveConfig.getController()::getY)
          .headingWhile(true);

      driveFieldOrientedDirectAngle = driveFieldOriented(driveDirectAngle);
      driveFieldOrientedAnglularVelocity = driveFieldOriented(driveAngularVelocity);
      setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return swerveDrive.swerveDrivePoseEstimator;
  }

  public ChassisSpeeds getVelocity() {
    return swerveDrive.getRobotVelocity();
  }
  public void stopDrive() {
    swerveDrive.drive(new ChassisSpeeds());
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });

  }

  // These are for the pose estimator subsystem, which needs to access the
  // kinematics, module positions, and yaw of the swerve drive
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public SwerveModulePosition[] getModulePositions() {
    return swerveDrive.getModulePositions();
  }

  public Rotation2d getYaw() {
    return swerveDrive.getYaw();
  }

  public boolean isFieldOriented() {
    if (driveOrientation == Orientation.FIELD) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isRobotOriented() {
    if (driveOrientation == Orientation.ROBOT) {
      return true;
    } else {
      return false;
    }
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void resetOdometry() {
    swerveDrive.resetOdometry(new Pose2d());

  }

  public void orientationToggle() {
    if (driveOrientation == Orientation.FIELD) {
      driveOrientation = Orientation.ROBOT;
    } else {
      driveOrientation = Orientation.FIELD;
    }
  }
}
