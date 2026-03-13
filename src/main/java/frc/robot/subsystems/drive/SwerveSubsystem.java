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
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import swervelib.SwerveInputStream;

//Imports for Pose Estimator
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.configuration.Constants.VisionConstants;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private SwerveDrive swerveDrive;
  private final Supplier<Optional<EstimatedRobotPose>> m_visionMeasurement;
  private Field2d m_field = new Field2d();

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

    public String textValue() {
      return this.name();
    }
  }

  private SwerveSubsysConfig swerveConfig = null;
  private SwerveInputStream driveDirectAngle = null;
  private Command driveFieldOrientedDirectAngle = null;
  private Command driveFieldOrientedAnglularVelocity = null;
  private SwerveInputStream driveAngularVelocity = null;
  Supplier<Pose2d> currentPose;

  private Orientation driveOrientation = Orientation.FIELD;


  public SwerveSubsystem(SwerveSubsysConfig config, Supplier<Optional<EstimatedRobotPose>> visionMeasurement, Pose2d initialPose) {
    this.swerveConfig = config;
    this.m_visionMeasurement = visionMeasurement;

    if (swerveConfig.getIsPresent()) {

      try {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.SwerveConstants.maxSpeed,
                                                                    new Pose2d(1, 4, new Rotation2d()));
        zeroGyro();
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

        swerveDrive.setHeadingCorrection(false);

        // Alternative method if you don't want to supply the conversion factor via JSON
        // files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
        // angleConversionFactor, driveConversionFactor);
      } catch (Exception e) {
        throw new RuntimeException(e);
      }
      driveAngularVelocity = SwerveInputStream.of(swerveDrive,
          () -> swerveConfig.getController().getY() * -0.7,
          () -> swerveConfig.getController().getX() * -0.7)
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
      // publish field orientation to smart dashboard

      SmartDashboard.putString("Swerve Orientation", driveOrientation.textValue());
      SmartDashboard.putBoolean("Swerve Present", swerveConfig.getIsPresent());

      RobotConfig PathPlannerConfig;
      // Create robot config object for Pathplanner
      try {
        PathPlannerConfig = RobotConfig.fromGUISettings();

        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getEstimatedPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotOriented(speeds), // Method that will drive the robot given ROBOT
                                                                  // RELATIVE ChassisSpeeds. Also optionally outputs
                                                                  // individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                            // holonomic drive trains
                new PIDConstants(1.3, 0.0, 0.0), // Translation PID constants
                new PIDConstants(25.0, 5.0, 0.32) // Rotation PID constants
            ),
            PathPlannerConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red
              // alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }

      // taken from poseEstimator SS
      swerveDrive.resetOdometry(initialPose);
      swerveDrive.setVisionMeasurementStdDevs(VisionConstants.VISION_ST_DEVS);

      SmartDashboard.putData("Field", m_field);
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

  // methods from Pose Estimator SS
  public void update() {
    m_visionMeasurement.get().ifPresentOrElse(measurement -> {
        SmartDashboard.putBoolean("Vision Measurement Present", true);
        swerveDrive.addVisionMeasurement(
            measurement.estimatedPose.toPose2d(),
            measurement.timestampSeconds);
            },
      () -> SmartDashboard.putBoolean("Vision Measurement Present", false));
    }

  public void resetPose(Pose2d newPose) {
    swerveDrive.resetOdometry(newPose);
  }

  public Pose2d getEstimatedPose() {
    return swerveDrive.getPose();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;

  }

  // Unused
  public void resetPose() {
    resetOdometry();
  }


  @Override
  public void periodic() {
    SmartDashboard.putString("Swerve Orientation", driveOrientation.textValue());
    // periodic from pose estimator
    update();
    Pose2d pose = getEstimatedPose();
    Logger.recordOutput("PoseEstimator/EstimatedPose", pose); // For AdvantageScope
    Logger.recordOutput("PoseEstimator/X", pose.getX());
    Logger.recordOutput("PoseEstimator/Y", pose.getY());
    Logger.recordOutput("PoseEstimator/Theta", pose.getRotation().getRadians());

    m_field.setRobotPose(pose);
    // This method will be called once per scheduler run
  }


  @Override
  public void simulationPeriodic() {
    SmartDashboard.putString("Swerve Orientation", driveOrientation.textValue());
    // This method will be called once per scheduler run during simulation
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

  public void driveRobotOriented(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public Command driveRobotOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.drive(velocity.get());
    });
  }

  // These are for the pose estimator subsystem, which needs to access the
  // kinematics, module positions, and yaw of the swerve drive

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

  public void setMaximumSpeed(double speedMultiplier) {
    swerveDrive.setMaximumAllowableSpeeds(Constants.SwerveConstants.maxSpeed * speedMultiplier, swerveDrive.getMaximumChassisAngularVelocity());
  }

  public void orientationToggle() {
    if (driveOrientation == Orientation.FIELD) {
      driveOrientation = Orientation.ROBOT;
    } else {
      driveOrientation = Orientation.FIELD;
    }
  }
}
