// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Meter;
import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class SwerveSubsystem extends SubsystemBase {
  private enum Orientation {
    FIELD(0), 
    ROBOT(1);
  
    private int value;

    private Orientation (int value){
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


  private Orientation driveOrientation = Orientation.FIELD;

  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  private SwerveDrive  swerveDrive;

  public SwerveSubsystem(SwerveSubsysConfig config) {
    this.swerveConfig = config;
    if (swerveConfig.getIsPresent()) {
      try
      {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.SwerveConstants.maxSpeed, new Pose2d(new Translation2d(Meter.of(1), 
                                                                                                                                    Meter.of(4)), 
                                                                                                                                  Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
      } catch (Exception e)
      {
        throw new RuntimeException(e);
      }
      driveAngularVelocity = SwerveInputStream.of(swerveDrive,
                                                  () -> swerveConfig.getController().getY() * -1,
                                                  () -> swerveConfig.getController().getX() * -1)
                                                .withControllerRotationAxis(swerveConfig.getController()::getTwist)
                                                .deadband(swerveConfig.getDeadband())
                                                .scaleTranslation(0.8)
                                                .allianceRelativeControl(() -> isFieldOriented())
                                                .robotRelative(() -> isRobotOriented());
      /**
      * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
      */
      driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(swerveConfig.getController()::getX,
                                                                               swerveConfig.getController()::getY)
                                                    .headingWhile(true);

      driveFieldOrientedDirectAngle      = driveFieldOriented(driveDirectAngle);
      driveFieldOrientedAnglularVelocity = driveFieldOriented(driveAngularVelocity);
      setDefaultCommand(driveFieldOrientedAnglularVelocity);
      //publish field oreiantation to smart dashboard
    }
    SmartDashboard.putString("Swerve Orientation", driveOrientation.textValue());
    SmartDashboard.putBoolean("Swerve Present", swerveConfig.getIsPresent());
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
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Swerve Orientation", driveOrientation.textValue());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putString("Swerve Orientation", driveOrientation.textValue());
    // This method will be called once per scheduler run during simulation
  }


  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public boolean isFieldOriented (){
    if (driveOrientation == Orientation.FIELD){
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isRobotOriented (){
    if (driveOrientation == Orientation.ROBOT){
      return true;
    }
    else {
      return false;
    }
  }

  public void orientationToggle (){
    if (driveOrientation == Orientation.FIELD){
      driveOrientation = Orientation.ROBOT;
    }
    else {
      driveOrientation = Orientation.FIELD;
    }
  }
}
