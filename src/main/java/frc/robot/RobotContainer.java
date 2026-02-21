// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.configuration.Constants;
import static frc.robot.configuration.Constants.OperatorConstants.SubSystemIDEnum.*;
//import frc.robot.commands.ControllerCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.components.motor.MotorIO;
import frc.robot.components.motor.MotorIOKraken;
import frc.robot.configuration.Constants.OperatorConstants;
import frc.robot.configuration.configs.ShooterSubsysConfig;
import frc.robot.configuration.configs.SubsystemConfig;
import frc.robot.subsystems.climb.ExampleSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.pose.PoseEstimatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import swervelib.SwerveInputStream;
import frc.robot.components.control.PID;
import frc.robot.components.controller.ControllerIfc;
import frc.robot.components.controller.JoystickControllerIfc;
import frc.robot.components.controller.XboxControllerIfc;
// import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // subsystems:
  // private final ShooterSubsysConfig ShooterSSConfig = new
  // ShooterSubsysConfig(true, SHOOTER_SUBSYSTEM);
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ShooterSubsystem m_ShooterSubsystem = new
  // ShooterSubsystem(ShooterSSConfig);
  // private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(m_visionSubsystem, m_swerveSubsystem,new Pose2d()); // Instantiate pose estimator (reliant on vision and swerve subsystems though)

  // create 2 instances of our new controller interface:
  private final ControllerIfc m_driverController = new XboxControllerIfc(OperatorConstants.controllerPort1);
  // private final ControllerIfc m_operatorController = new
  // XboxControllerIfc(OperatorConstants.controllerPort2);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getY() * -1,
      () -> m_driverController.getX() * -1)
      .withControllerRotationAxis(m_driverController::getTwist)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getX,
      m_driverController::getY)
      .headingWhile(true);

  Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // new Trigger(m_Shooter::commandShooter)
    // .onTrue(new ControllerCommand(m_driverController));

    // new Trigger(m_driverController::commandShooter)
    // .onTrue(new ControllerCommand(m_driverController));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    // TODO: Temporary start motor (Move to operator controller?)
    // m_driverController.runIntake().onTrue(
    // new InstantCommand( () ->
    // m_motor.setVoltage(6.0))
    // );

    // Temporary stop motor
    // m_driverController.stopIntake().onTrue(
    // new InstantCommand( () ->
    // m_motor.stop())
    // );

    // TODO: Should this be on the operator controller?
    /*
     * m_driverController.runShooter().whileTrue(
     * new RunCommand( () -> {
     * double trigger = m_driverController.controlMotorSpeed();
     * double targetRPM = (trigger * MAX_RPM);
     * double targetRadsPerSec =
     * Units.rotationsPerMinuteToRadiansPerSecond(targetRPM);
     * double volts = feedforward.calculate(targetRadsPerSec);
     * m_motor.setVoltage(volts);
     * }).withTimeout(3)
     * );
     */

    // TODO: Replace onchange when class is futher developed (and move to operator
    // controller?)
    /*
     * m_operatorController.startShooter().onTrue(
     * new InstantCommand( () ->
     * m_ShooterSubsystem.toggleShoot())
     * 
     * );
     * // m_driverController.runIntake().onTrue(
     * // new InstantCommand( () ->
     * // m_IntakeSubsystem.toggleIntake())
     * 
     * // );
     * 
     * // NEO (names of controller commands unimportant for this case / temporary)
     * m_driverController.runIntake().whileTrue(
     * m_ShooterSubsystem.sysIdNeoQuasistatic(SysIdRoutine.Direction.kForward)
     * );
     * 
     * m_driverController.runShooter().whileTrue(
     * m_ShooterSubsystem.sysIdNeoQuasistatic(SysIdRoutine.Direction.kReverse)
     * );
     * 
     * m_driverController.stopIntake().whileTrue(
     * m_ShooterSubsystem.sysIdNeoDynamic(SysIdRoutine.Direction.kForward)
     * );
     * 
     * m_driverController.runClimb().whileTrue(
     * m_ShooterSubsystem.sysIdNeoDynamic(SysIdRoutine.Direction.kReverse)
     * );
     * 
     * 
     * }
     * 
     * 
     * 
     * );
     */

    // TODO: Is this supposed to be on the operator controller?
    // m_driverController.runIntake().onTrue(
    // new InstantCommand( () ->
    // m_IntakeSubsystem.toggleIntake())
    // );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
