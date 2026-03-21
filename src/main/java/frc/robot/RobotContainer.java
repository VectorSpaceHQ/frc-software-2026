// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimTowardsHubCommand;
import frc.robot.commands.AutoShootCommand;
// import frc.robot.commands.Autos;
// import frc.robot.commands.DriveToTargetCommand;
// UNUSED: import frc.robot.configuration.Constants;
import static frc.robot.configuration.Constants.OperatorConstants.SubSystemIDEnum.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// UNUSED: import com.pathplanner.lib.commands.PathPlannerAuto;

// UNUSED: import frc.robot.commands.ControllerCommand;
// UNUSED: import frc.robot.commands.ExampleCommand;
// UNUSED: import frc.robot.components.motor.MotorIO;
// UNUSED: import frc.robot.components.motor.MotorIOKraken;
import frc.robot.configuration.Constants.OperatorConstants;
import frc.robot.configuration.Constants.ShooterConstants;
// UNUSED: import frc.robot.configuration.Constants.VisionConstants;
import frc.robot.configuration.Constants.IntakeConstants;
// UNUSED: import frc.robot.configuration.configs.SubsystemConfig;
import frc.robot.configuration.configs.SwerveSubsysConfig;
import frc.robot.configuration.configs.VisionSubsysConfig;
import frc.robot.configuration.configs.ShooterSubsysConfig;
import frc.robot.configuration.configs.IndexerSubsysConfig;
import frc.robot.configuration.configs.IntakeSubsysConfig;

import frc.robot.subsystems.shooter.ShooterSubsystem;
// UNUSED: import frc.robot.subsystems.shooter.ShooterSubsystem.SysIdTarget;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// UNUSED: import swervelib.SwerveInputStream;
// UNUSED: import frc.robot.components.control.PID;
import frc.robot.components.controller.ControllerIfc;
// UNUSED: import frc.robot.components.controller.JoystickControllerIfc;
import frc.robot.components.controller.XboxControllerIfc;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// UNUSED: import frc.robot.configuration.Constants.SysIdEnums;
import frc.robot.configuration.Constants.SysIdEnums.SysIdTarget;

// UNUSED: import static edu.wpi.first.units.Units.Volts;

// UNUSED: import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  // create 2 instances of our new controller interface:
  private final ControllerIfc m_driverController = new XboxControllerIfc(OperatorConstants.controllerPort1);
  private final ControllerIfc m_operatorController = new XboxControllerIfc(OperatorConstants.controllerPort2);

  // Only for testing and tuning (such as SysId routines)
  private final ControllerIfc m_testingController = new XboxControllerIfc(OperatorConstants.controllerPort3);

  // subsystems:
  private final ShooterSubsysConfig ShooterSSConfig = new ShooterSubsysConfig(true, SHOOTER_SUBSYSTEM);
  private final IndexerSubsysConfig IndexerSSConfig = new IndexerSubsysConfig(true, INDEXER_SUBSYSTEM);
  private final IntakeSubsysConfig IntakeSSConfig = new IntakeSubsysConfig(true, INTAKE_SUBSYSTEM);
  private final VisionSubsysConfig VisionSSConfig = new VisionSubsysConfig(true, VISION_SUBSYSTEM);
  private final SwerveSubsysConfig SwerveSSConfig = new SwerveSubsysConfig(true,
      SWERVE_SUBSYSTEM,
      m_driverController,
      OperatorConstants.DEADBAND);

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(IntakeSSConfig);
  private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem(IndexerSSConfig);
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(VisionSSConfig);
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(SwerveSSConfig,
          () -> m_visionSubsystem.getLatestVisionMeasurement(), 
          new Pose2d(3.5, 4, new Rotation2d())); //change this value to modify our initial pose
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(ShooterSSConfig, m_swerveSubsystem);

  // auto command chooser
  private final SendableChooser<Command> autoChooser;

  // For sys id
  private static final SysIdTarget SYSID_TARGET = SysIdTarget.FEEDER;
  private static final boolean RUNNING_SYS_ID = IntakeConstants.RUNNING_SYS_ID || ShooterConstants.RUNNING_SYS_ID;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_ShooterSubsystem.setSysIdTarget(SYSID_TARGET);
    // Register Named Commands. These will be used in our auto routines.
    // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    // NamedCommands.registerCommand("exampleCommand",
    // exampleSubsystem.exampleCommand());
    // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());
    NamedCommands.registerCommand("Auto Shoot", new AutoShootCommand(m_ShooterSubsystem, m_IndexerSubsystem));

    // Configure the trigger bindings
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    // Auto chooser with specific autos has documentation if we want it.

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    // Make sure to check controller interfaces:
    if (RUNNING_SYS_ID) {
      m_testingController.runQuasistatic().whileTrue(
          m_ShooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

      m_testingController.runQuasistaticReverse().whileTrue(
          m_ShooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

      m_testingController.runQuasidynamic().whileTrue(
          m_ShooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

      m_testingController.runQuasidynamicReverse().whileTrue(
          m_ShooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    } else {
        //intake commands
      m_operatorController.sendPivotUp().onTrue(
          new InstantCommand(() -> m_IntakeSubsystem.sendPivotUp(), m_IntakeSubsystem));

      m_operatorController.sendPivotDown().onTrue(
        new SequentialCommandGroup(
          new InstantCommand(() -> m_IntakeSubsystem.sendPivotDown(), m_IntakeSubsystem).withTimeout(1.25),
          new WaitCommand(0.5),
          new InstantCommand(() -> m_IntakeSubsystem.stopPivotAlt(), m_IntakeSubsystem)));

      m_operatorController.toggleIntakeRollers().onTrue(
          new InstantCommand(() -> m_IntakeSubsystem.toggleRollers(), m_IntakeSubsystem));

      m_operatorController.toggleIndexer().onTrue(
          new InstantCommand(() -> m_IndexerSubsystem.toggleIndexer(), m_IndexerSubsystem));
    //shooter commands
    m_operatorController.toggleShooter().onTrue(
          new InstantCommand(() -> m_ShooterSubsystem.toggleShooter(), m_ShooterSubsystem));
    m_operatorController.closeShot().onTrue(
        new InstantCommand(() -> m_ShooterSubsystem.setCloseShot(), m_ShooterSubsystem).andThen(
            new InstantCommand(() -> m_ShooterSubsystem.toggleShooter(), m_ShooterSubsystem)));
    m_operatorController.farShot().onTrue(
        new InstantCommand(() -> m_ShooterSubsystem.setFarShot(), m_ShooterSubsystem).andThen(
        new InstantCommand(() -> m_ShooterSubsystem.toggleShooter(), m_ShooterSubsystem)));
    m_operatorController.autoShot().onTrue(
        new RunCommand(() -> m_ShooterSubsystem.solver(), m_ShooterSubsystem));
    }

    // m_driverController.driveToTarget().whileTrue(
    //     new DriveToTargetCommand(
    //         m_swerveSubsystem,
    //         m_visionSubsystem,
    //         null,
    //         1.0,
    //         0.05,
    //         0.035,
    //         Rotation2d.fromDegrees(0)).withTimeout(10));
    m_driverController.aimTowardsHub().whileTrue(
        new AimTowardsHubCommand(m_swerveSubsystem)); // Left trigger (can change)

    // Do not require the swerve subsystem for these InstantCommands so they don't
    // interrupt longer-running drive/aim commands that also require swerve.
    m_driverController.halfSpeedModifier().onTrue(
            new InstantCommand(() -> m_swerveSubsystem.setSpeedScaling(2))
        ).onFalse(
            new InstantCommand(() -> m_swerveSubsystem.setSpeedScaling(1.3))
        );           

    m_visionSubsystem.onCameraConnected.onTrue(
        new InstantCommand(() -> m_visionSubsystem.updateCameraStatus())
    ).onFalse(
        new InstantCommand(() -> m_visionSubsystem.updateCameraStatus())
    );

    
    m_driverController.toggleOrientation().onTrue(
        new InstantCommand(() -> m_swerveSubsystem.orientationToggle(), m_swerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return autoChooser.getSelected();
  }
}
