// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.Constants;
import static frc.robot.Constants.OperatorConstants.SubSystemIDEnum.*;
//import frc.robot.commands.ControllerCommand;
import frc.robot.commands.ExampleCommand;
import frc.Interfaces.ControllerIfc;
import frc.Interfaces.JoystickControllerIfc;
import frc.Interfaces.XboxControllerIfc;
import frc.robot.components.motor.MotorIO;
import frc.robot.components.motor.MotorIOKraken;
import frc.robot.subsystems.ExampleSubsystem;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SubsystemConfig;
import frc.robot.subsystems.ShooterSubsysConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //subsystems:
  private final ShooterSubsysConfig ShooterSSConfig = new ShooterSubsysConfig(true, SHOOTER_SUBSYSTEM);
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(ShooterSSConfig);
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  //create 2 instances of our new controller interface:
  private final ControllerIfc m_driverController = new XboxControllerIfc(OperatorConstants.controllerPort1);
  private final ControllerIfc m_operatorController = new XboxControllerIfc(OperatorConstants.controllerPort2);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getY() * -1,
                                                                () -> m_driverController.getX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getTwist)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getX,
                                                                                             m_driverController::getY)
                                                           .headingWhile(true);

  Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));

     //new Trigger(m_Shooter::commandShooter)
         //.onTrue(new ControllerCommand(m_driverController));
    
    // new Trigger(m_driverController::commandShooter)
    //     .onTrue(new ControllerCommand(m_driverController));      

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    //TODO: Temporary start motor (Move to operator controller?)
    // m_driverController.runIntake().onTrue(
    //     new InstantCommand( () -> 
    //     m_motor.setVoltage(6.0))
    // );
    
    // Temporary stop motor
    // m_driverController.stopIntake().onTrue(
    //     new InstantCommand( () -> 
    //     m_motor.stop())
    // );

    //TODO: Should this be on the operator controller?
   /*  m_driverController.runShooter().whileTrue(
      new RunCommand( () -> {
        double trigger = m_driverController.controlMotorSpeed();
        double targetRPM = (trigger * MAX_RPM);
        double targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(targetRPM);
        double volts = feedforward.calculate(targetRadsPerSec);
        m_motor.setVoltage(volts);
      }).withTimeout(3) 
    );*/

    //TODO: Replace onchange when class is futher developed (and move to operator controller?)
    m_operatorController.runShooter().onTrue(
      new InstantCommand( () -> 
        m_ShooterSubsystem.toggleShoot())
    
    );

    //TODO: Is this supposed to be on the operator controller?
    //m_driverController.runIntake().onTrue(
    //  new InstantCommand( () -> 
    //  m_IntakeSubsystem.toggleIntake())
    //);
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
