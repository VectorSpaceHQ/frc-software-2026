// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.controller;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.Constants.OperatorConstants.ControllerEnum;
//import edu.wpi.first.wpilibj.Joystick;

public interface ControllerIfc {

  // Swerve
  public double getX();

  public double getY();

  public double getTwist();

  public Trigger toggleOrientation();

  // Shooter
  public Trigger runShooter();

  public Trigger stopShooter();

  // Intake
  public Trigger toggleIntakeRollers();

  public Trigger toggleIntakePivot();

  // Indexer
  public Trigger toggleIndexer();

  // Climb
  public Trigger runClimb();

  public Trigger stopClimb();

  // Vision
  public Trigger driveToTarget();

  // Tuning
  public Trigger runQuasistatic();

  public Trigger runQuasidynamic();

  public Trigger runQuasistaticReverse();

  public Trigger runQuasidynamicReverse();

  // Auto
  // public Trigger exampleAutoCommand();

}
