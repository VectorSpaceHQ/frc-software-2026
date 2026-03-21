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

  public double getTwistY();

  public Trigger toggleOrientation();

  public Trigger halfSpeedModifier();

  // Shooter
  public Trigger startShooter();

  public Trigger stopShooter();

  public Trigger toggleShooter();

  public Trigger closeShot();

  public Trigger farShot();

  public Trigger autoShot();

  // Intake
  public Trigger toggleIntakeRollers();

  public Trigger sendPivotUp();

  public Trigger sendPivotDown();

  // Indexer
  public Trigger toggleIndexer();

  // Climb (TODO)
  public Trigger runClimbUp();

  public Trigger runClimbDown();

  public Trigger stopClimb();

  // Vision
  public Trigger driveToTarget();

  public Trigger aimTowardsHub();

  // Tuning
  public Trigger runQuasistatic();

  public Trigger runQuasidynamic();

  public Trigger runQuasistaticReverse();

  public Trigger runQuasidynamicReverse();

  // Put Autos here:

}
