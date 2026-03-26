package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoShooterSpinUpCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;

  public AutoShooterSpinUpCommand(ShooterSubsystem shooter) {
    this.shooterSubsystem = shooter;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    // Turn indexer and shooter on
    if (!shooterSubsystem.getShooterStatus()) {
      shooterSubsystem.toggleShooter();
    }

    // Set speeds to shooter wheels, set feeder wheel rpm to 0.
    shooterSubsystem.setAutoShot();
    shooterSubsystem.setFeederRPM(0);
  }

  @Override
  public void execute() {
    // Do I keep this here?
    shooterSubsystem.setCloseShot();
  }

  @Override
  public void end(boolean interrupted) {
  // Turn shooter and indexer off
    if (shooterSubsystem.getShooterStatus()) {
      shooterSubsystem.toggleShooter();
    }

    // Set shooter rpms to zero
    shooterSubsystem.zeroRPM();
  }

  @Override
  public boolean isFinished() {
    // Stay running until 15 seconds of auto ends
    return false;
  }
}