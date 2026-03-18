package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public AutoShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
    this.shooterSubsystem = shooter;
    this.indexerSubsystem = indexer;
    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {
    // Turn indexer and shooter on
    if (!shooterSubsystem.getShooterStatus()) {
      shooterSubsystem.toggleShooter();
    }
    if (!indexerSubsystem.getIndexerStatus()) {
      indexerSubsystem.toggleIndexer();
    }

    // Set speeds
    shooterSubsystem.setCloseShot();
    indexerSubsystem.setIndexerRPM(2500); 
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
    if (indexerSubsystem.getIndexerStatus()) {
      indexerSubsystem.toggleIndexer();
    }

    // Set shooter rpms to zero
    shooterSubsystem.zeroRPM();
    indexerSubsystem.setIndexerRPM(0);
  }

  @Override
  public boolean isFinished() {
    // Stay running until 15 seconds of auto ends
    return false;
  }
}