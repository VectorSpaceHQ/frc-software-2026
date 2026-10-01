package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class AutoIndexCommand extends Command {
  private final IndexerSubsystem indexerSubsystem;

  public AutoIndexCommand(IndexerSubsystem indexer) {
    this.indexerSubsystem = indexer;
    addRequirements(indexerSubsystem);
  }

  @Override
  public void initialize() {
    // Turn indexer on

    if (!indexerSubsystem.getIndexerStatus()) {
      indexerSubsystem.toggleIndexer();
    }
    indexerSubsystem.setIndexerRPM(2500); 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  // Turn indexer off
    if (indexerSubsystem.getIndexerStatus()) {
      indexerSubsystem.toggleIndexer();
    }

  }

  @Override
  public boolean isFinished() {
    // Stay running until 15 seconds of auto ends
    return false;
  }
}