
package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public AutoShootCommand(ShooterSubsystem Shooter, IndexerSubsystem Indexer) {
    this.shooterSubsystem = Shooter;
    this.indexerSubsystem = Indexer;
    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(shooterSubsystem.getShooterStatus() == false){
      shooterSubsystem.toggleShooter(); //enable shooter if disabled
    }
    if(indexerSubsystem.getIndexerStatus() == false){
      indexerSubsystem.toggleIndexer(); //enable indexer if disabled
      indexerSubsystem.setIndexerRPM(100); //set indexer RPM
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setEnglishVelocity(0);
    shooterSubsystem.setMainVelocity(0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    shooterSubsystem.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
