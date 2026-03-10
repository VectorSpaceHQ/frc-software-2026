
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommands extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private double targetPivotAngle;

  public ShooterCommands(ShooterSubsystem Shooter) {
    this.shooterSubsystem = Shooter;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//    shooterSubsystem.setSysIdTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.toggleShooter();

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
