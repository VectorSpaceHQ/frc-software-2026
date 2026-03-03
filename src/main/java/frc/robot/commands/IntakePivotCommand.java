
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakePivotCommand extends Command {
  private final IntakeSubsystem intake;
  private double targetPivotAngle;

  public IntakePivotCommand(IntakeSubsystem intakeSubsystem, double targetAngle) {
    this.intake = intakeSubsystem;
    this.targetPivotAngle = targetAngle;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPivotTarget(targetPivotAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runPivot();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.pivotAtPosition();
  }
}
