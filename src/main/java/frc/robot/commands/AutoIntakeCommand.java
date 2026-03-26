package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class AutoIntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;

  public AutoIntakeCommand(IntakeSubsystem intake) {
    this.intakeSubsystem = intake;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    // Turn indexer and shooter on
    intakeSubsystem.sendPivotDown();
    if(!intakeSubsystem.getIntakestatus()){
      //if rollers are disabled, enable rollers
      intakeSubsystem.toggleRollers();
    }
  }

  @Override
  public void execute() {
    // Do I keep this here?

  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopPivotAlt();
    intakeSubsystem.toggleRollers();
  }

  @Override
  public boolean isFinished() {
    // Stay running until 15 seconds of auto ends
    return false;
  }
}