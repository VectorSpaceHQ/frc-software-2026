package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.SwerveSubsystem;

public class HalfSpeedDriveCommand extends Command {

    private final SwerveSubsystem swerve;

    public HalfSpeedDriveCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Half Speed Drive", "Enabled");
    }

    @Override
    public void execute() {
        // Drive using the dedicated half speed stream in Swerve. Feed raw speeds directly
        // to the drive method.
        swerve.getSwerveDrive().driveFieldOriented(swerve.getHalfSpeedStream().get());

    }


    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Half Speed Drive", "Disabled");
    }

}
