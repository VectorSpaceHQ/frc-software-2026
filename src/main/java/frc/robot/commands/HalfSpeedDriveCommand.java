package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.configuration.Constants.ShooterConstants;
import frc.robot.configuration.Constants.VisionConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class HalfSpeedDriveCommand extends Command {

    private final SwerveSubsystem swerve;

    public HalfSpeedDriveCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("AimTowardsHub/Status", "Initialized");
    }

    @Override
    public void execute() {
        SmartDashboard.putString("AimTowardsHub/Status", "Executing");
        // Drive using the dedicated half speed stream in Swerve. Feed raw speeds directly
        // to the drive method.
        swerve.getSwerveDrive().drive(swerve.getHalfSpeedStream().get());

    }


    @Override
    public void end(boolean interrupted) {

    }

}
