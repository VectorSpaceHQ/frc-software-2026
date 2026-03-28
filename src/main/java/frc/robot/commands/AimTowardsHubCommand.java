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

public class AimTowardsHubCommand extends Command {

    private final SwerveSubsystem swerve;

    private Rotation2d lastTargetHeading = new Rotation2d();
    private boolean isCurrentlyAligned = false;
    private static boolean isLogging = false;
    private final boolean isAutonomous;

    // Teleop constructor (auto false)
    public AimTowardsHubCommand(SwerveSubsystem swerve) {
        this(swerve, false);
    }

    // Auto constructor (auto true)
    public AimTowardsHubCommand(SwerveSubsystem swerve, boolean isAutonomous) {
        this.swerve = swerve;
        this.isAutonomous = isAutonomous;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("AimTowardsHub/Status", "Initializing");

        swerve.setAimTargetSupplier(() -> {
            
            Translation2d shooterOffset;
            Pose2d goalPosition = ShooterConstants.blueHubCenter;
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    goalPosition = ShooterConstants.redHubCenter;
                    // shooterOffset = new Translation2d(
                    //     -VisionConstants.TRANSLATION_X,
                    //     -VisionConstants.TRANSLATION_Y);
                        shooterOffset = new Translation2d(
                        -0.25,
                        -0.25);
                } else {
                    goalPosition = ShooterConstants.blueHubCenter;

                    // shooterOffset = new Translation2d(
                    //     VisionConstants.TRANSLATION_X,
                    //     VisionConstants.TRANSLATION_Y);
                        shooterOffset = new Translation2d(
                        0.25,
                        0.25);
                }
            } else{
                shooterOffset = new Translation2d();
            }
            // shooterOffset = new Translation2d();
            // Translation2d shooterOffset = new Translation2d(
            //         VisionConstants.TRANSLATION_X,
            //         VisionConstants.TRANSLATION_Y);

            // Shift the goal offset by the shooter translation
            Translation2d shiftedGoalPosition = goalPosition.getTranslation().minus(shooterOffset);
            lastTargetHeading = shiftedGoalPosition.getAngle();

            return new Pose2d(shiftedGoalPosition, new Rotation2d());
        });

        swerve.setAiming(true);
        SmartDashboard.putString("AimTowardsHub/Status", "Initialized");
    }

    @Override
    public void execute() {
        SmartDashboard.putString("AimTowardsHub/Status", "Executing");

        Rotation2d shooterHeading = swerve.getEstimatedPose().getRotation();

        // Wrap around to find the smallest distance using minus
        double errorRads = Math.abs(lastTargetHeading.minus(shooterHeading).getRadians());

        isCurrentlyAligned = swerve.getAimStream()
                // Within 10 degrees
                .aimLock(Units.Radians.of(VisionConstants.ROTATION_TOLERANCE_RAD))
                .getAsBoolean();
        if (isLogging()) {
            Logger.recordOutput("AimTowardsHub/RotationErrorDeg", Math.toDegrees(errorRads));
            Logger.recordOutput("AimTowardsHub/RotationErrorRads", errorRads);
            Logger.recordOutput("AimTowardsHub/IsAligned", isCurrentlyAligned);

            Logger.recordOutput("AimTowardsHub/TargetHeading", lastTargetHeading);
            Logger.recordOutput("AimTowardsHub/CurrentLeftHeading", shooterHeading);

            SmartDashboard.putNumber("AimTowardsHub/RotationErrorDeg", Math.toDegrees(errorRads));
            SmartDashboard.putNumber("AimTowardsHub/RotationErrorRads", errorRads);
            SmartDashboard.putNumber("AimTowardsHub/shooterHeading", shooterHeading.getDegrees());
            SmartDashboard.putString("AimTowardsHub/Status", "Executed");
        }
    }

    public boolean isAligned() {
        return isCurrentlyAligned;
    }

    public boolean isLogging() {
        return isLogging;
    }

    public void toggleLogging() {
        isLogging = !isLogging;
    }

    @Override
    public boolean isFinished() {
        if (isAutonomous) {
            // In auto, finish the command when aligned
            return isCurrentlyAligned;
        } else {
            // In teleop, don't finish until button isn't held anymore
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        isCurrentlyAligned = false;
        swerve.setAiming(false);
        swerve.setAimTargetSupplier(swerve::getEstimatedPose);

        if (isLogging()) {
            Logger.recordOutput("AimTowardsHub/Status", interrupted ? "Interrupted" : "Finished");
            Logger.recordOutput("AimTowardsHub/IsAligned", false);

            SmartDashboard.putString("AimTowardsHub/Status", interrupted ? "Interrupted" : "Finished");
        }
    }

}
