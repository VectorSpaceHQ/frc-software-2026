package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.configuration.Constants.ShooterConstants;
import frc.robot.configuration.Constants.VisionConstants;

public class AimTowardsHubCommand extends Command {

    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    private Rotation2d lastTargetHeading = new Rotation2d();
    private boolean isCurrentlyAligned = false;

    public AimTowardsHubCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
        this.swerve = swerve;
        this.vision = vision;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("AimTowardsHub/Status", "Initializing");

        swerve.setAimTargetSupplier(() -> {
            Pose2d robotPose = swerve.getEstimatedPose();
            // TODO: Update with the correct/new methods to get hubcenter and distance to hub
           
            Pose2d goalPosition = ShooterConstants.blueHubCenter;
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    goalPosition = ShooterConstants.redHubCenter;
                } else {
                    goalPosition = ShooterConstants.blueHubCenter;
                }
            }
            lastTargetHeading = goalPosition.getRotation();
            return goalPosition;
  
        });

        swerve.setAiming(true);
        // TODO: Change loggers to sendables (optional)?
        Logger.recordOutput("AimTowardsHub/Status", "Aiming Enabled (Actual Hub Target)");
        SmartDashboard.putString("AimTowardsHub/Status", "Initialized");
    }

    @Override
    public void execute() {
        SmartDashboard.putString("AimTowardsHub/Status", "Executing");

        // Drive using the dedicated aiming stream in Swerve (includes driver translation but needs testing).
        // Feed raw speeds directly to the drive method.
        swerve.getSwerveDrive().drive(swerve.getAimStream().get());

        // Calculate error
    //   Rotation2d shooterHeading = swerve.getEstimatedPose().getRotation().plus(Rotation2d.fromDegrees(90));
        Rotation2d shooterHeading = swerve.getEstimatedPose().getRotation();

        // Wrap around to find the smallest distance using minus (in radians, could change).
        double errorRads = Math.abs(lastTargetHeading.minus(shooterHeading).getRadians());
        // Return true of target is within 3 degrees (converting from degrees -> radians -> degrees is a little redundant on my part).
        // Add 90 degrees to account for shooter
   
        isCurrentlyAligned = swerve.getAimStream()
                            .aimLock(Units.Radians.of(VisionConstants.ROTATION_TOLERANCE_RAD))
                            .getAsBoolean();

        // Telemetry.
        // TODO: Update with the correct/new methods to get hubcenter and distance to hub
        // Translation3d targetHub = vision.getTargetHubCenter();
        // if (targetHub != null) {
        //     double currentDistance = vision.getDistanceToHub(swerve.getEstimatedPose(), targetHub);
        //     Logger.recordOutput("AimTowardsHub/CurrentDistance", currentDistance);
        //     SmartDashboard.putNumber("AimTowardsHub/CurrentDistance", currentDistance);
        // }

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

    public boolean isAligned() { // Could be used for auto or feedback (as an alternative)
        return isCurrentlyAligned;
    }

    @Override
    public boolean isFinished() {
        return isCurrentlyAligned;
    }

    @Override
    public void end(boolean interrupted) {
        // Reset everything
        swerve.setAiming(false);
        isCurrentlyAligned = false;
        swerve.setAimTargetSupplier(swerve::getEstimatedPose);
        swerve.stopDrive();

        Logger.recordOutput("AimTowardsHub/Status", interrupted ? "Interrupted" : "Finished");
        Logger.recordOutput("AimTowardsHub/IsAligned", false);
        
        SmartDashboard.putString("AimTowardsHub/Status", interrupted ? "Interrupted" : "Finished");
    }

    // Note: I found that if you want to have shoot on the move, along with the aimLookAhead (necessary) and feedforward (useful), you would need to calculate where the hub position should be based on the velocity of the robot and the time the ball is in the air (based on the trajectory calculations, probably in solver or somewhere). Then you can add an offset (like a fake hub) that accounts for these things (otherwise shooting would be off while moving even with the other two methods). I can't do this right now. But anyway, first, you should test to see if the robot aims towards the hub. Some parameters may need tuning (for example, if it overshoots).
}