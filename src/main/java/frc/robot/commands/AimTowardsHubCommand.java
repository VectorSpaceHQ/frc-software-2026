package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import swervelib.SwerveInputStream;

import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.configuration.Constants.DriveToTargetConstants;

import java.util.function.DoubleSupplier;

public class AimTowardsHubCommand extends Command {

    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    private final LoggedNetworkNumber loggedRotationP;
    private final LoggedNetworkNumber loggedRotationMaxVel;
    private final LoggedNetworkNumber loggedRotationMaxAccel;
    private final LoggedNetworkNumber loggedToleranceAngleRadians;

    private final ProfiledPIDController rotationPID;
    
    private SwerveInputStream inputStream;
    private Translation3d targetHub;

    public AimTowardsHubCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
        this.swerve = swerve;
        this.vision = vision;

        loggedRotationP = new LoggedNetworkNumber("AimTowardsHub/RotationP", DriveToTargetConstants.ROTATION_P);
        loggedRotationMaxVel = new LoggedNetworkNumber("AimTowardsHub/RotationMaxVel", DriveToTargetConstants.ROTATION_MAX_VEL);
        loggedRotationMaxAccel = new LoggedNetworkNumber("AimTowardsHub/RotationMaxAccel", DriveToTargetConstants.ROTATION_MAX_ACCEL);
        loggedToleranceAngleRadians = new LoggedNetworkNumber("AimTowardsHub/ToleranceAngleRad", Math.toRadians(1.5));

        rotationPID = new ProfiledPIDController(
                DriveToTargetConstants.ROTATION_P,
                DriveToTargetConstants.ROTATION_I,
                DriveToTargetConstants.ROTATION_D,
                new TrapezoidProfile.Constraints(
                        DriveToTargetConstants.ROTATION_MAX_VEL,
                        DriveToTargetConstants.ROTATION_MAX_ACCEL));

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setIntegratorRange(-1, 1);
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        targetHub = null;
        inputStream = null;
        Logger.recordOutput("AimTowardsHub/Status", "Initializing...");
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getEstimatedPose();

        rotationPID.setP(loggedRotationP.get());
        rotationPID.setConstraints(new TrapezoidProfile.Constraints(
                loggedRotationMaxVel.get(),
                loggedRotationMaxAccel.get()));
        rotationPID.setTolerance(loggedToleranceAngleRadians.get());

        if (targetHub == null) {
            targetHub = vision.getTargetHubCenter();
            rotationPID.reset(robotPose.getRotation().getRadians());
            
        }
            Logger.recordOutput("AimTowardsHub/Status", "Locked on Hub");
        if (inputStream == null) {
            DoubleSupplier zero = () -> 0.0;
            
            inputStream = SwerveInputStream.of(swerve.getSwerveDrive(), zero, zero)
                    .withControllerRotationAxis(() -> {
                        double rawRadPerSec = rotationPID.calculate(
                            swerve.getEstimatedPose().getRotation().getRadians(), 
                            vision.getHeadingToHub(swerve.getEstimatedPose(), targetHub).getRadians()
                        );
                        // Normalize to [-1.0, 1.0] for YAGSL
                        return rawRadPerSec / loggedRotationMaxVel.get();
                    });
        }

        swerve.getSwerveDrive().drive(inputStream.get());

        // Telemetry
        double currentDistance = vision.getDistanceToHub(robotPose, targetHub);
        
        Logger.recordOutput("AimTowardsHub/TargetDistanceMeters", currentDistance);
        Logger.recordOutput("AimTowardsHub/RotationErrorRads", rotationPID.getPositionError());
        Logger.recordOutput("AimTowardsHub/AtTarget", rotationPID.atGoal());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopDrive();
        Logger.recordOutput("AimTowardsHub/Status", interrupted ? "Interrupted" : "Finished");
    }
}