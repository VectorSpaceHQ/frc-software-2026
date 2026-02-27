package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import swervelib.SwerveInputStream;

import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.configuration.Constants.DriveToTargetConstants;
import frc.robot.configuration.Constants.VisionConstants.Apriltags;

import java.util.Optional;
import java.util.function.DoubleSupplier;

// Drives the robot to a set distance in front of an AprilTag.
// If tagId is null/None, uses the best visible tag at runtime.
public class DriveToTargetCommand extends Command {

        private final SwerveSubsystem swerve;
        private final VisionSubsystem vision;

        private final LoggedNetworkNumber loggedDistance;
        private final LoggedNetworkNumber loggedToleranceMeters;
        private final LoggedNetworkNumber loggedToleranceAngleRadians;
        private final LoggedNetworkNumber loggedHeadingOffsetDegrees;
        private final LoggedNetworkNumber loggedTranslationP;
        private final LoggedNetworkNumber loggedTranslationMaxVel;
        private final LoggedNetworkNumber loggedTranslationMaxAccel;
        private final LoggedNetworkNumber loggedRotationP;
        private final LoggedNetworkNumber loggedRotationMaxVel;
        private final LoggedNetworkNumber loggedRotationMaxAccel;

        private Apriltags tagId;
        private Apriltags activeTag; // for multiple schedules
        private DoubleSupplier zero;

        private SwerveInputStream inputStream;
        private Pose2d targetPose;
        private Pose2d robotPose;
        private Translation2d targetTranslation;
        private Translation2d targetOffset;
        private Rotation2d targetRotation;
        private double distanceError;
        private double rotationError;

        // Timeout for command if no tag is seen (for testing?)
        private final Timer tagSearchTimer = new Timer();
        private static final double timeout = 2.0;

        // Not final for tuning purposes
        private ProfiledPIDController translationPID;
        private ProfiledPIDController rotationPID;

        public DriveToTargetCommand(
                        SwerveSubsystem swerve,
                        VisionSubsystem vision,
                        Apriltags tagId,
                        double distanceMeters,
                        double toleranceMeters,
                        double toleranceAngleRadians,
                        Rotation2d headingOffset) {

                this.swerve = swerve;
                this.vision = vision;
                this.tagId = tagId;

                // For debugging: using constructor parameters as dashboard starting defaults
                // (tunable through Elastic and AdvantageScope)

                // We don't have to use sendable builder registries in this case (check
                // RobotContainer and Constants)
                loggedDistance = new LoggedNetworkNumber("DriveToTarget/DistanceMeters", distanceMeters); // 1 meter
                loggedToleranceMeters = new LoggedNetworkNumber("DriveToTarget/ToleranceMeters", toleranceMeters); // 0.05
                                                                                                                   // meters
                loggedToleranceAngleRadians = new LoggedNetworkNumber("DriveToTarget/ToleranceAngle",
                                toleranceAngleRadians); // 0.035 radians (2 degrees)
                loggedHeadingOffsetDegrees = new LoggedNetworkNumber("DriveToTarget/HeadingOffsetDeg",
                                headingOffset.getDegrees()); // Defaults to zero degrees
                loggedTranslationP = new LoggedNetworkNumber("DriveToTarget/TranslationP",
                                DriveToTargetConstants.TRANSLATION_P);
                loggedTranslationMaxVel = new LoggedNetworkNumber("DriveToTarget/TranslationMaxVel",
                                DriveToTargetConstants.TRANSLATION_MAX_VEL);
                loggedTranslationMaxAccel = new LoggedNetworkNumber("DriveToTarget/TranslationMaxAccel",
                                DriveToTargetConstants.TRANSLATION_MAX_ACCEL);
                loggedRotationP = new LoggedNetworkNumber("DriveToTarget/RotationP",
                                DriveToTargetConstants.ROTATION_P);
                loggedRotationMaxVel = new LoggedNetworkNumber("DriveToTarget/RotationMaxVel",
                                DriveToTargetConstants.ROTATION_MAX_VEL);
                loggedRotationMaxAccel = new LoggedNetworkNumber("DriveToTarget/RotationMaxAccel",
                                DriveToTargetConstants.ROTATION_MAX_ACCEL);

                // Can be tuned through dashboards as well
                translationPID = new ProfiledPIDController(
                                DriveToTargetConstants.TRANSLATION_P,
                                DriveToTargetConstants.TRANSLATION_I,
                                DriveToTargetConstants.TRANSLATION_D,
                                new TrapezoidProfile.Constraints(
                                                DriveToTargetConstants.TRANSLATION_MAX_VEL,
                                                DriveToTargetConstants.TRANSLATION_MAX_ACCEL));

                rotationPID = new ProfiledPIDController(
                                DriveToTargetConstants.ROTATION_P,
                                DriveToTargetConstants.ROTATION_I,
                                DriveToTargetConstants.ROTATION_D,
                                new TrapezoidProfile.Constraints(
                                                DriveToTargetConstants.ROTATION_MAX_VEL,
                                                DriveToTargetConstants.ROTATION_MAX_ACCEL));

                rotationPID.enableContinuousInput(-Math.PI, Math.PI); // Takes shortest path

                translationPID.setTolerance(toleranceMeters); // Values that count as close enough
                translationPID.setIntegratorRange(-0.1, 0.1); // Unused

                rotationPID.setTolerance(toleranceAngleRadians);
                rotationPID.setIntegratorRange(-0.1, 0.1); // Unused

                addRequirements(swerve);
        }

        @Override
        public void initialize() {
                // Start at an empty (zeroed) state
                targetPose = null;
                inputStream = null;
                robotPose = swerve.getEstimatedPose();

                // error each cycle, so resetting to zero here can cause an initial velocity
                // spike
                // (SwerveInputStream calculates translationPID error)
                rotationPID.reset(robotPose.getRotation().getRadians()); // Reset to current heading rather than zero

                // Start the tag search timeout so the command doesn't wait forever
                tagSearchTimer.restart();

                Logger.recordOutput("DriveToTarget/Status", "Waiting For Tag...");
        }

        @Override
        public void execute() {
                // Fetch the latest robot pose once per loop so all calculations use the same
                // value
                robotPose = swerve.getEstimatedPose();

                // Re-apply tolerances and PID values each loop in case values were changed in
                // dashboard

                // Will have to repeat for integral and derivative gains if we are using those
                // Proportional gain probably has to be adjusted but that is later
                translationPID.setTolerance(loggedToleranceMeters.get());
                rotationPID.setTolerance(loggedToleranceAngleRadians.get());

                translationPID.setP(loggedTranslationP.get());
                translationPID.setConstraints(new TrapezoidProfile.Constraints(
                                loggedTranslationMaxVel.get(),
                                loggedTranslationMaxAccel.get()));
                rotationPID.setP(loggedRotationP.get());

                rotationPID.setConstraints(new TrapezoidProfile.Constraints(
                                loggedRotationMaxVel.get(),
                                loggedRotationMaxAccel.get()));

                // Lock in the target pose (once) on the first valid vision frame
                // tagId is set to null in the constructor in RobotContainer, so basically
                // if (any tag is visible) or (no tag is visible)
                if (targetPose == null) {
                        activeTag = tagId;
                        Optional<Apriltags> seenTag = vision.getBestVisibleTag();

                        if (activeTag == null || activeTag == Apriltags.None) {
                                // If it is not a part of the Apriltags enum (Constants)
                                if (seenTag.isEmpty()) {
                                        return;
                                }
                                activeTag = seenTag.get();
                        }

                        // Get the pose of that tag from the 2026 Rebuilt layout (from vision)
                        Pose2d tagPose = vision.getLayout()
                                        .getTagPose(activeTag.getId())
                                        .map(p -> p.toPose2d()) // converts from 3d to 2d pose
                                        .orElse(null);

                        if (tagPose == null) { // If there is no pose of the april tag yet, then continue
                                return;
                        }

                        // Place target point in -x direction (in front of the tag along its normal /
                        // tag-space).
                        targetOffset = new Translation2d(-loggedDistance.get(), 0.0)
                                        .rotateBy(tagPose.getRotation());

                        targetTranslation = tagPose.getTranslation().plus(targetOffset); // Translates from
                                                                                         // tag-relative to
                                                                                         // field-relative

                        // Robot should face the tag (opposite of the tag's heading)
                        targetRotation = tagPose.getRotation()
                                        .plus(Rotation2d.fromDegrees(180))
                                        .plus(Rotation2d.fromDegrees(loggedHeadingOffsetDegrees.get()));

                        targetPose = new Pose2d(targetTranslation, targetRotation); // Create target pose
                        Logger.recordOutput("DriveToTarget/Status", "Found Tag.");
                        Logger.recordOutput("DriveToTarget/TargetPose", targetPose);
                        Logger.recordOutput("DriveToTarget/ActiveTargetTagId", activeTag.getId());

                        // Zero out joystick axes to suppress user control so only the PIDs control
                        // motion
                        zero = () -> 0.0;
                        inputStream = SwerveInputStream
                                        .of(swerve.getSwerveDrive(), zero, zero)
                                        .withControllerRotationAxis(zero)
                                        .driveToPose(
                                                        () -> targetPose,
                                                        translationPID,
                                                        rotationPID);

                        inputStream.driveToPoseEnabled(true);
                }

                // Compute chassis speeds to drive towards target pose
                if (inputStream != null) {
                        swerve.getSwerveDrive().drive(inputStream.get());

                        // For debugging
                        Logger.recordOutput("DriveToTarget/RobotPose", robotPose);

                        distanceError = robotPose.getTranslation()
                                        .getDistance(targetPose.getTranslation());
                        Logger.recordOutput("DriveToTarget/DistanceErrorMeters", distanceError);

                        rotationError = Math.abs(robotPose.getRotation() // Ensure it is positive
                                        .minus(targetPose.getRotation())
                                        .getRadians());
                        Logger.recordOutput("DriveToTarget/RotationErrorRadians", rotationError);
                }
        }

        @Override
        public boolean isFinished() {
                // End early if no tag was found within the timeout window
                if (targetPose == null) {
                        if (tagSearchTimer.hasElapsed(timeout)) {
                                Logger.recordOutput("DriveToTarget/Status", "Tag Search Timed Out :(");
                                return true;
                        }
                        return false;
                }

                // Avoid stale error values
                if (inputStream == null) {
                        return false;
                }

                // Recompute errors fresh from current pose rather than relying on cached values
                // (not global)
                Pose2d currentPose = swerve.getEstimatedPose();
                double currentDistanceError = currentPose.getTranslation()
                                .getDistance(targetPose.getTranslation());
                double currentRotationError = Math.abs(currentPose.getRotation()
                                .minus(targetPose.getRotation())
                                .getRadians());

                // Check that the robot has slowed down to avoid finishing with remaining
                // momentum (and crashing into something, also not global)
                ChassisSpeeds velocity = swerve.getVelocity();
                double speed = Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond); // Hypotenuse (using
                                                                                                   // x- and
                                                                                                   // y-components)

                return currentDistanceError < loggedToleranceMeters.get()
                                && currentRotationError < loggedToleranceAngleRadians.get()
                                && speed < 0.1; // Command finishes when robot is close enough and nearly stopped
        }

        @Override
        public void end(boolean interrupted) {

                if (inputStream != null) {
                        inputStream.driveToPoseEnabled(false); // Disables driving to pose
                }
                // Stops the drive by calling a new empty chassis speeds (defaulted to zero)
                swerve.stopDrive();

                if (interrupted) {
                        Logger.recordOutput("DriveToTarget/Status", "Interrupted");
                } else {
                        Logger.recordOutput("DriveToTarget/Status", "Finished");
                }
        }
}