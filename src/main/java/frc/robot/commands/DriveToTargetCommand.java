package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        private LoggedNetworkNumber loggedDistance;
        private LoggedNetworkNumber loggedToleranceMeters;
        private LoggedNetworkNumber loggedToleranceAngleRadians;
        private LoggedNetworkNumber loggedHeadingOffsetDegrees;
        private LoggedNetworkNumber loggedTranslationP;
        private LoggedNetworkNumber loggedTranslationMaxVel;
        private LoggedNetworkNumber loggedTranslationMaxAccel;
        private LoggedNetworkNumber loggedRotationP;
        private LoggedNetworkNumber loggedRotationMaxVel;
        private LoggedNetworkNumber loggedRotationMaxAccel;

        private Apriltags tagId;
        private Apriltags activeTag; // for multiple schedules

        private SwerveInputStream inputStream;
        private Pose2d targetPose;

        private Translation2d targetTranslation;
        private Rotation2d targetRotation;

        // Timeout for command if no tag is seen (for testing?)
        private Timer tagSearchTimer = new Timer();
        private double timeout = 5.0;

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
                targetTranslation = null;
                targetRotation = null;
                inputStream = null;
                activeTag = null;

                // Start the tag search timeout so the command doesn't wait forever
                tagSearchTimer.restart();

                Logger.recordOutput("DriveToTarget/Status", "Waiting For Tag...");
        }

        @Override
        public void execute() {
                // Fetch the latest robot pose once per loop so all calculations use the same
                // value
                Pose2d robotPose = swerve.getEstimatedPose();

                // Re-apply tolerances and PID values each loop in case values were changed in
                // dashboard
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
                        Optional<Apriltags> seenTag = vision.getBestVisibleTag();

                        // Switch to last seen tag if no tag is currently visible
                        if (seenTag.isEmpty() && activeTag != null) {
                                seenTag = Optional.of(activeTag);
                        }

                        // If there was no seen tag but a tagId was provided in the constructor, use its
                        // field pose instead
                        if (seenTag.isEmpty() && activeTag == null && tagId != null && tagId != Apriltags.None) {
                                seenTag = Optional.of(tagId);
                        }

                        // Don't do anything when there was no tag ever visible (or if not a part of the
                        // enum)
                        if (seenTag.isEmpty()) {
                                return;
                        }

                        // If tag is seen and could be any tag, or there is no tag, or tag seen is same
                        // as one in command (which is null)
                        if (tagId == null || tagId == Apriltags.None || tagId == seenTag.get()) {

                                activeTag = seenTag.get(); // Change seen tag to active tag (see above)

                                // Get the pose of that tag from the 2026 Rebuilt layout (from vision)
                                Pose2d tagPose = vision.getLayout()
                                                .getTagPose(activeTag.getId())
                                                .map(p -> p.toPose2d()) // converts from 3d to 2d pose
                                                .orElse(null);

                                if (tagPose != null) {

                                        // Place target point in -x direction (in front of the tag along its normal /
                                        // tag-space)
                                        targetTranslation = new Translation2d(-loggedDistance.get(), 0.0)
                                                        .rotateBy(tagPose.getRotation());

                                        // Robot should face the tag (opposite of the tag's heading)
                                        targetRotation = tagPose.getRotation()
                                                        .plus(Rotation2d.fromDegrees(180))
                                                        .plus(Rotation2d.fromDegrees(loggedHeadingOffsetDegrees.get()));

                                        targetPose = new Pose2d(targetTranslation, targetRotation); // Create target
                                                                                                    // pose

                                        // Reset PIDs to prevent spikes
                                        translationPID.reset(
                                                        robotPose.getTranslation()
                                                                        .getDistance(targetTranslation));

                                        rotationPID.reset(robotPose.getRotation().getRadians()); // Reset to current
                                                                                                 // heading rather than
                                                                                                 // zero?

                                        Logger.recordOutput("DriveToTarget/Status", "Target Locked");
                                        Logger.recordOutput("DriveToTarget/TargetPose", targetPose);
                                        Logger.recordOutput("DriveToTarget/ActiveTargetTagId", activeTag.getId());
                                }
                        }
                }

                // Zero out joystick axes to suppress user control so only the PIDs control
                // motion

                if (targetPose != null && inputStream == null) {

                        DoubleSupplier zero = () -> 0.0;

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
                        double distanceError = robotPose.getTranslation()
                                        .getDistance(targetPose.getTranslation());

                        Logger.recordOutput("DriveToTarget/DistanceErrorMeters", distanceError);
                        Logger.recordOutput("DriveToTarget/RotationErrorRadians",
                                        Math.abs(rotationPID.getPositionError()));
                }
        }

        @Override
        public boolean isFinished() {
                boolean isFinished = false;
                if (targetPose == null) {

                        // When it reaches timeout without a target
                        isFinished = tagSearchTimer.hasElapsed(timeout);

                        // If we have a target and valid stream
                } else if (inputStream != null) {

                        // Finish when PIDs reach their goals
                        isFinished = translationPID.atGoal() && rotationPID.atGoal();
                } else {

                        isFinished = false;
                }

                return isFinished;

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