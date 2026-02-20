package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import frc.robot.subsystems.vision.VisionSubsystem;

public class PoseEstimatorSubsystem extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEstimator;
    private VisionSubsystem visionSubsystem;

    public PoseEstimatorSubsystem(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPose) {

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                gyroAngle,
                modulePositions,
                initialPose,
                VecBuilder.fill(0.1, 0.1, 0.05), // State std devs (meters, meters, radians): needs to be detrmined
                                                 // experimentally
                VecBuilder.fill(0.9, 0.9, 0.9)); // Vision measurement std devs, also);
    }

    public void update(
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyroAngle, modulePositions);
        poseEstimator.update(gyroAngle, modulePositions);

        if (visionSubsystem.getLatestVisionMeasurement().isPresent()) {
            EstimatedRobotPose visionMeasurement = visionSubsystem.getLatestVisionMeasurement().get();
            poseEstimator.addVisionMeasurement(
                    visionMeasurement.estimatedPose.toPose2d(),
                    visionMeasurement.timestampSeconds);
        }

    }

    public void addVisionMeasurement(
            Pose2d visionPose,
            double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d newPose) {
        poseEstimator.resetPosition(
                gyroAngle,
                modulePositions,
                newPose);
    }

    @Override
    public void periodic() {
        // Log the estimated pose for debugging and visualization purposes
        Pose2d pose = poseEstimator.getEstimatedPosition();

        Logger.recordOutput("PoseEstimator/X", pose.getX());
        Logger.recordOutput("PoseEstimator/Y", pose.getY());
        Logger.recordOutput("PoseEstimator/Theta",
                pose.getRotation().getRadians()); // In radians

        Logger.recordOutput("PoseEstimator/EstimatedPose", pose);
    }
}
