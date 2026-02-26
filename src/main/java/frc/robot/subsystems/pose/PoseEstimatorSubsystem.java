package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.configuration.Constants.VisionConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {

    private final VisionSubsystem visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    public PoseEstimatorSubsystem(
            VisionSubsystem visionSubsystem,
            SwerveSubsystem swerveSubsystem,
            Pose2d initialPose) {

        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        swerveSubsystem.getSwerveDrive().resetOdometry(initialPose);
        swerveSubsystem.getSwerveDrive().setVisionMeasurementStdDevs(VisionConstants.VISION_ST_DEVS);

    }

    public void update() {
        var visionMeasurement = visionSubsystem.getLatestVisionMeasurement();

        visionMeasurement.ifPresent(measurement -> { // If there is a present vision measurement (estimated robot pose based on vision)
            swerveSubsystem.getSwerveDrive().addVisionMeasurement(
                    measurement.estimatedPose.toPose2d(),
                    measurement.timestampSeconds);
            Logger.recordOutput("PoseEstimator/RawVisionPose",
                    measurement.estimatedPose.toPose2d());
            Logger.recordOutput("PoseEstimator/VisionTimestamp",
                    measurement.timestampSeconds);
        });

    }

    public void resetPose(Pose2d newPose) {
        swerveSubsystem.getSwerveDrive().resetOdometry(newPose);
    }

    public Pose2d getEstimatedPose() {
        return swerveSubsystem.getSwerveDrive().getPose();
    }
    
    // Unused
    public void resetPose() {
        swerveSubsystem.resetOdometry();

    }

    @Override
    public void periodic() {
        update();
        Pose2d pose = getEstimatedPose();
        Supplier<Pose2d> currentPose = () -> pose;
        Logger.recordOutput("PoseEstimator/EstimatedPose", pose); // For AdvantageScope
        Logger.recordOutput("PoseEstimator/X", pose.getX());
        Logger.recordOutput("PoseEstimator/Y", pose.getY());
        Logger.recordOutput("PoseEstimator/Theta", pose.getRotation().getRadians());

    }
}