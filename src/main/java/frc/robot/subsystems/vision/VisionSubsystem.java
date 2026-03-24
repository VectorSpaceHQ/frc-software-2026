package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.configuration.Constants.VisionConstants;
import frc.robot.configuration.Constants.VisionConstants.Apriltags;
import frc.robot.configuration.configs.VisionSubsysConfig;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

    // Config file for vision
    private VisionSubsysConfig visionConfig = null;
    // PhotonVision camera
    private PhotonCamera camera;

    // AprilTag layout for the 2026 field
    private AprilTagFieldLayout layout;

    // Pose estimator used ONLY to generate vision measurements

    private PhotonPoseEstimator poseEstimator;

    private double lastVisionTimestamp = -1;

    // Status of the camera
    private boolean cameraConnected;

    // Stores most recent valid vision measurement
    private Optional<EstimatedRobotPose> latestVisionMeasurement = Optional.empty();
    // Stores best visible tag from accepted frame
    private Optional<Apriltags> bestVisibleTag = Optional.empty();

    // Getting All Unread Results
    private List<PhotonPipelineResult> allUnreadResults = new ArrayList<>();

    // Distance from the robot to the camera
    private Transform3d robotToCamera = VisionConstants.robotToCamera; // Removed Inverse

    // Vision Subsystem constructor
    public VisionSubsystem(VisionSubsysConfig config) {
        this.visionConfig = config;
        if (visionConfig.getIsPresent()) {

            // Initialize camera with name matching PhotonVision GUI (HAS TO MATCH)
            camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

            updateCameraStatus();

            try {
                initializeAprilTagFieldLayout();
            } catch (IOException e) {
                cameraConnected = false;
                System.err.println("Warning: Camera not connected." + e.getMessage());
            }
        }       

        SmartDashboard.putBoolean("Vision Present", visionConfig.getIsPresent());
        SmartDashboard.putBoolean("Camera Present", cameraConnected);
    }

    public void updateCameraStatus() {

        cameraConnected = camera.isConnected(); // True if camera is connected

        if (!cameraConnected) {
            System.err.println("Warning: Camera not connected.");
            return;
        }

        System.out.println("Camera connected. Vision Subsystem initialized.");
    }

    // Method to check if the camera is connected
    public boolean isCameraConnected() {
        return cameraConnected;
    }

    // Initializes the AprilTag field layout from the JSON file containing the 2026
    // layout
    private void initializeAprilTagFieldLayout() throws IOException {

        layout = AprilTagFieldLayout.loadField(VisionConstants.FIELD_WELDED_2026);

        // Initialize pose estimator (ONLY for generating measurements)
        poseEstimator = new PhotonPoseEstimator(
                layout,
                VisionConstants.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);

        poseEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    // Gets the target yaw directly from camera measurement
    public double getTargetYaw(int id) {

        if (allUnreadResults.isEmpty())
            return Double.NaN;

        var latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

        for (PhotonTrackedTarget target : latestResult.getTargets()) {
            if (target.getFiducialId() == id) {
                return target.getYaw();
            }
        }

        return Double.NaN;
    }

    // Gets the straight-line range from camera to tag
    public double getTargetRange(int id) {

        if (allUnreadResults.isEmpty())
            return Double.NaN;

        var latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

        for (PhotonTrackedTarget target : latestResult.getTargets()) {
            if (target.getFiducialId() == id) {
                return target.getBestCameraToTarget()
                        .getTranslation()
                        .getNorm();
            }
        }

        return Double.NaN;
    }

    // Updates the vision measurement (periodic)

    private void updateVisionMeasurement() {

        for (int resultsIndex = allUnreadResults.size() - 1; resultsIndex >= 0; resultsIndex--) {

            var result = allUnreadResults.get(resultsIndex);
            SmartDashboard.putBoolean("Result Has Targets", result.hasTargets());
            if (!result.hasTargets())
                continue;

            // Reject duplicate timestamps
            double timestamp = result.getTimestampSeconds();
            SmartDashboard.putNumber("Timestamp", timestamp);
            if (timestamp <= lastVisionTimestamp) {
                SmartDashboard.putString("Timestamp", "Too Old");
                continue;

            }

            // For stale poses
            double poseAge = Timer.getFPGATimestamp() - timestamp;
            SmartDashboard.putNumber("Pose Age", poseAge);
            if (poseAge > VisionConstants.MAX_POSE_AGE) {
                SmartDashboard.putString("Pose Age", "Too Old");
                continue;
            }

            // Select the lowest ambiguity valid target (not boolean anymore)
            Optional<PhotonTrackedTarget> validTarget = result.getTargets().stream()
                    .filter(t -> t.getPoseAmbiguity() < VisionConstants.MAX_AMBIGUITY)
                    .min((a, b) -> Double.compare(a.getPoseAmbiguity(), b.getPoseAmbiguity()));
            SmartDashboard.putBoolean("Result Has Valid Targets", validTarget.isPresent());

            if (validTarget.isEmpty()) {
                continue;
            }

            Optional<EstimatedRobotPose> estimatedVisionPose = poseEstimator.update(result);
            SmartDashboard.putBoolean("Estimated Vision Pose", estimatedVisionPose.isPresent());

            if (estimatedVisionPose.isEmpty()) {
                continue;
            }

            // Accept measurement
            latestVisionMeasurement = estimatedVisionPose;

            if (latestVisionMeasurement.isPresent()) {
                SmartDashboard.putString("Vision Measurement", "Present");
            } else {
                SmartDashboard.putString("Vision Measurement", "Absent");
            }

            lastVisionTimestamp = timestamp;

            int id = validTarget.get().getFiducialId();
            for (Apriltags tag : Apriltags.values()) {
                if (tag.getId() == id) {
                    bestVisibleTag = Optional.of(tag);
                    break;
                }
            }

            break;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Camera Present", cameraConnected);
        if(!cameraConnected){
            updateCameraStatus(); //trigger wasn't working
        }
        if (cameraConnected) {
            allUnreadResults = camera.getAllUnreadResults();
            SmartDashboard.putBoolean("Vision measurement empty", allUnreadResults.isEmpty());

            if (!allUnreadResults.isEmpty()) {
                updateVisionMeasurement();
            }
        }
    }

    public AprilTagFieldLayout getLayout() {
        return layout;
    }

    // Returns the latest valid vision pose measurement
    public Optional<EstimatedRobotPose> getLatestVisionMeasurement() {
        return latestVisionMeasurement;
    }

    public Optional<Apriltags> getBestVisibleTag() {
        return bestVisibleTag;
    }

    public Trigger onCameraConnected = new Trigger(() -> camera.isConnected());
}
