package frc.robot.subsystems.vision;

import static frc.robot.configuration.Constants.VisionConstants;

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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    // PhotonVision camera
    private PhotonCamera camera;

    // AprilTag layout for the 2026 field
    private AprilTagFieldLayout layout;

    // Pose estimator used ONLY to generate vision measurements
    private PhotonPoseEstimator poseEstimator;

    // Status of the camera
    private boolean cameraConnected;

    // Stores most recent valid vision measurement
    private Optional<EstimatedRobotPose> latestVisionMeasurement = Optional.empty();

    // Getting All Unread Results
    private List<PhotonPipelineResult> allUnreadResults = new ArrayList<>();

    // Distance from the robot to the camera
    private Transform3d robotToCamera = VisionConstants.cameraToRobot.inverse();

    // Vision Subsystem constructor
    public VisionSubsystem() {

        // Initialize camera with name matching PhotonVision GUI (HAS TO MATCH)
        camera = new PhotonCamera(VisionConstants.camera_name);

        initializeSubsystem();

        try {
            initializeAprilTagFieldLayout();
        } catch (IOException e) {
            cameraConnected = false;
        }
    }

    private void initializeSubsystem() {

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

    // Initializes the AprilTag field layout from the JSON file containing the 2026 layout
    private void initializeAprilTagFieldLayout() throws IOException {

        layout = AprilTagFieldLayout.loadField(VisionConstants.field_welded_2026);

        // Origin Point
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            layout.setOrigin(alliance.get() == Alliance.Red
                    ? OriginPosition.kRedAllianceWallRightSide
                    : OriginPosition.kBlueAllianceWallRightSide);
        }

        // Initialize pose estimator (ONLY for generating measurements)
        poseEstimator = new PhotonPoseEstimator(
                layout,
                VisionConstants.MULTI_TAG_PNP_ON_PROCESSOR,
                robotToCamera);

        poseEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    // Returns the latest valid vision pose measurement
    public Optional<EstimatedRobotPose> getLatestVisionMeasurement() {
        return latestVisionMeasurement;
    }

    // Gets the target yaw directly from camera measurement
    public double getTargetYaw(int id) {

        if (allUnreadResults.isEmpty()) return Double.NaN;

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

        if (allUnreadResults.isEmpty()) return Double.NaN;

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

        latestVisionMeasurement = Optional.empty();

        for (int resultsIndex = allUnreadResults.size() - 1; resultsIndex >= 0; resultsIndex--) {

            var result = allUnreadResults.get(resultsIndex);

            if (!result.hasTargets()) continue;

            // Target must have acceptable ambiguity
            boolean hasValidTags = result.getTargets().stream()
                    .anyMatch(t -> t.getPoseAmbiguity() < VisionConstants.maxAmbiguity);

            if (!hasValidTags) continue;

            Optional<EstimatedRobotPose> estimatedVisionPose =
                    poseEstimator.update(result);

            if (estimatedVisionPose.isPresent()) {
                latestVisionMeasurement = estimatedVisionPose;
                break;
            }
        }
    }

    @Override
    public void periodic() {

        if (!cameraConnected) return;

        allUnreadResults = camera.getAllUnreadResults();

        if (!allUnreadResults.isEmpty()) {
            updateVisionMeasurement();
        } else {
            latestVisionMeasurement = Optional.empty();
        }
    }
}
