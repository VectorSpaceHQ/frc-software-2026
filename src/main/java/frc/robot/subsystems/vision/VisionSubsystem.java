package frc.robot.subsystems.vision;

import static frc.robot.configuration.Constants.VisionConstants;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.ArrayList;
import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;

public class VisionSubsystem extends SubsystemBase {
    
    
    private PhotonCamera camera;
    private AprilTagFieldLayout layout;
    private PhotonPoseEstimator poseEstimator;

    // A hashmap that stores key (id) and value (target/range) pairs. Used to check
    // if the last valid value is valid so that the driveTargetCommand works every
    // cycle
    private Map<Integer, Double> lastValidYaw = new HashMap<>();
    private Map<Integer, Double> lastValidRange = new HashMap<>();

    

    // Distance from the robot to the camera
    private Transform3d robotToCamera = VisionConstants.cameraToRobot.inverse();

    // Status of the camera
    private boolean cameraConnected;

    // Stored estimated pose
    private Optional<EstimatedRobotPose> storedEstimatedPose = Optional.empty();

    // Getting All Unread Results
    private List<PhotonPipelineResult> allUnreadResults = new ArrayList<>();

    // Shuffleboard entries
    private ShuffleboardTab visionTab;
    private ShuffleboardLayout anglesCol;
    private ShuffleboardLayout tagCol;
    private ShuffleboardLayout transformCol;
    private ShuffleboardLayout targetCalcCol;
    private ShuffleboardLayout poseCol;

    private GenericEntry yawEntry;
    private GenericEntry pitchEntry;
    private GenericEntry skewEntry;
    private GenericEntry idEntry;
    private GenericEntry areaEntry;
    private GenericEntry ambiguityEntry;
    private GenericEntry xEntry;
    private GenericEntry yEntry;
    private GenericEntry zEntry;
    private GenericEntry rawTargetYawEntry;
    private GenericEntry yawStatusEntry;
    private GenericEntry rawTargetRangeEntry;
    private GenericEntry rangeStatusEntry;
    private GenericEntry visionPoseXEntry;
    private GenericEntry visionPoseYEntry;
    private GenericEntry visionPoseHeadingEntry;
    private GenericEntry hasValidPoseEntry;
    private GenericEntry poseTimestampEntry;

    // Vision Subsystem constructor
    public VisionSubsystem() {
        // Initialize camera with name matching PhotonVision GUI (HAS TO MATCH)
        camera = new PhotonCamera(VisionConstants.camera_name);

        initializeSubsystem();
        initializeShuffleboard();

        try {
            initializeAprilTagFieldLayout();
        } catch (IOException e) {
            System.err.println("Error initializing AprilTag field layout: " + e.getMessage());
            cameraConnected = false;
        }
    }

    private void initializeSubsystem() {
        try {
            cameraConnected = camera.isConnected(); // True if camera is connected

            if (!cameraConnected) {
                System.err.println("Warning: Camera not connected.");
                return;
            }
            System.out.println("Camera connected. Vision Subsystem initialized.");

        } catch (Exception e) {
            System.err.println("Error initializing camera: " + e.getMessage());
            cameraConnected = false;
        }
    }

    // Method to check if the camera is connected
    public boolean isCameraConnected() {
        return cameraConnected;
    }

    // Initializes the AprilTag field layout from the JSON file containing the 2026
    // layout
    private void initializeAprilTagFieldLayout() throws IOException {
        layout = AprilTagFieldLayout.loadField(VisionConstants.field_welded_2026);

        // Origin Point
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            layout.setOrigin(alliance.get() == Alliance.Red
                    ? OriginPosition.kRedAllianceWallRightSide
                    : OriginPosition.kBlueAllianceWallRightSide);
        }

        // Initialize pose estimator
        poseEstimator = new PhotonPoseEstimator(layout, VisionConstants.MULTI_TAG_PNP_ON_PROCESSOR, robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    }

    // Initializing the shuffleboard entries
    private void initializeShuffleboard() {
        visionTab = Shuffleboard.getTab("Vision Results");

        // Define column layouts across the tab
        anglesCol = visionTab.getLayout("Angles", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 6);
        tagCol = visionTab.getLayout("Tag", BuiltInLayouts.kList).withPosition(1, 0).withSize(1, 6);
        transformCol = visionTab.getLayout("Transform", BuiltInLayouts.kList).withPosition(2, 0).withSize(1, 6);
        targetCalcCol = visionTab.getLayout("Target Calc", BuiltInLayouts.kList).withPosition(3, 0).withSize(1, 6);
        poseCol = visionTab.getLayout("Pose", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 6);

        // Populate entries within the layouts
        yawEntry = anglesCol.add("Yaw", 0.0).getEntry();
        pitchEntry = anglesCol.add("Pitch", 0.0).getEntry();
        areaEntry = anglesCol.add("Area", 0.0).getEntry();
        skewEntry = anglesCol.add("Skew", 0.0).getEntry();

        idEntry = tagCol.add("ID", 0.0).getEntry();
        ambiguityEntry = tagCol.add("Ambiguity", 0.0).getEntry();

        xEntry = transformCol.add("X", 0.0).getEntry();
        yEntry = transformCol.add("Y", 0.0).getEntry();
        zEntry = transformCol.add("Z", 0.0).getEntry();

        rawTargetYawEntry = targetCalcCol.add("Raw Target Yaw", 0.0).getEntry();
        yawStatusEntry = targetCalcCol.add("Yaw Status", "No Data").getEntry();
        rawTargetRangeEntry = targetCalcCol.add("Raw Target Range", 0.0).getEntry();
        rangeStatusEntry = targetCalcCol.add("Range Status", "No Data").getEntry();

        visionPoseXEntry = poseCol.add("Vision Pose X", 0.0).getEntry();
        visionPoseYEntry = poseCol.add("Vision Pose Y", 0.0).getEntry();
        visionPoseHeadingEntry = poseCol.add("Vision Pose Heading", 0.0).getEntry();
        hasValidPoseEntry = poseCol.add("Has Valid Pose", false).getEntry();
        poseTimestampEntry = poseCol.add("Pose Timestamp", 0.0).getEntry();
    }

    // Gets the target yaw
    public double getTargetYaw(int id) {
        double yawValue = Double.NaN; // Default to NaN
        yawStatusEntry.setString("No Data");

        var tagPose = layout.getTagPose(id);
        if (tagPose.isPresent()) {
            Pose2d tagPose2d = tagPose.get().toPose2d();
            Optional<Pose2d> robotPose = getRobotPose();

            if (robotPose.isPresent()) {
                Rotation2d returnYaw = PhotonUtils.getYawToPose(robotPose.get(), tagPose2d);
                yawValue = returnYaw.getDegrees();
                rawTargetYawEntry.setDouble(yawValue);
                yawStatusEntry.setString("Current Measurement");

                // Store valid measurement
                if (!Double.isNaN(yawValue)) {
                    lastValidYaw.put(id, yawValue);

                }
            }
        }

        // If current calculation is NaN but we have a previous value, use that
        if (Double.isNaN(yawValue) && lastValidYaw.containsKey(id)) {
            yawValue = lastValidYaw.get(id);
            yawStatusEntry.setString("Using Last Valid");
        }
        return yawValue;
    }

    // for the DriveTargetCommand but still kinda useless
    public boolean isTargetVisible(int id) {
        boolean targetIsVisible = false;

        if (cameraConnected && !allUnreadResults.isEmpty()) {
            var latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

            if (latestResult.hasTargets()) {
                for (var target : latestResult.getTargets()) {
                    if (target.getFiducialId() == id) {
                        targetIsVisible = true;
                        break;
                    }
                }
            }
        }

        return targetIsVisible;
    }

    // Gets the target range
    public double getTargetRange(int id) {

        double rangeValue = Double.NaN; // Default to NaN
        rangeStatusEntry.setString("No Data");
        var tagPose = layout.getTagPose(id);

        if (tagPose.isPresent()) {
            Optional<Pose2d> robotPose = getRobotPose();

            if (robotPose.isPresent()) {
                rangeValue = PhotonUtils.getDistanceToPose(
                        robotPose.get(),
                        tagPose.get().toPose2d());
                rawTargetRangeEntry.setDouble(rangeValue);
                rangeStatusEntry.setString("Current Measurement");
            }
            // Store valid measurement
            if (!Double.isNaN(rangeValue)) {
                lastValidRange.put(id, rangeValue);
            }
        }

        // If current calculation is NaN but we have a previous value, use that
        if (Double.isNaN(rangeValue) && lastValidRange.containsKey(id)) {
            rangeValue = lastValidRange.get(id);
            rangeStatusEntry.setString("Using Last Valid");
        }
        return rangeValue;
    }

    // Updates the vision pose using the pose estimator (periodic)
    private void updateVisionPoseEstimate() {

        // Start from most recent result
        for (int resultsIndex = allUnreadResults.size() - 1; resultsIndex >= 0; resultsIndex--) {
            var result = allUnreadResults.get(resultsIndex);

            if (result.hasTargets()) {
                boolean hasValidTags = false;
                for (var target : result.getTargets()) {

                    // Target has to have a certain ambiguity and a valid tag pose
                    if (layout.getTagPose(target.getFiducialId()).isPresent() &&
                            target.getPoseAmbiguity() < VisionConstants.maxAmbiguity) {
                        hasValidTags = true;
                        break;
                    }
                }

                if (hasValidTags) {
                    Optional<EstimatedRobotPose> estimatedVisionPose = poseEstimator.update(result);

                    if (estimatedVisionPose.isPresent()) {
                        // Stores vision pose and logs
                        storedEstimatedPose = estimatedVisionPose;
                        Pose2d pose = estimatedVisionPose.get().estimatedPose.toPose2d();
                        visionPoseXEntry.setDouble(pose.getX());
                        visionPoseYEntry.setDouble(pose.getY());
                        visionPoseHeadingEntry.setDouble(pose.getRotation().getDegrees());
                    }
                    break;
                }
            }
        }

    }

    // Gets the tag pose
    public Optional<Pose3d> getTagPose(int id) {
        if (layout != null) {
            return layout.getTagPose(id);
        }
        return Optional.empty();
    }

    // Gets the april tag ID by name (to be implemented)
    public Optional<Pose3d> getPoseByName(VisionConstants.Apriltags tagName) {
        // Use the official getTagPose(int) method
        return layout.getTagPose(tagName.getId());
    }

    // Used to ensure that the vision pose is not stale
    public boolean isFreshPose() {
        if (!storedEstimatedPose.isPresent()) {
            return false;
        }

        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double poseTime = storedEstimatedPose.get().timestampSeconds;
    return (currentTime - poseTime) < VisionConstants.maxPoseAge;
    }

    // Converts 3d vision pose to 2d vision pose and gets it
    public Optional<Pose2d> getRobotPose() {
        if (storedEstimatedPose.isPresent()) {
            return Optional.of(storedEstimatedPose.get().estimatedPose.toPose2d());
        }
        return Optional.empty();
    }

    // Method to get the timestamp of the latest vision pose
    public double getTimestamp() {

        double timestamp = -1;

        if (storedEstimatedPose.isPresent()) {
            EstimatedRobotPose estimatedRobotPose = storedEstimatedPose.get();
            timestamp = estimatedRobotPose.timestampSeconds;
        }

        return timestamp;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        hasValidPoseEntry.setBoolean(storedEstimatedPose.isPresent());
        poseTimestampEntry.setDouble(getTimestamp());
        if (cameraConnected) {
            allUnreadResults = camera.getAllUnreadResults();
            // Update the robot pose estimate using the estimator
            updateVisionPoseEstimate();

            if (!allUnreadResults.isEmpty() && allUnreadResults.get(allUnreadResults.size() - 1).hasTargets()) {
                var latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

                for (PhotonTrackedTarget target : latestResult.getTargets()) {

                    double yaw = target.getYaw();
                    double pitch = target.getPitch();
                    double area = target.getArea();
                    double skew = target.getSkew();
                    double id = target.getFiducialId();
                    double ambiguity = target.getPoseAmbiguity();
                    Transform3d transform = target.getBestCameraToTarget();
                    double x = transform.getTranslation().getX();
                    double y = transform.getTranslation().getY();
                    double z = transform.getTranslation().getZ();

                    yawEntry.setDouble(yaw);
                    pitchEntry.setDouble(pitch);
                    areaEntry.setDouble(area);
                    skewEntry.setDouble(skew);
                    idEntry.setDouble(id);
                    ambiguityEntry.setDouble(ambiguity);
                    xEntry.setDouble(x);
                    yEntry.setDouble(y);
                    zEntry.setDouble(z);

                    hasValidPoseEntry.setBoolean(storedEstimatedPose.isPresent());
                    poseTimestampEntry.setDouble(getTimestamp());

                }
            }
        }
    }
}