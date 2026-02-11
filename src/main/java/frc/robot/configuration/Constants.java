
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;

        // controller interface constants
        public enum ControllerEnum {
            XBOX,
            FLIGHTSTICK,
            PS4,
            PS5,
            Auto
        }

        // change this value to change what controller type is used by
        // ControllerSubsystem.java, enums may be a better approach.
        public static final ControllerEnum controllerType1 = ControllerEnum.FLIGHTSTICK;
        public static final int controllerPort1 = 0;
        public static final ControllerEnum controllerType2 = ControllerEnum.FLIGHTSTICK;
        public static final int controllerPort2 = 1;

    
    // motor canid
    public enum MotorCanIDEnum{
      SWERVE_FRONT_LEFT_X60_CANID(2),
      SWERVE_FRONT_LEFT_X44_CANID(3),
      SWERVE_FRONT_RIGHT_X60_CANID(4),
      SWERVE_FRONT_RIGHT_X44_CANID(5),
      SWERVE_BACK_LEFT_X60_CANID(6),
      SWERVE_BACK_LEFT_X44_CANID(7),
      SWERVE_BACK_RIGHT_X60_CANID(8),
      SWERVE_BACK_RIGHT_X44_CANID(9),
      HOPPER_EXTENDER_CANID(10),
      INTAKE_ROLLERS_1_CANID(11),
      INTAKE_ROLLERS_2_CANID(12),
      INTAKE_PIVOT_CANID(13),
      WASHING_MACHINE_INDEXER_CANID(14),
      FEED_ROLLERS_CANID(15),
      FIRING_ROLLERS_CANID(16),
      SHOOTER_TOP_MOTOR_CANID(17),
      SHOOTER_BOTTOM_MOTOR_CANID(18),
      CLIMBER_CANID(19),
      PROTO_SHOOTER_TOP_MOTOR_CANID(19),
      PROTO_SHOOTER_BOTTOM_MOTOR_CANID(20);


      private int CanID;

      private MotorCanIDEnum(int CanID){
        this.CanID = CanID;
      }

      public int getCanID(){
        return this.CanID;
      }
    }
    //subsystem IDs
    public enum SubSystemIDEnum{
      SHOOTER_SUBSYSTEM,
      INTAKE_SUBSYSTEM,
      SWERVE_SUBSYSTEM,
      HOPPER_SUBSYSTEM,
      CLIMBER_SUBSYSTEM,
      VISION_SUBSYSTEM
    }
  }
    
    public static final double MAX_MOTOR_VOLTS = 12.0;

    // vision constants
    public static class VisionConstants {
        // Constants for the camera name and field layout path
        public static final AprilTagFields field_welded_2026 = AprilTagFields.k2026RebuiltWelded;
        public static final String camera_name = "Team10257_Front_Camera";

        // Strategy for processing multiple AprilTags on the coprocessor
        public static final PhotonPoseEstimator.PoseStrategy MULTI_TAG_PNP_ON_PROCESSOR = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        // Constants for the maximum pose age and ambiguity
        public static final double maxPoseAge = 0.5;
        public static final double maxAmbiguity = 0.2;

        // Constants for the Transformation3d objects for the camera and robot
        public static final double TranslationX = 0.1; // Meters forward from the robot center
        public static final double TranslationY = 0.2; // Meters to the left from the robot center
        public static final double TranslationZ = 0.1; // Meters above the robot center

        public static final double RotationX = 0.0; // No rotation around the X-axis
        public static final double RotationY = Math.toRadians(-20); // Rotate 20 degrees downward
        public static final double RotationZ = 0.0; // No rotation around the Z-axis

        // Constants for the distance from the camera to the robot
        public static final Transform3d cameraToRobot = new Transform3d( // Will have to change before next deploy
                new Translation3d(TranslationX, TranslationY, TranslationZ),
                new Rotation3d(RotationX, RotationY, RotationZ));

        // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-apriltag-images-user-guide.pdf
        // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
        // Lefts and rights are relative to the appropriate alliance side
        // driverstations. Consistent with 2026 rebuilt fields.

        public enum Apriltags {
            None(0.0),
            RedTrenchRightNeutralSide(1.0),
            RedHubRightBumpNeutralSide(2.0), // Right
            RedHubRightNeutralSide(3.0), // Front
            RedHubLeftNeutralSide(4.0), // Front
            RedHubLeftBumpNeutralSide(5.0), // Left
            RedTrenchLeftNeutralSide(6.0), // Left
            RedTrenchLeftAllianceSide(7.0), // Left
            RedHubLeftBumpAllianceSide(8.0), // Left
            RedHubLeftAllianceSide(9.0), // Back
            RedHubRightAllianceSide(10.0), // Back
            RedHubRightBumpAllianceSide(11.0), // Right
            RedTrenchRightAllianceSide(12.0), // Right
            RedOutpostRightSide(13.0), // Closer to wall (alliance side)
            RedOutpostLeftSide(14.0), // Closer to center (alliance side)
            RedTowerLeftSide(15.0), // Left (alliance side)
            RedTowerRightSide(16.0), // Right (alliance side)

            BlueTrenchRightNeutralSide(17.0),
            BlueHubRightBumpNeutralSide(18.0), // Right
            BlueHubRightNeutralSide(19.0), // Front
            BlueHubLeftNeutralSide(20.0), // Front
            BlueHubLeftBumpNeutralSide(21.0), // Left
            BlueTrenchLeftNeutralSide(22.0), // Left
            BlueTrenchLeftAllianceSide(23.0), // Left
            BlueHubLeftBumpAllianceSide(24.0), // Left
            BlueHubLeftAllianceSide(25.0), // Back
            BlueHubRightAllianceSide(26.0), // Back
            BlueHubRightBumpAllianceSide(27.0), // Right
            BlueTrenchRightAllianceSide(28.0), // Right
            BlueOutpostRightSide(29.0), // Closer to wall (alliance side)
            BlueOutpostLeftSide(30.0), // Closer to center (alliance side)
            BlueTowerLeftSide(31.0), // Left (alliance side)
            BlueTowerRightSide(32.0);
            
            private int value;

            private Apriltags(double id) {
                this.value = (int) id;
            }
            public int getId() {
                return this.value;
            }
        }
    }

}
