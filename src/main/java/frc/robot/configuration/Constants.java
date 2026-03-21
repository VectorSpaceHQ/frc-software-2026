
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

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
    public static final double DEADBAND = 0.05;

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
    public static final ControllerEnum controllerType3 = ControllerEnum.FLIGHTSTICK;
    public static final int controllerPort3 = 2;

    // motor canid
    public enum MotorCanIDEnum {
      SWERVE_FRONT_RIGHT_X60_CANID(2),
      SWERVE_FRONT_RIGHT_X44_CANID(3),
      SWERVE_FRONT_LEFT_X60_CANID(4),
      SWERVE_FRONT_LEFT_X44_CANID(5),
      SWERVE_BACK_LEFT_X60_CANID(6),
      SWERVE_BACK_LEFT_X44_CANID(7),
      SWERVE_BACK_RIGHT_X60_CANID(8),
      SWERVE_BACK_RIGHT_X44_CANID(9),
      SWERVE_FRONT_RIGHT_CANCODER_CANID(21),
      SWERVE_FRONT_LEFT_CANCODER_CANID(22),
      SWERVE_BACK_LEFT_CANCODER_CANID(23),
      SWERVE_BACK_RIGHT_CANCODER_CANID(24),
      HOPPER_EXTENDER_CANID(10),
      INTAKE_ROLLERS_CANID(11),
      INTAKE_PIVOT_LEFT_CANID(12),
      INTAKE_PIVOT_RIGHT_CANID(13),
      WASHING_MACHINE_INDEXER_CANID(14),
      FEED_ROLLERS_CANID(15),
      SHOOTER_ENGLISH_MOTOR_CANID(17),
      SHOOTER_MAIN_MOTOR_CANID(18),
      CLIMBER_CANID(19);

      private int CanID;

      private MotorCanIDEnum(int CanID) {
        this.CanID = CanID;
      }

      public int getCanID() {
        return this.CanID;
      }
    }

    // subsystem IDs
    public enum SubSystemIDEnum {
      SHOOTER_SUBSYSTEM,
      INTAKE_SUBSYSTEM,
      INDEXER_SUBSYSTEM,
      SWERVE_SUBSYSTEM,
      HOPPER_SUBSYSTEM,
      CLIMBER_SUBSYSTEM,
      VISION_SUBSYSTEM
    }

  }

  public static final double MAX_MOTOR_VOLTS = 12.0;

  public static class SysIdEnums {
    public enum SysIdTarget {
      ENGLISH,
      MAIN,
      FEEDER,
      PIVOT,
      ROLLER
    }

  }

  public static class SwerveConstants {
    public static final double maxSpeed = Units.feetToMeters(4.5);
  }

  public static class IntakeConstants {
    public static final boolean RUNNING_SYS_ID = false;
    public static final double MAX_VOLTAGE = 12.0;

    // Roller Motor
    public static final double ROLLER_MAX_RPM = 11000.0;
    public static final double ROLLER_STARTER_RPM = -5500;
    public static final double ROLLER_GEAR_RATIO = 1.0;
    public static final int ROLLER_CURRENT_LIMIT = 20;
    public static final double ROLLER_kS = 0.25;
    public static final double ROLLER_kP = 0.01;
    public static final double ROLLER_kI = 0.0;
    public static final double ROLLER_kD = 0.0;
    public static final double ROLLER_kV = 0.019098;
    public static final double ROLLER_kA = 0.0;

    // Basically everything below here won't be used but whatever (nice to have)
    // Pivot Motor
    public static final double PIVOT_GEAR_RATIO = 60.0;
    public static final double PIVOT_MIN_ANGLE_RAD = -1.360;
    public static final double PIVOT_MAX_ANGLE_RAD = 0.05;
    public static final double PIVOT_TOLERANCE_RAD = 0.05;
    public static final int PIVOT_CURRENT_LIMIT = 20;
    public static final double PIVOT_kS = 0.0; // TODO: Find kS
    public static final double PIVOT_kG = 0.0; // TODO: Find kG
    public static final double PIVOT_kV = 0.0; // TODO: Find kV
    public static final double PIVOT_kA = 0.0; // TODO: Find kA
    public static final double PIVOT_kP = 0.0015; // TODO: Find kP
    public static final double PIVOT_kI = 0.01; // TODO: Find kI
    public static final double PIVOT_kD = 0.0; // TODO: Find kD

    public enum PivotState {
      UP(-2.5), // Volts to pivot up (fight against gravity)
      DOWN(1.5), // Volts to pivot down
      OFF(0.0);

      public final double voltage;

      PivotState(double voltage) {
        this.voltage = voltage;
      }

      public String textName() {
        return this.name();
      }
    }
  }

  public static class IndexerConstants {
    public static final double MAX_RPM = 6000;
    public static final double GEAR_RATIO = 1/25.0;
    public static final int INDEXER_CURRENT_LIMIT = 40;
    public static final double kS = 0.25;
    public static final double kP = 0.005;
    public static final double kI = 0.0005;
    public static final double kD = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double INDEXER_RPM = 2500;
  }

  public static class ShooterConstants {
    public static final Pose2d blueHubCenter = new Pose2d(4.620419, 4.034631, new Rotation2d());
    public static final Pose2d redHubCenter = new Pose2d(11.919581, 4.034631, new Rotation2d());
    // Shared Constants
    public static final double MAX_VOLTAGE = 12.0;
    public static final double SHOOTER_SPEED_TOLERANCE_RPM = 75;
    public static final boolean RUNNING_SYS_ID = false;

    // English Motor (Kraken X60)
    public static final double ENGLISH_MAX_RPM = 3000.0;
    public static final int ENGLISH_CURRENT_LIMIT = 60;
    public static final double ENGLISH_WHEEL_DIAMETER = 4; //in   
    public static final double ENGLISH_GEAR_RATIO = 0.75;
    public static final double ENGLISH_kS = 0.18;
    public static final double ENGLISH_kP = 0.3;// NOTE: need to retuen from heavier wheel 0.45;
    public static final double ENGLISH_kI = 0.5;
    public static final double ENGLISH_kD = 0.0005;
    public static final double ENGLISH_kV = 0.116;
    public static final double ENGLISH_kA = 0.0;
    public static final InvertedValue ENGLISH_INVERSION = InvertedValue.Clockwise_Positive;
    //change this value to flip motor direction

    // Main Motor (Kraken X60)
    public static final double MAIN_MAX_RPM = 4000.0;
    public static final int MAIN_CURRENT_LIMIT = 60;
    public static final double MAIN_WHEEL_DIAMETER = 6; //in    
    public static final double MAIN_GEAR_RATIO = 1;    
    public static final double MAIN_kS = 0.2;
    public static final double MAIN_kP = 0.35;// NOTE: need to retrun from heavier wheel 0.5;
    public static final double MAIN_kI = 0.6;
    public static final double MAIN_kD = 0.0005;
    public static final double MAIN_kV = 0.118;
    public static final double MAIN_kA = 0.0;
    public static final InvertedValue MAIN_INVERSION = InvertedValue.Clockwise_Positive;
    //change this value to flip motor direction

    // Feeder Motor (NEO / SparkMax)
    public static final double FEEDER_MAX_RPM = 5676.0;
    public static final int FEEDER_CURRENT_LIMIT = 40;
    public static final double FEEDER_GEAR_RATIO = 2;        
    public static final double FEEDER_kS = 0.0;
    public static final double FEEDER_kP = 0.0;
    public static final double FEEDER_kI = 0.0;
    public static final double FEEDER_kD = 0.0;
    public static final double FEEDER_kV = 0.00216;
    public static final double FEEDER_kA = 0.0;
    public static final double FEEDER_SLEW_RATE = 0.5; //seconds to go from 0 to top speed
    public static final boolean FEEDER_INVERSION = false; //set true if we want to flip positive/negative on motor
  }

  public static class DriveToTargetConstants { // Constants for profiled PID controllers

    public static final double TRANSLATION_P = 1.3;
    public static final double TRANSLATION_I = 0.0;
    public static final double TRANSLATION_D = 0.0;
    public static final double TRANSLATION_MAX_VEL = 4.0;
    public static final double TRANSLATION_MAX_ACCEL = 4.0;

    public static final double ROTATION_P = 25.0;
    public static final double ROTATION_I = 5.0;
    public static final double ROTATION_D = 0.32;
    public static final double ROTATION_MAX_VEL = 6.0;
    public static final double ROTATION_MAX_ACCEL = 8.0;

  }

  // vision constants
  public static class VisionConstants {
    // Constants for the camera name and field layout path
    public static final AprilTagFields FIELD_WELDED_2026 = AprilTagFields.k2026RebuiltWelded;
    public static final String CAMERA_NAME = "Front_Camera_Robot2";

    // Strategy for processing multiple AprilTags on the coprocessor
    public static final PhotonPoseEstimator.PoseStrategy MULTI_TAG_PNP_ON_COPROCESSOR = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    // Constants for the maximum pose age and ambiguity
    public static final double MAX_POSE_AGE = 0.5;
    public static final double MAX_AMBIGUITY = 0.35;

    // Constants for the Transformation3d objects for the camera and robot
    public static final double TRANSLATION_X = -0.1778; // Meters forward from the robot center
    public static final double TRANSLATION_Y = 0.3302; // Meters to the left from the robot center
    public static final double TRANSLATION_Z = 0.632; // Meters above the robot center

    public static final double ROTATION_X = Math.toRadians(0); // 90 degree rotation around the X-axis CCW
    public static final double ROTATION_Y = Math.toRadians(15); // Rotate 20 degrees cw
    public static final double ROTATION_Z = Math.toRadians(90); // 90 degree rotation around the Z-axis CW
    public static final Vector<N3> VISION_ST_DEVS = VecBuilder.fill(0.7, 0.7, Math.toRadians(15)); // Meters, Meters,
                                                                                                   // Degrees (to
                                                                                                   // radians)

    // Constants for the distance from the camera to the robot
    public static final Transform3d robotToCamera = new Transform3d( // Actually robotToCamera...
        new Translation3d(TRANSLATION_X, TRANSLATION_Y, TRANSLATION_Z),
        new Rotation3d(ROTATION_X, ROTATION_Y, ROTATION_Z));

    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-apriltag-images-user-guide.pdf
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // Lefts and rights are relative to the appropriate alliance side
    // driverstations. Consistent with 2026 rebuilt fields.

    // For reference (and potentially commands)
    // Note: Location is not only relative to the alliance, but the location of
    // AprilTags on the hub are with *respect to their current side*
    // Note: Right side of
    public enum Apriltags {
      // In order: Alliance color, element section(s), area zone (ex: 2 is on the red
      // alliance, on the hub, in the center of the hub, closer to the neutral zone)
      None(-1),

      // Red Alliance
      RedTrenchRightSideInNeutralZone(1), // Right side of Trench, Neutral Zone
      RedHubRightSideBumpInNeutralZone(2), // Right side of Hub, Bump side, Neutral Zone
      RedHubRightSideCenterInNeutralZone(3), // Right side of Hub, Center, Neutral Zone
      RedHubLeftSideCenterInNeutralZone(4), // Left side of Hub, Center, Neutral Zone
      RedHubLeftSideBumpInNeutralZone(5), // Left side of Hub, Bump side, Neutral Zone
      RedTrenchLeftSideInNeutralZone(6), // Left side of Trench, Neutral Zone
      RedTrenchLeftSideInAllianceZone(7), // Left side of Trench, Alliance Zone
      RedHubLeftSideBumpInAllianceZone(8), // Left side of Hub, Bump side, Alliance Zone
      RedHubLeftSideCenterInAllianceZone(9), // Left side of Hub, Center, Alliance Zone
      RedHubRightSideCenterInAllianceZone(10), // Right side of Hub, Center, Alliance Zone
      RedHubRightSideBumpInAllianceZone(11), // Right side of Hub, Bump side, Alliance Zone
      RedTrenchRightSideInAllianceZone(12), // Right side of Trench, Alliance Zone
      RedOutpostRightSideInAllianceZone(13), // Right side of Outpost, Alliance Zone
      RedOutpostLeftSideInAllianceZone(14), // Left side of Outpost, Alliance Zone
      RedTowerLeftSideInAllianceZone(15), // Left side of Tower, Alliance Zone
      RedTowerRightSideInAllianceZone(16), // Right side of Tower, Alliance Zone

      // Blue Alliance
      BlueTrenchRightSideInNeutralZone(17), // Right side of Trench, Neutral Zone
      BlueHubRightSideBumpInNeutralZone(18), // Right side of Hub, Bump side, Neutral Zone
      BlueHubRightSideCenterInNeutralZone(19), // Right side of Hub, Center, Neutral Zone
      BlueHubLeftSideCenterInNeutralZone(20), // Left side of Hub, Center, Neutral Zone
      BlueHubLeftSideBumpInNeutralZone(21), // Left side of Hub, Bump side, Neutral Zone
      BlueTrenchLeftSideInNeutralZone(22), // Left side of Trench, Neutral Zone
      BlueTrenchLeftSideInAllianceZone(23), // Left side of Trench, Alliance Zone
      BlueHubLeftSideBumpInAllianceZone(24), // Left side of Hub, Bump side, Alliance Zone
      BlueHubLeftSideCenterInAllianceZone(25), // Left side of Hub, Center, Alliance Zone
      BlueHubRightSideCenterInAllianceZone(26), // Right side of Hub, Center, Alliance Zone
      BlueHubRightSideBumpInAllianceZone(27), // Right side of Hub, Bump side, Alliance Zone
      BlueTrenchRightSideInAllianceZone(28), // Right side of Trench, Alliance Zone
      BlueOutpostRightSideInAllianceZone(29), // Right side of Outpost, Alliance Zone
      BlueOutpostLeftSideInAllianceZone(30), // Left side of Outpost, Alliance Zone
      BlueTowerLeftSideInAllianceZone(31), // Left side of Tower, Alliance Zone
      BlueTowerRightSideInAllianceZone(32); // Right side of Tower, Alliance Zone

      private int value;

      private Apriltags(int id) {
        this.value = id;
      }

      public int getId() {
        return this.value;

      }

      public Pose2d getPose(AprilTagFieldLayout layout) {
        return layout.getTagPose(getId())
            .map(p -> p.toPose2d())
            .orElseThrow(() -> new RuntimeException("Tag not found in layout: " + getId()));
      }
    }
    public static final Translation3d HUB_TARGET_OFFSET = new Translation3d(0.0, 0.0, 0.7); // Can be tuned, roughly 0.7 meters above Apriltag center
    public static final Pose2d goalPosition = new Pose2d(4.620419, 4.034631, new Rotation2d());
  }

}
