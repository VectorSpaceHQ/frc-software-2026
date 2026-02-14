// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
    
    //controller interface constants
    public enum ControllerEnum{
      XBOX,
      FLIGHTSTICK,
      PS4,
      PS5,
      Auto
    }
    //change this value to change what controller type is used by ControllerSubsystem.java, enums may be a better approach.
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
  public static class SwerveConstants {
    public static final double maxSpeed = Units.feetToMeters(4.5);
  }
  public static final double MAX_MOTOR_VOLTS = 12.0;
}
