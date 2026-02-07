// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    
  }
  public static final double MAX_MOTOR_VOLTS = 12.0;
}
