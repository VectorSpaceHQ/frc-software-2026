package frc.robot.components.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {
  public class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d rotation2d = new Rotation2d(); // Yaw position
    public Rotation3d rotation3d = new Rotation3d();
    public double yawVelocityRadPerSec = 0.0;
  }

  // public double getRate(); Add getVelocityZWorld() from core pigeon2 instead?
  // Idk
  
  public default void reset() {
  }

  public default void DisplayIMUData() {
  }

  public default void updateInputs(GyroIOInputs inputs) {
  }
}
