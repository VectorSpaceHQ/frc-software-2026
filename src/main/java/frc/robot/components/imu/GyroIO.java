package frc.robot.components.imu;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {
  public Rotation2d getRotation2d();
  public Rotation3d getRotation3d();
  // public double getRate();
  public void reset();
  public void DisplayIMUData();
}