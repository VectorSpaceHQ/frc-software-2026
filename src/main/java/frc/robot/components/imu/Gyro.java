package frc.robot.components.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;

public class Gyro implements GyroIO {
  // The gyro sensor
  private Pigeon2 m_gyro = null;

  public Gyro() {
    m_gyro = new Pigeon2(0, "rio");
  }


  @Override
  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  @Override
  public Rotation3d getRotation3d() {
    return m_gyro.getRotation3d();
  }

 

  @Override
  public void reset() {
    m_gyro.setYaw(0);
  }

  @Override
  public void DisplayIMUData() { // Lazy
    System.out.println("Gyro Rotation2d: " + getRotation2d());
    System.out.println("Gyro Rotation3d: " + getRotation3d());
    System.out.println("Gyro Yaw: " + m_gyro.getYaw());
    
  }
}