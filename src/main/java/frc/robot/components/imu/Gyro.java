package frc.robot.components.imu;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Gyro implements GyroIO {
  // The gyro sensor
  private Pigeon2 m_gyro = null;
  private final StatusSignal<Angle> yaw = m_gyro.getYaw();
  private final StatusSignal<AngularVelocity> yawVelocity = m_gyro.getAngularVelocityZWorld();

  public Gyro() {
    m_gyro = new Pigeon2(0, "rio"); // Make sure device ID and bus name are correct (add variable?)
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());
    m_gyro.getConfigurator().setYaw(0.0); // Zero's yaw at startup?
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK); // I assume if status code
                                                                                            // is OK, then the signal is
                                                                                            // valid and the gyro is
                                                                                            // connected.
    inputs.rotation2d = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.rotation3d = m_gyro.getRotation3d(); // Is this really necessary?
    inputs.yawVelocityRadPerSec = Math.toRadians(yawVelocity.getValueAsDouble());
  }

  @Override
  public void reset() {
    m_gyro.setYaw(0);
  }

  @Override
  public void DisplayIMUData() { // Lazy
    System.out.println("Gyro Yaw: " + yaw.getValueAsDouble());

  }
}