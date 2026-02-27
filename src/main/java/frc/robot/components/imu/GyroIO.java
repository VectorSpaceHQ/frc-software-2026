package frc.robot.components.imu;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public boolean connected;
    public Rotation2d rotation2d = new Rotation2d();
    public double yawVelocityRadPerSec;
  }

  default void updateInputs(GyroIOInputs inputs) {
  }

  default void reset() {
  }
}
