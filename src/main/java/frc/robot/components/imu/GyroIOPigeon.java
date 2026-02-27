package frc.robot.components.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.components.imu.GyroIO;

public class GyroIOPigeon implements GyroIO {

    // The gyro sensor
    private final Pigeon2 m_gyro;

    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    public GyroIOPigeon(int canID) {

        m_gyro = new Pigeon2(canID, "rio");

        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        m_gyro.setYaw(0.0); // Zero yaw at startup

        yaw = m_gyro.getYaw();
        yawVelocity = m_gyro.getAngularVelocityZWorld();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        BaseStatusSignal.refreshAll(yaw, yawVelocity);

        inputs.connected = true;
        inputs.rotation2d = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Math.toRadians(yawVelocity.getValueAsDouble());
    }

    @Override
    public void reset() {
        m_gyro.setYaw(0);
    }
}
