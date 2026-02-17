package frc.robot.components.motor;
import org.littletonrobotics.junction.AutoLog;


public interface MotorIO {
    
    @AutoLog
    public class MotorIOInputs {
        public double positionRad;
        public double velocityRadPerSec;
        public double appliedVoltage;
        public double currentAmps;
    }

    public default void periodic() { // For sim only
    }

    public default void updateInputs(MotorIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void stop() {
    }
}
