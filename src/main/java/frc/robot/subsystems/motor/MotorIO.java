package frc.robot.subsystems.motor;

public interface MotorIO {
    public static class MotorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(MotorIOInputs inputs) {
    }
    
    public default void update(double dt) {
    }

    public default void setVelocity(double velocityRadPerSec) {
    }

    public default void setVoltage(double volts) {
    }

    public default void stop() {
    }
}
