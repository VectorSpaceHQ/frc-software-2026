package frc.robot.components.motor;

public interface MotorIO {
    public static class MotorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
    }

    public default void periodic () { // For sim only
    }

    public default void updateInputs(MotorIOInputs inputs) {
    }
    
    public default void setVoltage(double volts) {
    }

    public default void setVelocity(double velocityRadPerSec, double feedforwardVolts) { // Temporary
    }

    public default void stop() {
    }
}
