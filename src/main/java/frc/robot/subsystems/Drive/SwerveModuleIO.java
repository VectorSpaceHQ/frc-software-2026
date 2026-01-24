package frc.robot.subsystems.Drive;

public interface SwerveModuleIO {
    public static class SwerveModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double driveCurrentAmps = 0.0;

        public double steerPositionRad = 0.0;
        public double steerVelocityRadPerSec = 0.0;
        public double steerAppliedVoltage = 0.0;
        public double steerCurrentAmps = 0.0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    public default void setDriveVelocity(double driveVelocityRadPerSec, double feedforwardVolts) {
    }

    public default void setDriveVoltage(double volts) {
    }

    public default void setSteerPosition(double steerPositionRad) {
    }

    public default void setSteerVoltage(double volts) {
    }

    public default void stop() {
    }

}