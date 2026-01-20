package frc.robot.subsystems;

public interface DriveIO {
    public static class DriveInterfaceInputs {
        public double leftVelocity = 0.0;
        public double rightVelocity = 0.0;

        public double leftPositionRad = 0.0;
        public double rightPositionRad = 0.0;

        public double leftAppliedVoltage = 0.0;
        public double rightAppliedVoltage = 0.0;
    }

    public default void updateInputs(DriveInterfaceInputs inputs) {

    }

    public default void setVelocity(double leftVelocity, double rightVelocity) {

    }

    public default void setVoltage(double leftVoltage, double rightVoltage) {

    }
}
