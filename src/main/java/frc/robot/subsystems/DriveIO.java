package frc.robot.subsystems; //

public interface DriveIO {
    public static class DriveInterfaceInputs {
        public double leftVelocity = 0.0;
        public double rightVelocity = 0.0;

        public double leftPosition = 0.0;
        public double rightPosition = 0.0;

        public double leftAppliedVoltage = 0.0;
        public double rightAppliedVoltage = 0.0;

        public double[] leftAppliedCurrent;
        public double[] rightAppliedCurrent;
    }

    public default void update(double dt) {

    }

    public default void updateInputs(DriveInterfaceInputs leftInputs, DriveInterfaceInputs rightInputs) {

    }

    public default void setVelocity(double leftVelocity, double rightVelocity) {

    }

    public default void setVoltage(double leftVoltage, double rightVoltage) {

    }

    public default void setCurrent(double leftCurrent, double rightCurrent) {

    }
}
