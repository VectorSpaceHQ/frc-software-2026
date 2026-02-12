package frc.robot.components.motor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;

public class MotorIOSparkMax implements MotorIO {

    private final SparkMax motor;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder relativeEncoder;
    private final SparkLowLevel.MotorType motorType = SparkLowLevel.MotorType.kBrushless; // Brushless motor

    public MotorIOSparkMax(int canID) {
        motor = new SparkMax(canID, motorType);
        absoluteEncoder = motor.getAbsoluteEncoder();
        relativeEncoder = motor.getEncoder();
        relativeEncoder.setPosition(absoluteEncoder.getPosition()); // Sync to zero
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad = relativeEncoder.getPosition() * 2.0 * Math.PI; // Convert from rotations to radians

        inputs.velocityRadPerSec = relativeEncoder.getVelocity() * 2.0 * Math.PI * (1/60.0); // Convert from rotations per minute
                                                                                  // to radians per second

        inputs.appliedVoltage =   motor.getAppliedOutput() * motor.getBusVoltage();

        inputs.currentAmps = motor.getOutputCurrent();
        // Remember to add config files for sparkmax and neo motors to set current limits and other settings...
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts); // Applies the voltage
    }

    @Override
    public void stop() {
        motor.stopMotor(); // Stops the motor because no voltage (could be break mode or coast mode)
    }
}
