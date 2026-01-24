package frc.robot.components.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class MotorIOKraken implements MotorIO {

    private final TalonFX motor;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public MotorIOKraken(int canID) {
        motor = new TalonFX(canID);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad =
            motor.getPosition().getValueAsDouble() * 2.0 * Math.PI; // Convert from rotations to radians

        inputs.velocityRadPerSec =
            motor.getVelocity().getValueAsDouble() * 2.0 * Math.PI; // Convert from rotations per second to radians per second

        inputs.appliedVoltage =
            motor.getMotorVoltage().getValueAsDouble();

        inputs.currentAmps =
            motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts)); // Applies the voltage
    }
    @Override
    public void setVelocity(double velocityRadPerSec, double feedforwardVolts) {
        double velocityRotationsPerSec = velocityRadPerSec / (2.0 * Math.PI);
        motor.setControl(
            new VelocityVoltage(velocityRotationsPerSec).withFeedForward(feedforwardVolts));
    }

    @Override
    public void stop() {
        motor.setControl(neutralRequest); // Stops the motor because no voltage (could be break mode or coast mode)
    }
}
