package frc.robot.subsystems.Drive;

import frc.robot.components.motor.MotorIO;
import frc.robot.components.motor.MotorIOSim;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final MotorIOSim driveMotor;
    private final MotorIOSim steerMotor;
    private final CANcoder absoluteEncoder;

    private MotorIO.MotorIOInputs driveInputs = new MotorIO.MotorIOInputs();
    private MotorIO.MotorIOInputs steerInputs = new MotorIO.MotorIOInputs();

    public SwerveModuleIOSim(MotorIOSim driveMotor, MotorIOSim steerMotor, CANcoder absoluteEncoder) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.absoluteEncoder = absoluteEncoder;

        // Rest of configuration

    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveMotor.updateInputs(driveInputs);
        steerMotor.updateInputs(steerInputs);
    }

    @Override
    public void setDriveVelocity(double driveVelocityRadPerSec, double feedforwardVolts) {
        driveMotor.setVoltage(feedforwardVolts);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setSteerPosition(double steerPositionRad) {
    }

    @Override
    public void setSteerVoltage(double volts) {
        steerMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        driveMotor.stop();
        steerMotor.stop();
    }

}
