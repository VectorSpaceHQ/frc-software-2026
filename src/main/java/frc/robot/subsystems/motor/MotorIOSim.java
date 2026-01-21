package frc.robot.subsystems.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorIOSim implements MotorIO {

    private final DCMotorSim motorSim;

    private double appliedVoltage = 0.0;

    public MotorIOSim(DCMotor motor, double gearRatio, double moi) {
        var motorSystem = LinearSystemId.createDCMotorSystem(motor, moi, gearRatio);
        motorSim = new DCMotorSim(motorSystem, motor);
    }
    
    @Override
    public void updateSim(double dt) {
         motorSim.setInputVoltage(
            MathUtil.clamp(appliedVoltage, -12.0, 12.0));
        motorSim.update(dt);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad = motorSim.getAngularPositionRad();
        inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void stop() {
        appliedVoltage = 0.0;
    }
}
