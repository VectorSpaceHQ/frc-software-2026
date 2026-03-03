package frc.robot.components.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.components.motor.MotorIO;
import frc.robot.components.motor.MotorIO.MotorIOInputs;
import frc.robot.components.motor.MotorIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class PivotPID implements Sendable {

    private final MotorIOInputs m_motorInputs;
    private final MotorIO m_motor;
    private final MotorIOInputsAutoLogged motorInputs = new MotorIOInputsAutoLogged();

    private String name;
    private double m_gearRatio;

    private PIDController pid;
    private ArmFeedforward feedforward;

    // Set PID and feedforward default values (needs to be determined
    // experimentally)
    private double ks = 0; // static gain
    private double kg = 0; // gravity gain
    private double kv = 0; // velocity gain
    private double ka = 0; // acceleration gain

    private double kp = 0; // proportional gain
    private double ki = 0; // integral gain
    private double kd = 0; // derivative gain

    private double lowIntegrationRange = -2.0;
    private double highIntegrationRange = 2.0;

    private double m_targetAngleRad = 0;
    private double m_volts = 0;

    private double minAngleRad;
    private double maxAngleRad;

    // Using arm feedforward:
    public PivotPID(String name, MotorIO m_motorIO, double m_gearRatio,
            double minAngleRad, double maxAngleRad,
            double ks, double kg, double kv, double ka,
            double kp, double ki, double kd) {
        this.name = name;
        m_motor = m_motorIO;
        m_motorInputs = new MotorIOInputs();
        this.m_gearRatio = m_gearRatio;
        this.minAngleRad = minAngleRad; // Min and max angle defined in constants
        this.maxAngleRad = maxAngleRad;
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        pid = new PIDController(kp, ki, kd);
        pid.setIntegratorRange(lowIntegrationRange, highIntegrationRange); // Integral is only responsible for -2 to 2
                                                                           // volts of input (adjustable)
    }

    public MotorIOInputs getMotorInputs() {
        return m_motorInputs;
    }

    public double getCurrentAngleRad() {
        return m_motorInputs.positionRad / m_gearRatio;
    }

    public double getCurrentVelocityRadPerSec() {
        return m_motorInputs.velocityRadPerSec / m_gearRatio;
    }

    public double calculate() {
        double currentAngle = getCurrentAngleRad();
        double currentVelocity = getCurrentVelocityRadPerSec();
        m_volts = MathUtil.clamp(
                feedforward.calculate(currentAngle, currentVelocity)
                        + pid.calculate(currentAngle, m_targetAngleRad),
                -6,
                6);
        return m_volts;
    }

    public void m_setVoltage() { // For PID
        m_motor.setVoltage(m_volts);
    }

    public void m_setRawVoltage(double volts) {
        m_motor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    public void zeroVoltage() {
        m_motor.setVoltage(0);
    }

    public void m_updateInputs() {
        m_motor.updateInputs(m_motorInputs);
        motorInputs.positionRad = getCurrentAngleRad();
        motorInputs.velocityRadPerSec = getCurrentVelocityRadPerSec();
        motorInputs.appliedVoltage = m_motorInputs.appliedVoltage;
        motorInputs.currentAmps = m_motorInputs.currentAmps;
    }

    // Process the inputs for the logger
    public void processInputs(String key) {
        Logger.processInputs(key, motorInputs);
    }

    public void resetPID() {
        pid.reset();
    }

    public boolean withinLimits() {
        double angle = getCurrentAngleRad();
        return angle >= minAngleRad && angle <= maxAngleRad;
    }

    public boolean atPosition(double toleranceRad) {
        return Math.abs(m_targetAngleRad - getCurrentAngleRad()) <= toleranceRad;
    }

    public void setTargetAngleRad(double angleRad) {
        m_targetAngleRad = MathUtil.clamp(angleRad, minAngleRad, maxAngleRad);
    }

    public double getTargetAngleDeg() {
        return Units.radiansToDegrees(m_targetAngleRad);
    }

    public void setTargetAngleDeg(double angleDeg) {
        setTargetAngleRad(Units.degreesToRadians(angleDeg));
    }

    public double getCurrentAngleDeg() {
        return Units.radiansToDegrees(getCurrentAngleRad());
    }

    public void PivotPeriodic(boolean resetStatus, boolean active) {
        if (resetStatus) {
            resetPID();
        }
        if (active) {
            if (!withinLimits()) {
                zeroVoltage(); // Hard stop if out of bounds
                return;
            }
            calculate();
            m_setVoltage();
        } else {
            zeroVoltage();
        }
    }

    // Error values for logging
    public double getErrorDeg() {
        return Units.radiansToDegrees(m_targetAngleRad - getCurrentAngleRad());
    }

    public double getIntegralError() {
        return pid.getAccumulatedError();
    }

    // PID setters and getters for tuning
    public void setkP(double kp) {
        this.kp = kp;
        pid.setP(kp);
    }

    public void setkI(double ki) {
        this.ki = ki;
        pid.setI(ki);
    }

    public void setkD(double kd) {
        this.kd = kd;
        pid.setD(kd);
    }

    public void setkS(double ks) {
        this.ks = ks;
        feedforward = new ArmFeedforward(ks, kg, kv, ka);
    }

    public void setkG(double kg) {
        this.kg = kg;
        feedforward = new ArmFeedforward(ks, kg, kv, ka);
    }

    public void setkV(double kv) {
        this.kv = kv;
        feedforward = new ArmFeedforward(ks, kg, kv, ka);
    }

    public void setkA(double ka) {
        this.ka = ka;
        feedforward = new ArmFeedforward(ks, kg, kv, ka);
    }

    public double getkP() { return pid.getP(); }
    public double getkI() { return pid.getI(); }
    public double getkD() { return pid.getD(); }
    public double getkS() { return ks; }
    public double getkG() { return kg; }
    public double getkV() { return kv; }
    public double getkA() { return ka; }
    public double getM_volts() { return m_volts; }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(name != null ? name : "Pivot Status");
        builder.addDoubleProperty(name + "Target Angle (deg)", this::getTargetAngleDeg, this::setTargetAngleDeg);
        builder.addDoubleProperty(name + "Current Angle (deg)", this::getCurrentAngleDeg, null);
        builder.addDoubleProperty(name + "Error (deg)", this::getErrorDeg, null);
        builder.addDoubleProperty(name + "Integral Error", this::getIntegralError, null);
        builder.addDoubleProperty(name + "Volts", this::getM_volts, null);
        builder.addBooleanProperty(name + "Within Limits", this::withinLimits, null);
        builder.addDoubleProperty(name + "kP", this::getkP, this::setkP);
        builder.addDoubleProperty(name + "kI", this::getkI, this::setkI);
        builder.addDoubleProperty(name + "kD", this::getkD, this::setkD);
        builder.addDoubleProperty(name + "kS", this::getkS, this::setkS);
        builder.addDoubleProperty(name + "kG", this::getkG, this::setkG);
        builder.addDoubleProperty(name + "kV", this::getkV, this::setkV);
        builder.addDoubleProperty(name + "kA", this::getkA, this::setkA);
    }

}