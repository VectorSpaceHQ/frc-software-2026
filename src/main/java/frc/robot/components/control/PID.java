package frc.robot.components.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.components.motor.MotorIO.MotorIOInputs;
import frc.robot.components.motor.MotorIOInputsAutoLogged;
import frc.robot.components.motor.MotorIO;

import org.littletonrobotics.junction.Logger;

public class PID implements Sendable {
    // private variables to be implemented
    private final MotorIOInputs m_motorInputs;
    private final MotorIO m_motor;
    private final MotorIOInputsAutoLogged motorInputs = new MotorIOInputsAutoLogged();

    // Note: These values will get overwritten by constants values
    private String name;
    private double MAX_RPM = 6000.0;
    private double m_gearRatio = 1.5;
    private double MAX_VOLTS = 12.0; // Unused (For reference)
    private final double MAX_RPM_PER_VOLT;
    
    private PIDController pid;
    private SimpleMotorFeedforward feedforward;

    // Set PID and feedforward default values (needs to be determined
    // experimentally)
    private double ks = 0.25; // static gain
    private double kv = 0; // velocity gain
    private double ka = 0; // accelerartion gain

    // kP times error (target value - measured value = error in calculate function)
    private double kp = 0.002; // proportional gain
    private double ki = 0; // integral gain
    private double kd = 0; // derivative gain

    private double lowIntegrationRange = -2.0;
    private double highIntegrationRange = 2.0;

    private double m_error;
    private double m_errorRPM;
    private double m_integralError;

    // private double m_motorspeed;

    private double m_volts;

    private double m_RPM = 0;
    private double m_realRPM;

    public PID(String name, MotorIO m_motorIO, double MAX_RPM, double MAX_VOLTS) {
        this.name = name;
        m_motor = m_motorIO;
        m_motorInputs = new MotorIOInputs();
        MAX_RPM_PER_VOLT = Units.rotationsPerMinuteToRadiansPerSecond(MAX_RPM / MAX_VOLTS); // https://www.reca.lc/motors
        kv = (1.0 / MAX_RPM_PER_VOLT);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        pid = new PIDController(kp, ki, kd);
        pid.setIntegratorRange(lowIntegrationRange, highIntegrationRange); // Integral is only responsible for -2 to 2
                                                                           // volts of input (adjustable)
    }

    public PID(String name, MotorIO m_motorIO, double MAX_RPM, double MAX_VOLTS, double ks, double kp, double ki,
            double kd) {
        this.name = name;
        m_motor = m_motorIO;
        m_motorInputs = new MotorIOInputs();
        MAX_RPM_PER_VOLT = Units.rotationsPerMinuteToRadiansPerSecond(MAX_RPM / MAX_VOLTS); // https://www.reca.lc/motors
        kv = (1.0 / MAX_RPM_PER_VOLT);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        pid = new PIDController(kp, ki, kd);
        pid.setIntegratorRange(lowIntegrationRange, highIntegrationRange); // Integral is only responsible for -2 to 2
                                                                           // volts of input (adjustable)
    }

    // The constructor we are using:
    public PID(String name, MotorIO m_motorIO, double MAX_RPM, double MAX_VOLTS, double m_gearRatio, double ks, double kp, double ki,
            double kd, double kv, double ka) {
        this.name = name;
        m_motor = m_motorIO;
        this.m_gearRatio = m_gearRatio;
        m_motorInputs = new MotorIOInputs();
        MAX_RPM_PER_VOLT = Units.rotationsPerMinuteToRadiansPerSecond(MAX_RPM / MAX_VOLTS);
        kv = (1.0 / MAX_RPM_PER_VOLT); // https://www.reca.lc/motors
        this.ks = ks;
        this.ka = ka;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        pid = new PIDController(kp, ki, kd);
        pid.setIntegratorRange(lowIntegrationRange, highIntegrationRange); // Integral is only responsible for -2 to 2

    } // volts of input (adjustable)

    public MotorIOInputs getMotorInputs() {
        return m_motorInputs;
    }

    public double getRawMotorRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(m_motorInputs.velocityRadPerSec);
    }

    public double calculate() {
        double target = Units.rotationsPerMinuteToRadiansPerSecond(m_RPM);
        m_volts = MathUtil.clamp(
                feedforward.calculate(target)
                        + pid.calculate(motorInputs.velocityRadPerSec, target),
                -12.0,
                12.0);
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
        m_motor.updateInputs(m_motorInputs); // With flywheel
        motorInputs.positionRad = m_motorInputs.positionRad / m_gearRatio;
        motorInputs.velocityRadPerSec = m_motorInputs.velocityRadPerSec / m_gearRatio;
        motorInputs.appliedVoltage = m_motorInputs.appliedVoltage;
        motorInputs.currentAmps = m_motorInputs.currentAmps;
        
        m_realRPM = Units.radiansPerSecondToRotationsPerMinute(motorInputs.velocityRadPerSec);
        // System.out.println(
        // "Pos: " + m_motorInputs.positionRad +
        // " Vel: " + m_motorInputs.velocityRadPerSec +
        // " Volt: " + m_motorInputs.appliedVoltage +
        // " Curr: " + m_motorInputs.currentAmps);

    }

    // Process the inputs for the logger
    public void processInputs(String key) {
        Logger.processInputs(key, motorInputs);
    }

    public void resetPID() {
        pid.reset();
    }

    // Error values for logging
    public double getError() { // Should be in rpm
        m_error = pid.getError();
        m_errorRPM = Units.radiansPerSecondToRotationsPerMinute(m_error);
        return m_errorRPM;
    }

    public double getIntegralError() { // Should be in voltage
        m_integralError = pid.getAccumulatedError();
        return m_integralError;
    }

    public boolean atSpeed(double toleranceRPM) {
        return Math.abs(getError()) <= toleranceRPM;
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
        feedforward.setKs(ks);
    }

    public void setkV(double kv) {
        this.kv = kv;
        feedforward.setKv(kv);
    }

    public void setkA(double ka) {
        this.ka = ka;
        feedforward.setKa(ka);
    }

    public double getkP() {
        return pid.getP();
    }

    public double getkI() {
        return pid.getI();
    }

    public double getkD() {
        return pid.getD();
    }

    public double getkS() {
        return feedforward.getKs();
    }

    public double getkV() {
        return feedforward.getKv();
    }

    public double getkA() {
        return feedforward.getKa();
    }

    public double getM_RPM() {
        return m_RPM;
    }

    public void PIDPeriodic(boolean resetStatus, boolean toggleStatus) {
        if (resetStatus) {
            this.resetPID();
        }
        if (toggleStatus) {
            calculate();
            this.m_setVoltage();
        } else {
            this.zeroVoltage();
        }
    }

    public void setM_RPM(double m_RPM) {
        if (m_RPM > MAX_RPM) {
            m_RPM = MAX_RPM;
        }
        this.m_RPM = m_RPM;
    }

    public double getM_volts() {
        return m_volts;
    }

    public double getM_realRPM() {
        return m_realRPM;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(name != null ? name : "Motor Status");
        builder.addDoubleProperty(name + "RPM", this::getM_RPM, this::setM_RPM);
        builder.addDoubleProperty(name + "Volts", this::getM_volts, null);
        builder.addDoubleProperty(name + "Real RPM", this::getM_realRPM, null);
        builder.addDoubleProperty(name + "Raw RPM", this::getRawMotorRPM, null);
        builder.addDoubleProperty(name + "Motor Error", this::getError, null);
        builder.addDoubleProperty(name + "Motor Integrated Error", this::getIntegralError, null);
        builder.addDoubleProperty(name + "kP", this::getkP, this::setkP);
        builder.addDoubleProperty(name + "kI", this::getkI, this::setkI);
        builder.addDoubleProperty(name + "kD", this::getkD, this::setkD);
        builder.addDoubleProperty(name + "kS", this::getkS, this::setkS);
        builder.addDoubleProperty(name + "kV", this::getkV, this::setkV);
        builder.addDoubleProperty(name + "kA", this::getkA, this::setkA);

    }

}