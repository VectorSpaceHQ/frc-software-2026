package frc.robot.components.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.components.motor.MotorIO.MotorIOInputs;
import frc.robot.components.motor.MotorIO;

public class PID {
    // private variables to be implemented
    private final MotorIOInputs m_motorInputs;
    private final MotorIO m_motor;
    
    private double MAX_RPM = 6000.0;
    private final double MAX_VOLTS = 12.0;
    private final double MAX_RPM_PER_VOLT; 
    private PIDController pid;
    private SimpleMotorFeedforward feedforward;

    // Set PID and feedforward values (needs to be determined
    // experimentally)
    private double ks = 0.25; // static gain
    private double kv = 0; // velocity gain

    // kP times error (target value - measured value = error in calculate function)
    private double kp = 0.002; // proportional gain (example error would be 0.002 * (628.32 - 0.0) = 1.25664 volts at startup)
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

    public PID(MotorIO m_motorIO, double MAX_RPM, double MAX_VOLTS) {
        m_motor = m_motorIO;
        m_motorInputs = new MotorIOInputs();
        MAX_RPM_PER_VOLT = Units.rotationsPerMinuteToRadiansPerSecond(MAX_RPM/MAX_VOLTS); // https://www.reca.lc/motors
        kv = (1.0/MAX_RPM_PER_VOLT);
        feedforward = new SimpleMotorFeedforward(ks, kv);
        pid = new PIDController(kp, ki, kd);
        pid.setIntegratorRange(lowIntegrationRange, highIntegrationRange); // Integral is only responsible for -2 to 2
                                                                             // volts of input (adjustable)
    }
    
    public PID(MotorIO m_motorIO, double MAX_RPM, double MAX_VOLTS, double ks, double kp, double ki, double kd) {
        m_motor = m_motorIO;
        m_motorInputs = new MotorIOInputs();
        MAX_RPM_PER_VOLT = Units.rotationsPerMinuteToRadiansPerSecond(MAX_RPM/MAX_VOLTS); // https://www.reca.lc/motors
        kv = (1.0/MAX_RPM_PER_VOLT);
        feedforward = new SimpleMotorFeedforward(ks, kv);
        pid = new PIDController(kp, ki, kd);
        pid.setIntegratorRange(lowIntegrationRange, highIntegrationRange); // Integral is only responsible for -2 to 2
                                                                             // volts of input (adjustable)
    }

    public PID(MotorIO m_motorIO, double MAX_RPM, double MAX_VOLTS, double ks, double kp, double ki, double kd, double kv) {
        m_motor = m_motorIO;
        m_motorInputs = new MotorIOInputs();
        MAX_RPM_PER_VOLT = Units.rotationsPerMinuteToRadiansPerSecond(MAX_RPM/MAX_VOLTS); // https://www.reca.lc/motors
        this.kv = kv;
        feedforward = new SimpleMotorFeedforward(ks, kv);
        pid = new PIDController(kp, ki, kd);
        pid.setIntegratorRange(lowIntegrationRange, highIntegrationRange); // Integral is only responsible for -2 to 2
                                                                             // volts of input (adjustable)
    }

    public MotorIOInputs getMotorInputs() {
        return m_motorInputs;
    }
    public double calculate() {
        double m_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_RPM);
        double m_volts = MathUtil.clamp(feedforward.calculate(m_targetRadsPerSec)
                + pid.calculate(m_motorInputs.velocityRadPerSec, m_targetRadsPerSec), -12.0, 12.0);
        return m_volts;
    }

    public void m_setVoltage() {
        m_motor.setVoltage(this.calculate());
    }
    public void zeroVoltage() {
        m_motor.setVoltage(0);
    }

    public void m_updateInputs() {
        m_motor.updateInputs(m_motorInputs);
        m_realRPM = Units.radiansPerSecondToRotationsPerMinute(m_motorInputs.velocityRadPerSec);
   
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

    public double getkP() {
        return pid.getP();
    }

    public double getkI() {
        return pid.getI();
    }

    public double getkD() {
        return pid.getD();
    }

    public double getM_RPM() {
        return m_RPM;
    }

    public void PIDPeriodic(boolean resetStatus, boolean toggleStatus) {
        this.m_updateInputs();
        if (resetStatus) {
            this.resetPID();
        }
        if (toggleStatus) {
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

    public void getPIDStatus(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Status");
        builder.addDoubleProperty("TopRPM", this::getM_RPM, this::setM_RPM);
        builder.addDoubleProperty("Top Volts", this::getM_volts, null);
        builder.addDoubleProperty("Top Real RPM", this::getM_realRPM, null);
        builder.addDoubleProperty("Top Motor Error", this::getError, null);
        builder.addDoubleProperty("Top Motor Integrated Error", this::getIntegralError, null);
        builder.addDoubleProperty("Top kP", this::getkP, this::setkP);
        builder.addDoubleProperty("Top kI", this::getkI, this::setkI);
        builder.addDoubleProperty("Top kD", this::getkD, this::setkD);
    }
 
}
