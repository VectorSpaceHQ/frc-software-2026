// This class is somewhat useless now.
package frc.robot.components.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController; // Switched to Profiled
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    private ProfiledPIDController profiledPid; // Updated to Profiled
    private ArmFeedforward feedforward;

    // Set PID and feedforward default values
    private double ks = 0; // Static gain (voltage gain to start motor, closed-loop)
    private double kg = 0; // Gravity gain (voltage gain to make motor raise arm against gravity, closed-loop)
    private double kv = 0; // Velocity gain (voltage gain to make motor spin at constant velocity, closed-loop)
    private double ka = 0; // Acceleration gain (voltage gain to make motor spin at constant acceleration, closed-loop)

    private double kp = 0; // Proportional gain (voltage gain based on proportional error to approach setpoint and correct error, current)
    private double ki = 0; // Integral gain (voltage gain based on integral error to account for past errors accumulated over time, past)
    private double kd = 0; // Derivative gain (voltage gain based on derivative error to account for potential overshoots or extreme oscillations, future)

    // Max volts integral component can use
    private double lowIntegrationRange = -2.0;
    private double highIntegrationRange = 2.0;

    private double m_targetAngleRad = 0;
    private double m_volts = 0;

    private double minAngleRad;
    private double maxAngleRad;

    private TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(3, 3);

    public PivotPID(String name, MotorIO m_motorIO, double m_gearRatio,
            double minAngleRad, double maxAngleRad,
            double ks, double kg, double kv, double ka,
            double kp, double ki, double kd) {
        this.name = name;
        m_motor = m_motorIO;
        m_motorInputs = new MotorIOInputs();
        this.m_gearRatio = m_gearRatio;
        this.minAngleRad = minAngleRad;
        this.maxAngleRad = maxAngleRad;
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        // Initialize the profiled controller
        profiledPid = new ProfiledPIDController(kp, ki, kd, m_constraints);
        profiledPid.setIntegratorRange(lowIntegrationRange, highIntegrationRange);

        // Initialize feedforward
        feedforward = new ArmFeedforward(ks, kg, kv, ka);
    }
public MotorIOInputs getMotorInputs() {
        return m_motorInputs;
    }

    public void m_setRawVoltage(double volts) {
        m_motor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }
    public double getCurrentAngleRad() {
        return m_motorInputs.positionRad / m_gearRatio;
    }

    public double getCurrentVelocityRadPerSec() {
        return m_motorInputs.velocityRadPerSec / m_gearRatio;
    }

    public double calculate() {
        double currentAngle = getCurrentAngleRad();

        var setpoint = profiledPid.getSetpoint();

        double ffVolts = feedforward.calculate(setpoint.position, setpoint.velocity);

        double pidVolts = profiledPid.calculate(currentAngle, m_targetAngleRad);

        m_volts = MathUtil.clamp(ffVolts + pidVolts, -12, 12);
        return m_volts;
    }

    public void m_setVoltage() {
        m_motor.setVoltage(m_volts);
    }

    public void zeroVoltage() {
        m_motor.setVoltage(0);
    }

    public void m_updateInputs() {
        m_motor.updateInputs(m_motorInputs);
        // Sync the autologged inputs for AdvantageKit
        motorInputs.positionRad = getCurrentAngleRad();
        motorInputs.velocityRadPerSec = getCurrentVelocityRadPerSec();
        motorInputs.appliedVoltage = m_motorInputs.appliedVoltage;
        motorInputs.currentAmps = m_motorInputs.currentAmps;
    
    }

    public void processInputs(String key) {
        Logger.processInputs(key, motorInputs);
    }

    public void resetPID() {
        // Reset the profile to the current position of the arm
        profiledPid.reset(getCurrentAngleRad(), 0);
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

    public void setTargetAngleDeg(double angleDeg) {
        setTargetAngleRad(Units.degreesToRadians(angleDeg));
    }

    public double getTargetAngleDeg() {
        return Units.radiansToDegrees(m_targetAngleRad);
    }

    public double getCurrentAngleDeg() {
        return Units.radiansToDegrees(getCurrentAngleRad());
    }

    public void PivotPeriodic(boolean resetStatus, boolean active) {
        if (resetStatus) {
            resetPID();
        }

        if (active) {
            calculate();
            m_setVoltage();
        } else {
            zeroVoltage();
        }
    }

    // Tuning
    public void setkP(double kp) {
        this.kp = kp;
        profiledPid.setP(kp);
    }

    public void setkI(double ki) {
        this.ki = ki;
        profiledPid.setI(ki);
    }

    public void setkD(double kd) {
        this.kd = kd;
        profiledPid.setD(kd);
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

    public double getkP() {
        return profiledPid.getP();
    }

    public double getkI() {
        return profiledPid.getI();
    }

    public double getkD() {
        return profiledPid.getD();
    }

    public double getkS() {
        return ks;
    }

    public double getkG() {
        return kg;
    }

    public double getkV() {
        return kv;
    }

    public double getkA() {
        return ka;
    }

    public double getM_volts() {
        return m_volts;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(name != null ? name : "Pivot Status");
        builder.addDoubleProperty(name + "Target Angle (deg)", this::getTargetAngleDeg, this::setTargetAngleDeg);
        builder.addDoubleProperty(name + "Current Angle (deg)", this::getCurrentAngleDeg, null);
        builder.addDoubleProperty(name + "Volts", this::getM_volts, null);
        builder.addDoubleProperty(name + "kP", this::getkP, this::setkP);
        builder.addDoubleProperty(name + "kI", this::getkI, this::setkI);
        builder.addDoubleProperty(name + "kD", this::getkD, this::setkD);
        builder.addDoubleProperty(name + "kS", this::getkS, this::setkS);
        builder.addDoubleProperty(name + "kG", this::getkG, this::setkG);
        builder.addDoubleProperty(name + "kV", this::getkV, this::setkV);
        builder.addDoubleProperty(name + "kA", this::getkA, this::setkA);
    }
}