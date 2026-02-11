package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.Interfaces.ControllerIfc;
import frc.Interfaces.XboxControllerIfc;
import frc.robot.components.motor.MotorIO;
import frc.robot.components.motor.MotorIOKraken;
import frc.robot.components.motor.MotorIO.MotorIOInputs;

public class ShooterSubsystem extends SubsystemBase implements Sendable {
    private final MotorIO t_motor;
    private final MotorIO b_motor;
    private final double MAX_RPM = 6000; // 628.32 rads/s

    private final double velocity_MOTOR = Units.rotationsPerMinuteToRadiansPerSecond(509.3); // 53.33 rads/s
                                                                                             // https://www.reca.lc/motors

    // private final ControllerIfc m_driverController;
    // private final ControllerIfc m_operatorController;

    private final MotorIOInputs t_motorInputs;
    private final MotorIOInputs b_motorInputs;

    private boolean shooterStatus;
    private boolean lastShooterStatus;

    private SimpleMotorFeedforward t_feedforward;
    private SimpleMotorFeedforward b_feedforward;
    private PIDController t_pid;
    private PIDController b_pid;

    // Set PID and feedforward values (needs to be determined
    // experimentally)
    //
    private double ks = 0.25; // in voltage (to overcome static friction)
    private double kv = (1.0 / velocity_MOTOR); // inverse of rads per second per volt (VoltsPerRadianPerSecond)

    // kP times error (target value - measured value = error in calculate function)
    private double kp = 0.0015; // proportional gain (example error would be 0.002 * (628.32 - 0.0) = 1.25664
                                // volts at startup)
    private double ki = 0.01; // integral gain
    private double kd = 0; // derivative gain

    private double lowIntegrationRange = -2.0;
    private double highIntegrationRange = 2.0;

    private double t_error;
    private double b_error;
    private double t_errorRPM;
    private double b_errorRPM;
    private double t_integralError;
    private double b_integralError;

    double t_motorspeed;
    double b_motorspeed;
    double t_volts;
    double b_volts;

    double t_RPM = 0.2 * MAX_RPM;
    double b_RPM = 0.2 * MAX_RPM;
    double t_realRPM = 0;
    double b_realRPM = 0;

    public ShooterSubsystem() {
        t_motor = new MotorIOKraken(19);
        b_motor = new MotorIOKraken(20);

        t_motorInputs = new MotorIOInputs();
        b_motorInputs = new MotorIOInputs();

        t_feedforward = new SimpleMotorFeedforward(ks, kv);
        b_feedforward = new SimpleMotorFeedforward(ks, kv);

        t_pid = new PIDController(kp, ki, kd);
        b_pid = new PIDController(kp, ki, kd);

        t_pid.setIntegratorRange(lowIntegrationRange, highIntegrationRange); // Integral is only responsible for -2 to 2
                                                                             // volts of input (adjustable)
        b_pid.setIntegratorRange(lowIntegrationRange, highIntegrationRange); // Might need to be adjusted or commented
                                                                             // out

        shooterStatus = false;
        lastShooterStatus = false;

        SmartDashboard.putData("Shooter", this);
    }

    public void calculate() {

        double t_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(t_RPM);
        double b_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(b_RPM);

        t_volts = MathUtil.clamp(t_feedforward.calculate(t_targetRadsPerSec)
                + t_pid.calculate(t_motorInputs.velocityRadPerSec, t_targetRadsPerSec), -12.0, 12.0);

        b_volts = MathUtil.clamp(b_feedforward.calculate(b_targetRadsPerSec)
                + b_pid.calculate(b_motorInputs.velocityRadPerSec, b_targetRadsPerSec), -12.0, 12.0);

    }

    public void resetPID() {
        t_pid.reset();
        b_pid.reset();
    }

    // Place status values here
    public boolean getShooterStatus() {
        return shooterStatus;
    }

    public boolean getLastShooterStatus() {
        return lastShooterStatus;
    }

    // Error values for logging
    public double getTopError() { // Should be in rpm
        t_error = t_pid.getError();
        t_errorRPM = Units.radiansPerSecondToRotationsPerMinute(t_error);
        return t_errorRPM;
    }

    public double getBottomError() { // Should be in rpm
        b_error = b_pid.getError();
        b_errorRPM = Units.radiansPerSecondToRotationsPerMinute(b_error);
        return b_errorRPM;
    }

    public double getTopIntegralError() { // Should be in voltage
        t_integralError = t_pid.getAccumulatedError();
        return t_integralError;
    }

    public double getBottomIntegralError() { // Should be in voltage
        b_integralError = b_pid.getAccumulatedError();
        return b_integralError;
    }

    public boolean toggleShoot() {
        shooterStatus = !shooterStatus;
        return shooterStatus; // Return the new value rather than the opposite
    }

    // PID setters and getters for tuning
    public void setTopkP(double kp) {
        this.kp = kp;
        t_pid.setP(kp);
    }

    public void setTopkI(double ki) {
        this.ki = ki;
        t_pid.setI(ki);
    }

    public void setTopkD(double kd) {
        this.kd = kd;
        t_pid.setD(kd);
    }

    public void setBottomkP(double kp) {
        this.kp = kp;
        b_pid.setP(kp);
    }

    public void setBottomkI(double ki) {
        this.ki = ki;
        b_pid.setI(ki);
    }

    public void setBottomkD(double kd) {
        this.kd = kd;
        b_pid.setD(kd);
    }

    public double getTopkP() {
        return t_pid.getP();
    }

    public double getTopkI() {
        return t_pid.getI();
    }

    public double getTopkD() {
        return t_pid.getD();
    }

    public double getBottomkP() {
        return b_pid.getP();
    }

    public double getBottomkI() {
        return b_pid.getI();
    }

    public double getBottomkD() {
        return b_pid.getD();
    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop
        t_motor.updateInputs(t_motorInputs);
        b_motor.updateInputs(b_motorInputs);

        t_realRPM = Units.radiansPerSecondToRotationsPerMinute(t_motorInputs.velocityRadPerSec);
        b_realRPM = Units.radiansPerSecondToRotationsPerMinute(b_motorInputs.velocityRadPerSec);

        if (shooterStatus && !lastShooterStatus) { // If true (shooter is on and shooter was off)
            resetPID();
        }

        if (shooterStatus) { // If true (shooter is on)
            calculate();
            t_motor.setVoltage(t_volts);
            b_motor.setVoltage(b_volts);

        } else { // Shooter off
            t_motor.setVoltage(0);
            b_motor.setVoltage(0);
        }

        /*
         * This line changes the shooter status of last to the shooter status of current
         * (so lastShooterStatus turns to true when shooterStatus is true).
         * But lastShooterStatus is initialized to false, so when lastShooterStatus
         * equals
         * shooterStatus initially, !lastShooterStatus does not equal true, meaning that
         * the PID does not reset.
         * 
         * Furthermore, this happens every initialization of the shooter becoming true.
         * If shooterStatus is false, !lastShooterStatus is true; if shooterStatus is
         * true, !lastShooterStatus remains true and resets the PID before becoming
         * false again. This should fix the problem that the integral term is building
         * error before shooter turns on, which needs to be reset (not periodically but
         * after every time the shooter turns on)
         */

        lastShooterStatus = shooterStatus;

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Shooter init sendable called");
        builder.setSmartDashboardType("Shooter Controller");
        builder.addDoubleProperty("TopRPM", this::getT_RPM, this::setT_RPM);
        builder.addDoubleProperty("BottomRPM", this::getB_RPM, this::setB_RPM);
        builder.addDoubleProperty("Top Volts", this::getT_volts, null);
        builder.addDoubleProperty("Bottom Volts", this::getB_volts, null);
        builder.addDoubleProperty("Top Real RPM", this::getT_realRPM, null);
        builder.addDoubleProperty("Bottom Real RPM", this::getB_realRPM, null);
        builder.addDoubleProperty("Top Motor Error", this::getTopError, null);
        builder.addDoubleProperty("Bottom Motor Error", this::getBottomError, null);
        builder.addDoubleProperty("Top Motor Integrated Error", this::getTopIntegralError, null);
        builder.addDoubleProperty("Bottom Motor Integrated Error", this::getBottomIntegralError, null);
        builder.addBooleanProperty("Shooter Status", this::getShooterStatus, null);
        builder.addBooleanProperty("Last Shooter Status", this::getLastShooterStatus, null);
        builder.addDoubleProperty("Top kP", this::getTopkP, this::setTopkP);
        builder.addDoubleProperty("Top kI", this::getTopkI, this::setTopkI);
        builder.addDoubleProperty("Top kD", this::getTopkD, this::setTopkD);
        builder.addDoubleProperty("Bottom kP", this::getBottomkP, this::setBottomkP);
        builder.addDoubleProperty("Bottom kI", this::getBottomkI, this::setBottomkI);
        builder.addDoubleProperty("Bottom kD", this::getBottomkD, this::setBottomkD);

    }

    public double getT_RPM() {
        return t_RPM;
    }

    public void setT_RPM(double t_RPM) {
        if (t_RPM > MAX_RPM) {
            t_RPM = MAX_RPM;
        }
        this.t_RPM = t_RPM;
    }

    public double getB_RPM() {
        return b_RPM;
    }

    public void setB_RPM(double b_RPM) {
        if (b_RPM > MAX_RPM) {
            b_RPM = MAX_RPM;
        }
        this.b_RPM = b_RPM;
    }

    public double getT_volts() {
        return t_volts;
    }

    public double getB_volts() {
        return b_volts;
    }

    public double getT_realRPM() {
        return t_realRPM;
    }

    public double getB_realRPM() {
        return b_realRPM;
    }
}
