package frc.robot.subsystems;

import java.lang.Math;

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
    private final double k_SHOOTER_TOLERANCE_RPS = 50;

    // private final ControllerIfc m_driverController;
    // private final ControllerIfc m_operatorController;

    private final MotorIOInputs t_motorInputs;
    private final MotorIOInputs b_motorInputs;

    private boolean shooterstatus;
    private SimpleMotorFeedforward t_feedforward;
    private SimpleMotorFeedforward b_feedforward;
    private PIDController t_pid;
    private PIDController b_pid;

    // Set PID and feedforward values (needs to be determined
    // experimentally)
    //
    private double ks = 0.12; // in voltage (to overcome static friction)
    private double kv = (1.0 / velocity_MOTOR); // inverse of rads per second per volt (VoltsPerRadianPerSecond)

    // kP times error (target value - measured value = error in calculate function)
    private double kp = 0.0018; // proportional gain (example error would be 0.002 * (628.32 - 0.0) = 1.25664 volts at startup)
    private double kd = 0;
    private double ki = 0.00001;

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
        t_pid.setTolerance(k_SHOOTER_TOLERANCE_RPS);
        b_pid.setTolerance(k_SHOOTER_TOLERANCE_RPS);
        shooterstatus = false;
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

    // Place status values here
    public double getStatus() {
        return t_motorspeed;
    }

    public boolean toggleShoot() {
        //shooterstatus = !shooterstatus;
        return !shooterstatus;
    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop
        t_motor.updateInputs(t_motorInputs);
        b_motor.updateInputs(b_motorInputs);

        t_realRPM = Units.radiansPerSecondToRotationsPerMinute(t_motorInputs.velocityRadPerSec);
        b_realRPM = Units.radiansPerSecondToRotationsPerMinute(b_motorInputs.velocityRadPerSec);

        if (!shooterstatus) {
            calculate();
            t_motor.setVoltage(t_volts);
            b_motor.setVoltage(b_volts);
            
        } else {
            t_motor.setVoltage(0);
            b_motor.setVoltage(0);
        }
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
        // round to two places
        return (Math.floor(t_volts * 100) / 100.0);
    }

    public double getB_volts() {
        return (Math.floor(b_volts * 100) / 100.0);
    }

    public double getT_realRPM() {
        return Math.round(t_realRPM);
    }

    public double getB_realRPM() {
        return Math.round(b_realRPM);
    }
}
