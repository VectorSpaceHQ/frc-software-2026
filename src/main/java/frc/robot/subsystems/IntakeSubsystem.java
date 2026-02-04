package frc.robot.subsystems;

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


public class IntakeSubsystem extends SubsystemBase implements Sendable {
    private final MotorIO Feed1;
    private final MotorIO Feed2;
    private final double MAX_RPM = 6000; // 628.32 rads/s

    private final double velocity_MOTOR = Units.rotationsPerMinuteToRadiansPerSecond(509.3); // 53.33 rads/s
                                                                                             // https://www.reca.lc/motors
    private final double k_INTAKE_TOLERANCE_RPS = 50;

    // private final ControllerIfc m_driverController;
    // private final ControllerIfc m_operatorController;

    private final MotorIOInputs Feed1Inputs;
    private final MotorIOInputs Feed2Inputs;

    private boolean Intakestatus;
    private SimpleMotorFeedforward Feed1_feedforward;
    private SimpleMotorFeedforward Feed2_feedforward;
    private PIDController Feed1_pid;

    // Set PID and feedforward values (needs to be determined
    // experimentally)
    //
    private double ks = 0.2; // in voltage (to overcome static friction)
    private double kv = (1.0 / velocity_MOTOR); // inverse of rads per second per volt (VoltsPerRadianPerSecond)

    // kP times error (target value - measured value = error in calculate function)
    private double kp = 0.002; // proportional gain (example error would be 0.002 * (628.32 - 0.0) = 1.25664 volts at startup)
    private double kd = 0;
    private double ki = 0;

    double Feed1_motorspeed;
    double Feed2_motorspeed;
    double Feed1_volts;
    double Feed2_volts;

    double Feed1_RPM = 0.2 * MAX_RPM;
    double Feed2_RPM = -(Feed1_RPM);
    double Feed1_realRPM = 0;
    double Feed2_realRPM = 0;

    public IntakeSubsystem() {
        Feed1 = new MotorIOKraken(19); //change canID later
        Feed2 = new MotorIOKraken(20); //change canID later
        Feed1Inputs = new MotorIOInputs();
        Feed2Inputs = new MotorIOInputs();
        Feed1_feedforward = new SimpleMotorFeedforward(ks, kv);
        Feed2_feedforward = new SimpleMotorFeedforward(ks, kv);
        Feed1_pid = new PIDController(kp, ki, kd);   // TODO decideif we need separate pids (ask henry)
        Feed1_pid.setTolerance(k_INTAKE_TOLERANCE_RPS);
        Intakestatus = false;
        SmartDashboard.putData("Intake", this);
    }

    public void calculate() {

        double Feed1_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(Feed1_RPM);
        double Feed2_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(Feed2_RPM);

        Feed1_volts = MathUtil.clamp(Feed1_feedforward.calculate(Feed1_targetRadsPerSec)
                + Feed1_pid.calculate(Feed1Inputs.velocityRadPerSec, Feed1_targetRadsPerSec), -12.0, 12.0);
                
        Feed2_volts = MathUtil.clamp(Feed2_feedforward.calculate(Feed2_targetRadsPerSec)
                + Feed1_pid.calculate(Feed2Inputs.velocityRadPerSec, Feed2_targetRadsPerSec), -12.0, 12.0);

    }


    public boolean toggleIntake() {
        Intakestatus = !Intakestatus;
        return !Intakestatus;
    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop
        Feed1.updateInputs(Feed1Inputs);
        Feed2.updateInputs(Feed2Inputs);

        Feed1_realRPM = Units.radiansPerSecondToRotationsPerMinute(Feed1Inputs.velocityRadPerSec);
        Feed2_realRPM = Units.radiansPerSecondToRotationsPerMinute(Feed2Inputs.velocityRadPerSec);

        if (!Intakestatus) {
            calculate();

            Feed1.setVoltage(Feed1_volts);
            Feed2.setVoltage(Feed2_volts);
        } else {
            Feed1.setVoltage(0);
            Feed2.setVoltage(0);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Intake init sendable called");
        builder.setSmartDashboardType("Intake Controller");
        builder.addDoubleProperty("Feed1RPM", this::getFeed1_RPM, this::setFeed1_RPM);
        builder.addDoubleProperty("Feed2RPM", this::getFeed2_RPM, this::setFeed2_RPM);
        builder.addDoubleProperty("Feed1 Volts", this::getFeed1_volts, null);
        builder.addDoubleProperty("Feed2 Volts", this::getFeed2_volts, null);
        builder.addDoubleProperty("Feed1 Real RPM", this::getFeed1_realRPM, null);
        builder.addDoubleProperty("Feed2 Real RPM", this::getFeed2_realRPM, null);
    }

    public double getFeed1_RPM() {
        return Feed1_RPM;
    }

    public void setFeed1_RPM(double Feed1_RPM) {
        if (Feed1_RPM > MAX_RPM) {
            Feed1_RPM = MAX_RPM;
        }
        this.Feed1_RPM = Feed1_RPM;
    }

    public double getFeed2_RPM() {
        return Feed2_RPM;
    }

    public void setFeed2_RPM(double Feed2_RPM) {
        if (Feed2_RPM > MAX_RPM) {
            Feed2_RPM = MAX_RPM;
        }
        this.Feed2_RPM = Feed2_RPM;
    }

    public double getFeed1_volts() {
        return Feed1_volts;
    }

    public double getFeed2_volts() {
        return Feed2_volts;
    }

    public double getFeed1_realRPM() {
        return Feed1_realRPM;
    }

    public double getFeed2_realRPM() {
        return Feed2_realRPM;
    }
}
