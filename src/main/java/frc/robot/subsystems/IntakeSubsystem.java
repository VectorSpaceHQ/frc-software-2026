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
    private final MotorIO Feed_motor1;
    private final MotorIO Feed_motor2;
    private final double MAX_RPM = 6000; // 628.32 rads/s

    private final double velocity_MOTOR = Units.rotationsPerMinuteToRadiansPerSecond(509.3); // 53.33 rads/s
                                                                                             // https://www.reca.lc/motors
    private final double k_INTAKE_TOLERANCE_RPS = 50;

    // private final ControllerIfc m_driverController;
    // private final ControllerIfc m_operatorController;

    private final MotorIOInputs FeedMotor1Inputs;
    private final MotorIOInputs FeedMotor2Inputs;

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

    double l_motorspeed;
    double r_motorspeed;
    double Feed1_volts;
    double Feed2_volts;

    double Motor1_RPM = 0.2 * MAX_RPM;
    double Motor2_RPM = -(Motor1_RPM);
    double Motor1_realRPM = 0;
    double Motor2_realRPM = 0;

    public IntakeSubsystem() {
        Feed_motor1 = new MotorIOKraken(19);
        Feed_motor2 = new MotorIOKraken(20);
        FeedMotor1Inputs = new MotorIOInputs();
        FeedMotor2Inputs = new MotorIOInputs();
        Feed1_feedforward = new SimpleMotorFeedforward(ks, kv);
        Feed2_feedforward = new SimpleMotorFeedforward(ks, kv);
        Feed1_pid = new PIDController(kp, ki, kd);
        Feed1_pid.setTolerance(k_INTAKE_TOLERANCE_RPS);
        Intakestatus = false;
        SmartDashboard.putData("Intake", this);
    }

    public void calculate() {

        double t_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(Motor1_RPM);
        double b_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(Motor2_RPM);

        Feed1_volts = MathUtil.clamp(Feed1_feedforward.calculate(t_targetRadsPerSec)
                + Feed1_pid.calculate(FeedMotor1Inputs.velocityRadPerSec, t_targetRadsPerSec), -12.0, 12.0);
                
        Feed2_volts = MathUtil.clamp(Feed2_feedforward.calculate(b_targetRadsPerSec)
                + Feed1_pid.calculate(FeedMotor2Inputs.velocityRadPerSec, b_targetRadsPerSec), -12.0, 12.0);

    }


    public boolean toggleIntake() {
        Intakestatus = !Intakestatus;
        return !Intakestatus;
    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop
        Feed_motor1.updateInputs(FeedMotor1Inputs);
        Feed_motor2.updateInputs(FeedMotor2Inputs);

        Motor1_realRPM = Units.radiansPerSecondToRotationsPerMinute(FeedMotor1Inputs.velocityRadPerSec);
        Motor2_realRPM = Units.radiansPerSecondToRotationsPerMinute(FeedMotor2Inputs.velocityRadPerSec);

        if (!Intakestatus) {
            calculate();

            Feed_motor1.setVoltage(Feed1_volts);
            Feed_motor2.setVoltage(Feed2_volts);
        } else {
            Feed_motor1.setVoltage(0);
            Feed_motor2.setVoltage(0);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Intake init sendable called");
        builder.setSmartDashboardType("Intake Controller");
        builder.addDoubleProperty("LeftRPM", this::getl_RPM, this::setl_RPM);
        builder.addDoubleProperty("RightRPM", this::getr_RPM, this::setr_RPM);
        builder.addDoubleProperty("Left Volts", this::getl_volts, null);
        builder.addDoubleProperty("Right Volts", this::getr_volts, null);
        builder.addDoubleProperty("Left Real RPM", this::getl_realRPM, null);
        builder.addDoubleProperty("Right Real RPM", this::getr_realRPM, null);
    }

    public double getl_RPM() {
        return Motor1_RPM;
    }

    public void setl_RPM(double l_RPM) {
        if (l_RPM > MAX_RPM) {
            l_RPM = MAX_RPM;
        }
        this.Motor1_RPM = l_RPM;
    }

    public double getr_RPM() {
        return Motor2_RPM;
    }

    public void setr_RPM(double r_RPM) {
        if (r_RPM > MAX_RPM) {
            r_RPM = MAX_RPM;
        }
        this.Motor2_RPM = r_RPM;
    }

    public double getl_volts() {
        return Feed1_volts;
    }

    public double getr_volts() {
        return Feed2_volts;
    }

    public double getl_realRPM() {
        return Motor1_realRPM;
    }

    public double getr_realRPM() {
        return Motor2_realRPM;
    }
}
