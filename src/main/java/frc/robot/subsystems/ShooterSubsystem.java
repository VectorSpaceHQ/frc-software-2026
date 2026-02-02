package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Interfaces.ControllerIfc;
import frc.Interfaces.XboxControllerIfc;
import frc.robot.Constants.OperatorConstants;
import frc.robot.components.motor.MotorIO;
import frc.robot.components.motor.MotorIOKraken;
import frc.robot.components.motor.MotorIO.MotorIOInputs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ShooterSubsystem extends SubsystemBase implements Sendable{
    private final MotorIO t_motor;
    private final MotorIO b_motor;
    private final double MAX_RPM = 6000;
    private boolean shooterstatus;
    private SimpleMotorFeedforward feedforward;


    double  t_motorspeed;
    double  b_motorspeed; 
    double t_volts;
    double b_volts;

    double t_RPM = 0.2 * MAX_RPM;
    double b_RPM = 0.2 * MAX_RPM;
    double t_realRPM = 0;
    double b_realRPM = 0;

    public ShooterSubsystem(){
        t_motor = new MotorIOKraken(19);
        b_motor = new MotorIOKraken(20);
        feedforward = new SimpleMotorFeedforward(0.2, 12/509.3);
        shooterstatus = false;
        SmartDashboard.putData("Shooter",this);
    }


    public void calculate() {

        double t_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(t_RPM);
        t_volts = feedforward.calculate(t_targetRadsPerSec);

        
        double b_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(b_RPM);
        b_volts = feedforward.calculate(b_targetRadsPerSec);

    }


// Place status values here
    public double getStatus() {
        return  t_motorspeed;
    }

    public boolean toggleShoot() {
        if (!shooterstatus) {
            calculate();
            t_motor.setVoltage(t_volts);
            b_motor.setVoltage(b_volts);
        }
        else {
            t_motor.setVoltage(0);
            b_motor.setVoltage(0);
        }
        shooterstatus = !shooterstatus;
        return !shooterstatus ;
    }


    @Override
    public void periodic(){
        MotorIO.MotorIOInputs topMotorIOInputs = new MotorIOInputs();
        MotorIO.MotorIOInputs bottomMotorIOInputs = new MotorIOInputs();
        t_motor.updateInputs(topMotorIOInputs);
        b_motor.updateInputs(bottomMotorIOInputs);
        t_realRPM = Units.radiansPerSecondToRotationsPerMinute(topMotorIOInputs.velocityRadPerSec);
        b_realRPM = Units.radiansPerSecondToRotationsPerMinute(bottomMotorIOInputs.velocityRadPerSec);

    }

    @Override
    public void initSendable (SendableBuilder builder) {
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
        if (t_RPM > MAX_RPM){
            t_RPM = MAX_RPM;
        }
        this.t_RPM = t_RPM;
    }

    public double getB_RPM() {
        return b_RPM;
    }

    public void setB_RPM(double b_RPM) {
        if (b_RPM > MAX_RPM){
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
