package frc.robot.subsystems;

import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.Interfaces.ControllerIfc;
import frc.Interfaces.XboxControllerIfc;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.components.motor.MotorIO;
import frc.robot.components.motor.MotorIOKraken;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ShooterSubsystem {
    private final MotorIO t_motor;
    private final MotorIO b_motor;
    private final double MAX_RPM = 6000;
    private final double STATIC_GAIN = 0.2; // in voltage
    private final double kV_MOTOR = 509.3; // in rpm / V: https://www.reca.lc/motors
    private final ControllerIfc m_driverController;
    private final ControllerIfc m_operatorController;
    private boolean shooterstatus;
    private SimpleMotorFeedforward feedforward;
    private PIDController pid;

    double  t_motorspeed;
    double  b_motorspeed; 
    double t_volts;
    double b_volts;

    double t_RPM;
    double b_RPM;

    public ShooterSubsystem(){
        m_driverController = new XboxControllerIfc(OperatorConstants.controllerPort1);
        m_operatorController = new XboxControllerIfc(OperatorConstants.controllerPort2);
        t_motor = new MotorIOKraken(19);
        b_motor = new MotorIOKraken(20);
        feedforward = new SimpleMotorFeedforward(STATIC_GAIN, Constants.MAX_MOTOR_VOLTS / kV_MOTOR);
        
        double getX = .2;
        double t_targetRPM = (getX * MAX_RPM);
        double t_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(t_targetRPM);
        t_volts = feedforward.calculate(t_targetRadsPerSec);

        double getTwist = .2;
        double b_targetRPM = (getTwist * MAX_RPM);
        double b_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(b_targetRPM);
        b_volts = feedforward.calculate(b_targetRadsPerSec);

        shooterstatus = false;

    }
// Place status values here
    public double getStatus() {
        return  t_motorspeed;
    }

    public boolean toggleShoot() {
        if (!shooterstatus) {
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
}
