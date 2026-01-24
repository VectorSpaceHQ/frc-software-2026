package frc.robot.subsystems;

import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.Interfaces.ControllerIfc;
import frc.Interfaces.XboxControllerIfc;
import frc.robot.Constants.OperatorConstants;
import frc.robot.components.motor.MotorIO;
import frc.robot.components.motor.MotorIOKraken;

public class ShooterSubsystem {
    private final MotorIO t_motor;
    private final MotorIO b_motor;
    private final double MAX_RPM = 6000;
    private final ControllerIfc m_driverController;
    private final ControllerIfc m_operatorController;
    private boolean shooterstatus;
    private SimpleMotorFeedforward feedforward;

    double  t_motorspeed;
    double  b_motorspeed;

    public ShooterSubsystem(){
    m_driverController = new XboxControllerIfc(OperatorConstants.controllerPort1);
    m_operatorController = new XboxControllerIfc(OperatorConstants.controllerPort2);
    t_motor = new MotorIOKraken(19);
    b_motor = new MotorIOKraken(20);
    shooterstatus = false;
    feedforward = new SimpleMotorFeedforward(0.2, 12/509.3);
    t_motorspeed = 3.2;
    b_motorspeed = -3.2;
    }
// Place status values here
    public double getStatus() {
        return  t_motorspeed;
    }

    public boolean toggleShoot() {
    if (!shooterstatus) {
        t_motor.setVoltage(t_motorspeed);
        b_motor.setVoltage(b_motorspeed);
        
    }
    else {
        t_motor.setVoltage(0);
        b_motor.setVoltage(0);
    }
   shooterstatus = !shooterstatus;
    return !shooterstatus ;
    }
}
