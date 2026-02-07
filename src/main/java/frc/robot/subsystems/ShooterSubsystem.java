package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;


import frc.robot.components.motor.MotorIOKraken;
import frc.robot.components.motor.MotorIOSparkMax;
import frc.robot.components.control.PID;

public class ShooterSubsystem extends SubsystemBase {
    // private final MotorIO t_motor;
    // private final MotorIO b_motor;
    private final PID t_PID;
    private final PID b_PID;
    private final PID n_PID;
    
    // private final double velocity_MOTOR = Units.rotationsPerMinuteToRadiansPerSecond(509.3); // 53.33 rads/s
                                                                                             // https://www.reca.lc/motors

    // private final ControllerIfc m_driverController;
    // private final ControllerIfc m_operatorController;

    // private final MotorIOInputs t_motorInputs;
    // private final MotorIOInputs b_motorInputs;

    private boolean shooterStatus;
    private boolean lastShooterStatus;



    public ShooterSubsystem() {
        t_PID = new PID(new MotorIOKraken(19), 6000, 12, 0.25, 0.0015, 0.01, 0);
        b_PID = new PID(new MotorIOKraken(20), 6000, 12, 0.25, 0.0015, 0.01, 0, 1/Units.rotationsPerMinuteToRadiansPerSecond(509.3));
        n_PID = new PID(new MotorIOSparkMax(15), 6000, 12);
        // t_motorInputs = new MotorIOInputs();
        // b_motorInputs = new MotorIOInputs();

        shooterStatus = false;
        lastShooterStatus = false;

        SmartDashboard.putData("Shooter", this);
    }

    public boolean toggleShoot() {
        shooterStatus = !shooterStatus;
        return shooterStatus; // Return the new value rather than the opposite
    }
    // Place status values here
    public boolean getShooterStatus() {
        return shooterStatus;
    }

    public boolean getLastShooterStatus() {
        return lastShooterStatus;
    }

    
    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop

        t_PID.PIDPeriodic(shooterStatus && !lastShooterStatus, shooterStatus);
        b_PID.PIDPeriodic(shooterStatus && !lastShooterStatus, shooterStatus);
        n_PID.PIDPeriodic(shooterStatus && !lastShooterStatus, shooterStatus);
 

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
        builder.addBooleanProperty("Shooter Status", this::getShooterStatus, null);
        builder.addBooleanProperty("Last Shooter Status", this::getLastShooterStatus, null);
        t_PID.getPIDStatus(builder);
        b_PID.getPIDStatus(builder);
        n_PID.getPIDStatus(builder);
    }
    
}
