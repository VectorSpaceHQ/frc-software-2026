package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import frc.robot.components.motor.MotorIOSparkMax;
import frc.robot.components.control.PID;


public class IntakeSubsystem extends SubsystemBase implements Sendable {
  
    private final PID IntakeRollers1;
    private final PID IntakeRollers2;

    private boolean Intakestatus;
    private boolean lastIntakestatus;

    private final PID pivotMotorPid;
    private final ArmFeedforward pivotFeedforward;

    public IntakeSubsystem() {
        IntakeRollers1 = new PID(new MotorIOSparkMax(11),6000, 12, 0.25, 0.0015, 0.01, 0);
        IntakeRollers2 = new PID(new MotorIOSparkMax(12),-6000, 12, 0.25, 0.0015, 0.01, 0);
        pivotMotorPid = new PID(new MotorIOSparkMax(13),-6000, 12, 0.25, 0.0015, 0.01, 0);
        pivotFeedforward = new ArmFeedforward(0, 1.75, 1.95); // TODO: need to calibrate
  // Feed1Inputs = new MotorIOInputs();
  // Feed2Inputs = new MotorIOInputs();
        Intakestatus = false;
        lastIntakestatus = false;

        SmartDashboard.putData("Intake", this);
    }

    public boolean toggleIntake() {
        Intakestatus = !Intakestatus;
        return !Intakestatus;
    }

    // Place status values here
    public boolean getIntakestatus() {
        return Intakestatus;
    }

    public boolean getLastIntakestatus() {
        return lastIntakestatus;
    }


    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop
            IntakeRollers1.PIDPeriodic(Intakestatus && !lastIntakestatus, Intakestatus);
            IntakeRollers2.PIDPeriodic(Intakestatus && !lastIntakestatus, Intakestatus);

    }
    

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Intake init sendable called");
        builder.setSmartDashboardType("Intake Controller");
        builder.addBooleanProperty("Shooter Status", this::getIntakestatus, null);
        builder.addBooleanProperty("Last Shooter Status", this::getLastIntakestatus, null);
        IntakeRollers1.getPIDStatus(builder);
        IntakeRollers2.getPIDStatus(builder);
    }

}
