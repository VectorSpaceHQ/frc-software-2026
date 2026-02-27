package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.configs.IntakeSubsysConfig;

import frc.robot.components.motor.MotorIOSparkMax;
import frc.robot.components.control.PID;


public class IntakeSubsystem extends SubsystemBase implements Sendable {

    private IntakeSubsysConfig IntakeConfig = null;
    private PID IntakeRollers1 = null;
    private PID IntakeRollers2 = null;

    private boolean Intakestatus = false;
    private boolean lastIntakestatus = false;

    private PID pivotMotorPid = null;
    private ArmFeedforward pivotFeedforward = null;

    public IntakeSubsystem(IntakeSubsysConfig config) {
        this.IntakeConfig = config;
        
        if (this.IntakeConfig.getIsPresent()) {

            IntakeRollers1 = new PID("IntakeRollers1",new MotorIOSparkMax(this.IntakeConfig.getIntakeRoller1Id()),6000.0, 12.0, 0.25, 0.0015, 0.01, 0.0);
            IntakeRollers2 = new PID("IntakeRollers2",new MotorIOSparkMax(this.IntakeConfig.getIntakeRoller2Id()),6000, 12, 0.25, 0.0015, 0.01, 0);
            pivotMotorPid = new PID("PivotMotor" ,new MotorIOSparkMax(this.IntakeConfig.getIntakePivotId()),6000, 12, 0.25, 0.0015, 0.01, 0);
            pivotFeedforward = new ArmFeedforward(0, 1.75, 1.95); // TODO: need to calibrate
  
            Intakestatus = false;
            lastIntakestatus = false;

            SmartDashboard.putData("Intake", this);
        }
    
        SmartDashboard.putBoolean("Intake Present", IntakeConfig.getIsPresent());
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
        if (this.IntakeConfig.getIsPresent()) {
            IntakeRollers1.PIDPeriodic(Intakestatus && !lastIntakestatus, Intakestatus);
            IntakeRollers2.PIDPeriodic(Intakestatus && !lastIntakestatus, Intakestatus);
        }
    }
    

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Intake init sendable called");
        builder.setSmartDashboardType("Intake Controller");
        builder.addBooleanProperty("Shooter Status", this::getIntakestatus, null);
        builder.addBooleanProperty("Last Shooter Status", this::getLastIntakestatus, null);
        IntakeRollers1.initSendable(builder);
        IntakeRollers2.initSendable(builder);
    }

}
