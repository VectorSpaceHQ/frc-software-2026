package frc.robot.subsystems.intake;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.configuration.configs.IntakeSubsysConfig;
import frc.robot.configuration.Constants.SysIdEnums;
//import frc.robot.subsystems.shooter.ShooterSubsystem.SysIdTarget;
import frc.robot.configuration.Constants.SysIdEnums.SysIdTarget;
import frc.robot.components.motor.MotorIOSparkMax;
import frc.robot.components.motor.MotorIOSparkMaxFollower;
import frc.robot.components.control.PID;
import frc.robot.components.control.PivotPID;
import frc.robot.components.control.SysId;
import frc.robot.configuration.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends SubsystemBase {

    public enum SysIdTarget {
        ROLLER,
        PIVOT
    }

    private IntakeSubsysConfig IntakeConfig = null;
    private PID intakeRollerPid = null;
    private PivotPID pivotMotorPid = null; // left
    private SysIdRoutine intakeRollerSysId = null;
    private SysIdRoutine intakePivotSysId = null;

    private SysIdTarget sysIdTarget = SysIdTarget.ROLLER;
    private boolean Intakestatus = false;
    private boolean lastIntakestatus = false;
    private boolean runningSysId;

    private MotorIOSparkMax pivotMotor = null;

    @SuppressWarnings("unused")
    private MotorIOSparkMaxFollower pivotFollower = null;

    public IntakeSubsystem(IntakeSubsysConfig config) {
        this.IntakeConfig = config;
        runningSysId = IntakeConstants.RUNNING_SYS_ID;
        if (this.IntakeConfig.getIsPresent()) {

            intakeRollerPid = new PID("IntakeRoller", new MotorIOSparkMax(this.IntakeConfig.getIntakeRollerId()),
                    11000.0, 12.0,  1, 0.25, 0.01, 0.0, 0, 0.019098, 0);
            intakeRollerPid.setM_RPM(100);

            pivotMotor = new MotorIOSparkMax(this.IntakeConfig.getIntakePivotLeftId());
            pivotFollower = new MotorIOSparkMaxFollower( // Right pivot
                    this.IntakeConfig.getIntakePivotRightId(),
                    pivotMotor.getMotor(),
                    true // inverted
            );
            pivotMotorPid = new PivotPID("PivotMotor", pivotMotor,
                    60,
                    IntakeConstants.PIVOT_MIN_ANGLE_RAD,
                    IntakeConstants.PIVOT_MAX_ANGLE_RAD,
                    0, 0, 0, 0, // TODO: Find kS, kG, kV, kA
                    0.0015, 0.01, 0); // TODO: Find kP, kI, kD


            pivotMotor.zeroPosition();

            intakeRollerSysId = SysId.createRoutine(this, intakeRollerPid, "Roller");
            intakePivotSysId = SysId.createRoutine(this, pivotMotorPid, "Pivot");

            Intakestatus = false;
            lastIntakestatus = false;

            SmartDashboard.putData("Intake Roller PID", this.intakeRollerPid);
            SmartDashboard.putData("Pivot Motors PID", this.pivotMotorPid);
        }

        SmartDashboard.putBoolean("Intake Present", this.IntakeConfig.getIsPresent());
    }

    public boolean toggleIntake() {
        Intakestatus = !Intakestatus;
        return Intakestatus;
    }

    // Place status values here
    public boolean getIntakestatus() {
        return Intakestatus;
    }

    public boolean getLastIntakestatus() {
        return lastIntakestatus;
    }

    public void setPivotTarget(double angleDeg) {
        pivotMotorPid.setTargetAngleDeg(angleDeg);
    }

    public void setSysIdTarget(SysIdTarget target) {
        sysIdTarget = target;
    }

    public SysIdTarget getSysIdTarget() {
        return sysIdTarget;
    }

    private SysIdRoutine getActiveSysIdRoutine() {
        switch (sysIdTarget) {
            case PIVOT:  return intakePivotSysId;
            case ROLLER: return intakeRollerSysId;
            default:     return intakeRollerSysId;
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return getActiveSysIdRoutine().quasistatic(dir)
                .beforeStarting(() -> runningSysId = true)
                .finallyDo(() -> runningSysId = false);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return getActiveSysIdRoutine().dynamic(dir)
                .beforeStarting(() -> runningSysId = true)
                .finallyDo(() -> runningSysId = false);
    }

    public void runPivot() {
        pivotMotorPid.PivotPeriodic(Intakestatus && !lastIntakestatus, Intakestatus);
    }

    public void stopPivot() {
        pivotMotorPid.zeroVoltage();
    }

    public boolean pivotAtPosition() {
        return pivotMotorPid.atPosition(IntakeConstants.PIVOT_TOLERANCE_RAD);
    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop
        pivotMotorPid.m_updateInputs();
        intakeRollerPid.m_updateInputs();
        intakeRollerPid.processInputs("Intake/Roller");
        pivotMotorPid.processInputs("Intake/Pivot");

        if (!runningSysId && this.IntakeConfig.getIsPresent()) {
            intakeRollerPid.PIDPeriodic(Intakestatus && !lastIntakestatus, Intakestatus);
        }

        lastIntakestatus = Intakestatus;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Intake init sendable called");
        builder.setSmartDashboardType("Intake Controller");
        builder.addBooleanProperty("Intake Status", this::getIntakestatus, null);
        builder.addBooleanProperty("Last Intake Status", this::getLastIntakestatus, null);
        intakeRollerPid.initSendable(builder);
        pivotMotorPid.initSendable(builder);
    }

}