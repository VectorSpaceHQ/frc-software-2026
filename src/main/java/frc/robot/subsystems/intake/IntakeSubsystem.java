package frc.robot.subsystems.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.configuration.configs.IntakeSubsysConfig;
import frc.robot.configuration.Constants.IntakeConstants.PivotState;
import frc.robot.components.motor.MotorIOKraken;
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
    private MotorIOKraken intakeRollerMotor;
    private PivotPID pivotMotorPid = null; // left
    private SysIdRoutine intakeRollerSysId = null;
    private SysIdRoutine intakePivotSysId = null;

    private SysIdTarget sysIdTarget = SysIdTarget.ROLLER;
    private boolean Intakestatus = false;
    private boolean lastIntakestatus = false;
    private boolean runningSysId;

    private MotorIOSparkMax pivotMotor = null;
    private PivotState currentPivotState = PivotState.UP;

    @SuppressWarnings("unused")
    private MotorIOSparkMaxFollower pivotFollower = null;

    public IntakeSubsystem(IntakeSubsysConfig config) {
        this.IntakeConfig = config;
        runningSysId = IntakeConstants.RUNNING_SYS_ID;
        if (this.IntakeConfig.getIsPresent()) {

            // Intake Roller Mechanism
            // feel free to move this into your custom class but there need to be current
            // limits.

            intakeRollerMotor = new MotorIOKraken(this.IntakeConfig.getIntakeRollerId());
            intakeRollerPid = new PID(
                    "IntakeRoller",
                    intakeRollerMotor, // how to set current limit??
                    IntakeConstants.ROLLER_MAX_RPM,
                    IntakeConstants.MAX_VOLTAGE,
                    IntakeConstants.ROLLER_GEAR_RATIO,
                    IntakeConstants.ROLLER_kS,
                    IntakeConstants.ROLLER_kP,
                    IntakeConstants.ROLLER_kI,
                    IntakeConstants.ROLLER_kD,
                    IntakeConstants.ROLLER_kV,
                    IntakeConstants.ROLLER_kA);
            intakeRollerPid.setM_RPM(IntakeConstants.ROLLER_STARTER_RPM); // Negative to go in the other direction

            pivotMotor = new MotorIOSparkMax(this.IntakeConfig.getIntakePivotLeftId(), 20);
            pivotFollower = new MotorIOSparkMaxFollower( // Right pivot
                    this.IntakeConfig.getIntakePivotRightId(),
                    pivotMotor.getMotor(),
                    true, // inverted
                    IntakeConstants.PIVOT_CURRENT_LIMIT);

            // Pivot Motor Mechanism
            pivotMotorPid = new PivotPID(
                    "PivotMotor",
                    pivotMotor,
                    IntakeConstants.PIVOT_GEAR_RATIO,
                    IntakeConstants.PIVOT_MIN_ANGLE_RAD,
                    IntakeConstants.PIVOT_MAX_ANGLE_RAD,
                    IntakeConstants.PIVOT_kS,
                    IntakeConstants.PIVOT_kG,
                    IntakeConstants.PIVOT_kV,
                    IntakeConstants.PIVOT_kA,
                    IntakeConstants.PIVOT_kP,
                    IntakeConstants.PIVOT_kI,
                    IntakeConstants.PIVOT_kD);
            pivotMotor.zeroPosition();

            intakeRollerSysId = SysId.createRoutine(this, intakeRollerPid, "Roller");
            intakePivotSysId = SysId.createRoutine(this, pivotMotorPid, "Pivot");

            Intakestatus = false;
            lastIntakestatus = false;
            SmartDashboard.putData("Intake Controller", this);

        }

        SmartDashboard.putBoolean("Intake Present", this.IntakeConfig.getIsPresent());
    }

    public void toggleRollers() {
        Intakestatus = !Intakestatus;
    }

    // Toggles the pivot up
    public void sendPivotUp() {
        this.currentPivotState = PivotState.UP;
    }

    // Toggles the pivot down
    public void sendPivotDown() {
        this.currentPivotState = PivotState.DOWN;
    }

    // Alternate stop pivot command using pivot state
    public void stopPivotAlt() {
        this.currentPivotState = PivotState.OFF;
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
            case PIVOT:
                return intakePivotSysId;
            case ROLLER:
                return intakeRollerSysId;
            default:
                return intakeRollerSysId;
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
        pivotMotorPid.PivotPeriodic(false, true);
    }

    public void setPivotState(PivotState state) {
        this.currentPivotState = state;
    }

    public PivotState getPivotState() {
        return currentPivotState;
    }

    public void stopPivot() {
        pivotMotorPid.zeroVoltage();
    }

    public boolean pivotAtPosition() {
        return pivotMotorPid.atPosition(IntakeConstants.PIVOT_TOLERANCE_RAD);
    }

    public void resetPivotPID() {
        pivotMotorPid.resetPID();
    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop
        pivotMotorPid.m_updateInputs();
        intakeRollerPid.m_updateInputs();
        intakeRollerPid.processInputs("Intake/Roller");
        pivotMotorPid.processInputs("Intake/Pivot");

        if (!runningSysId && this.IntakeConfig.getIsPresent()) {
            // Just send the constant voltage of the current state
            pivotMotor.setVoltage(currentPivotState.voltage);

            intakeRollerPid.PIDPeriodic(Intakestatus && !lastIntakestatus, Intakestatus);
        }

        lastIntakestatus = Intakestatus;
        SmartDashboard.putString("Pivot Position", this.currentPivotState.textName());
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