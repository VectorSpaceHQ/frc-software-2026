package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.components.motor.MotorIOKraken;
import frc.robot.components.motor.MotorIOSparkMax;
import frc.robot.configuration.Constants.ShooterConstants;
import frc.robot.configuration.Constants.OperatorConstants;
import frc.robot.configuration.Constants.SysIdEnums;
import frc.robot.configuration.Constants.SysIdEnums.SysIdTarget;
import frc.robot.configuration.configs.ShooterSubsysConfig;
import frc.robot.components.control.PID;
import frc.robot.components.control.SysId;

public class ShooterSubsystem extends SubsystemBase {

    private ShooterSubsysConfig shooterConfig = null;

    private PID english_PID = null;
    private PID main_PID = null;
    private PID feeder_PID = null;
    private SysIdRoutine englishSysId = null;
    private SysIdRoutine mainSysId = null;
    private SysIdRoutine feederSysId = null;

    private SysIdTarget sysIdTarget = SysIdTarget.MAIN;

    // private final double velocity_MOTOR =
    // Units.rotationsPerMinuteToRadiansPerSecond(509.3); // 53.33 rads/s
    // https://www.reca.lc/motors

    // private final ControllerIfc m_driverController;
    // private final ControllerIfc m_operatorController;

    private boolean shooterConfigPresent;
    private boolean shooterStatus;
    private boolean lastShooterStatus;
    private boolean runningSysId;

    public ShooterSubsystem(ShooterSubsysConfig config) {
        this.shooterConfig = config;
        shooterConfigPresent = shooterConfig.getIsPresent();
        shooterStatus = false;
        lastShooterStatus = false;
        runningSysId = ShooterConstants.RUNNING_SYS_ID;

        if (shooterConfigPresent) {
            // English Flywheel Mechanism
            english_PID = new PID(
                    "English",
                    new MotorIOKraken(this.shooterConfig.getShooterEnglishId()),
                    ShooterConstants.ENGLISH_MAX_RPM,
                    ShooterConstants.MAX_VOLTAGE,
                    ShooterConstants.GEAR_RATIO,
                    ShooterConstants.ENGLISH_kS,
                    ShooterConstants.ENGLISH_kP,
                    ShooterConstants.ENGLISH_kI,
                    ShooterConstants.ENGLISH_kD,
                    ShooterConstants.ENGLISH_kV,
                    ShooterConstants.ENGLISH_kA);

            // Main Flywheel Mechanism
            main_PID = new PID(
                    "Main",
                    new MotorIOKraken(this.shooterConfig.getShooterMainId()),
                    ShooterConstants.MAIN_MAX_RPM,
                    ShooterConstants.MAX_VOLTAGE,
                    ShooterConstants.GEAR_RATIO,
                    ShooterConstants.MAIN_kS,
                    ShooterConstants.MAIN_kP,
                    ShooterConstants.MAIN_kI,
                    ShooterConstants.MAIN_kD,
                    ShooterConstants.MAIN_kV,
                    ShooterConstants.MAIN_kA);

            // Feeder Flywheel Mechanism
            feeder_PID = new PID(
                    "Feeder",
                    new MotorIOSparkMax(this.shooterConfig.getFeederId(), ShooterConstants.FEEDER_CURRENT_LIMIT),
                    ShooterConstants.FEEDER_MAX_RPM,
                    ShooterConstants.MAX_VOLTAGE,
                    ShooterConstants.GEAR_RATIO,
                    ShooterConstants.FEEDER_kS,
                    ShooterConstants.FEEDER_kP,
                    ShooterConstants.FEEDER_kI,
                    ShooterConstants.FEEDER_kD,
                    ShooterConstants.FEEDER_kV,
                    ShooterConstants.FEEDER_kA);

            englishSysId = SysId.createRoutine(this, english_PID, "English");
            mainSysId = SysId.createRoutine(this, main_PID, "Main");
            feederSysId = SysId.createRoutine(this, feeder_PID, "Feeder");
            // t_motorInputs = new MotorIOInputs();
            // b_motorInputs = new MotorIOInputs();

            SmartDashboard.putData("Shooter/English PID", english_PID);
            SmartDashboard.putData("Shooter/Main PID", main_PID);
            SmartDashboard.putData("Shooter/Feeder PID", feeder_PID);
        }

        SmartDashboard.putBoolean("Shooter Present", shooterConfig.getIsPresent());
    }

    // Just in case
    public boolean startShooter() {
        shooterStatus = true;
        return shooterStatus;
    }

    // Just in case
    public boolean stopShooter() {
        shooterStatus = false;
        return shooterStatus;
    }

    public boolean toggleShooter() {
        shooterStatus = !shooterStatus;
        return shooterStatus;
    }

    // private Command setShooter = run(()-> RPM.of(rpm)).ignoringDisable(true);
    // private Command stopShooter = setVoltage(Volts.of(0)).ignoringDisable(true);

    public void setSysIdTarget(SysIdTarget target) {
        sysIdTarget = target;
    }

    public SysIdTarget getSysIdTarget() {
        return sysIdTarget;
    }

    // Place status values here
    public boolean getShooterStatus() {
        return shooterStatus;
    }

    public boolean getLastShooterStatus() {
        return lastShooterStatus;
    }

    public SysIdRoutine getActiveSysIdRoutine() {
        switch (sysIdTarget) {
            case ENGLISH:
                return englishSysId;
            case MAIN:
                return mainSysId;
            case FEEDER:
                return feederSysId;
            default:
                return feederSysId;
        }
    }

    public boolean atSpeed() {
        return english_PID.atSpeed(ShooterConstants.SHOOTER_SPEED_TOLERANCE_RPM)
                && main_PID.atSpeed(ShooterConstants.SHOOTER_SPEED_TOLERANCE_RPM)
                && feeder_PID.atSpeed(ShooterConstants.SHOOTER_SPEED_TOLERANCE_RPM);
    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop
        english_PID.m_updateInputs();
        main_PID.m_updateInputs();
        feeder_PID.m_updateInputs();

        english_PID.processInputs("Shooter/English");
        main_PID.processInputs("Shooter/Main");
        feeder_PID.processInputs("Shooter/Feeder");

        if (runningSysId == false & shooterConfig.getIsPresent()) {
            english_PID.PIDPeriodic(shooterStatus && !lastShooterStatus, shooterStatus);
            main_PID.PIDPeriodic(shooterStatus && !lastShooterStatus, shooterStatus);
            feeder_PID.PIDPeriodic(shooterStatus && !lastShooterStatus, shooterStatus);
        }

        /*
         * This line changes the shooter status of last to the shooter status of current
         * (so lastShooterStatus turns to true when shooterStatus is true).
         * But lastShooterStatus is initialized to false, so when lastShooterStatus
         * equals
         * shooterStatus initially, !lastShooterStatus does not equal true, meaning that
         * the PID does not reset.
         * * Furthermore, this happens every initialization of the shooter becoming
         * true.
         * If shooterStatus is false, !lastShooterStatus is true; if shooterStatus is
         * true, !lastShooterStatus remains true and resets the PID before becoming
         * false again. This should fix the problem that the integral term is building
         * error before shooter turns on, which needs to be reset (not periodically but
         * after every time the shooter turns on)
         */

        lastShooterStatus = shooterStatus;

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

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Shooter init sendable called");
        builder.setSmartDashboardType("Shooter Controller");
        builder.addBooleanProperty("Shooter Status", this::getShooterStatus, null);
        builder.addBooleanProperty("Last Shooter Status", this::getLastShooterStatus, null);
        builder.addBooleanProperty("At speed", this::atSpeed, null);

        super.initSendable(builder);
        english_PID.initSendable(builder);
        main_PID.initSendable(builder);
        feeder_PID.initSendable(builder);

    }

}