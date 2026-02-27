package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.util.Units;
import frc.robot.components.motor.MotorIOKraken;
import frc.robot.components.motor.MotorIOSparkMax;
import frc.robot.configuration.Constants.ShooterConstants;
import frc.robot.configuration.configs.ShooterSubsysConfig;
import frc.robot.components.control.PID;
import frc.robot.components.control.SysId;


public class ShooterSubsystem extends SubsystemBase {

    public enum SysIdTarget {
        ENGLISH,
        MAIN,
        FEEDER
    }

    private ShooterSubsysConfig shooterConfig = null;

    private PID english_PID = null;
    private PID main_PID = null;
    private PID feeder_PID = null;
    private SysIdRoutine englishSysId = null;
    private SysIdRoutine mainSysId = null;
    private SysIdRoutine feederSysId = null;

    private SysIdTarget sysIdTarget = SysIdTarget.FEEDER;

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
        runningSysId = false;

        if (shooterConfigPresent) {
            english_PID = new PID("English", new MotorIOKraken(this.shooterConfig.getShooterEnglishId()), 6000, 12, 0,
                    0,
                    0, 0, 0, 0);
            main_PID = new PID("Bottom", new MotorIOKraken(this.shooterConfig.getShooterMainId()), 6000, 12, 0,
                    0, 0, 0, 0, 0);
            feeder_PID = new PID("Neo", new MotorIOSparkMax(this.shooterConfig.getFeederId()), 5676, 12, 0, 0,
                    0, 0, 0, 0);

            englishSysId = SysId.createRoutine(this, english_PID, "English");
            mainSysId = SysId.createRoutine(this, main_PID, "Main");
            feederSysId = SysId.createRoutine(this, feeder_PID, "Feeder");
            // t_motorInputs = new MotorIOInputs();
            // b_motorInputs = new MotorIOInputs();

            SmartDashboard.putData("Shooter/Top PID", english_PID);
            SmartDashboard.putData("Shooter/Bottom PID", main_PID);
            SmartDashboard.putData("Shooter/Neo PID", feeder_PID);
        }

        SmartDashboard.putBoolean("Shooter Present", shooterConfig.getIsPresent());
    }

    public boolean toggleShoot() {
        shooterStatus = !shooterStatus;
        return shooterStatus; // Return the new value rather than the opposite
    }

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

    private SysIdRoutine getActiveSysIdRoutine() {
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

        if (!runningSysId & shooterConfig.getIsPresent()) {
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