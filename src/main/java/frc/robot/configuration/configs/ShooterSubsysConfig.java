package frc.robot.configuration.configs;
import static frc.robot.configuration.Constants.OperatorConstants.MotorCanIDEnum.*;
import frc.robot.configuration.Constants.OperatorConstants.SubSystemIDEnum;

public class ShooterSubsysConfig extends SubsystemConfig {
    private final int FeederId = FEED_ROLLERS_CANID.getCanID();
    private final int ShooterMainId = SHOOTER_MAIN_MOTOR_CANID.getCanID();
    private final int ShooterEnglishId = SHOOTER_ENGLISH_MOTOR_CANID.getCanID();

    public ShooterSubsysConfig(boolean isPresent, SubSystemIDEnum subSysId) {
        super(isPresent, subSysId);

    }

    public int getFeederId() {
        return this.FeederId;
    }

    public int getShooterMainId() {
        return this.ShooterMainId;
    }

    public int getShooterEnglishId() {
        return this.ShooterEnglishId;
    }
}
