package frc.robot.subsystems;
import frc.robot.Constants;
import static frc.robot.Constants.OperatorConstants.MotorCanIDEnum.*;
import frc.robot.Constants.OperatorConstants.SubSystemIDEnum;

public class ShooterSubsysConfig extends SubsystemConfig {
    private final int FeederId = FEED_ROLLERS_CANID.getCanID();
    private final int FiringId = FIRING_ROLLERS_CANID.getCanID();
    //private final int ShooterTopId = SHOOTER_TOP_MOTOR_CANID.getCanID(); //TODO replace PROTO with these
    //private final int ShooterBottomId = SHOOTER_BOTTOM_MOTOR_CANID.getCanID(); //TODO replace PROTO with these
    private final int ShooterTopId = PROTO_SHOOTER_TOP_MOTOR_CANID.getCanID();
    private final int ShooterBottomId = PROTO_SHOOTER_BOTTOM_MOTOR_CANID.getCanID();
    public ShooterSubsysConfig(boolean isPresent,SubSystemIDEnum subSysId) {
        super(isPresent, subSysId);

    }
    public int getFeederId() {
        return this.FeederId;
    }
    public int getFiringId() {
        return this.FiringId;
    }
    public int getShooterTopId() {
        return this.ShooterTopId;
    }
    public int getShooterBottomId() {
        return this.ShooterBottomId;
    }
}
