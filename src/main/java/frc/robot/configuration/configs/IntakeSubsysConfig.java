package frc.robot.configuration.configs;
import static frc.robot.configuration.Constants.OperatorConstants.MotorCanIDEnum.*;
import frc.robot.configuration.Constants.OperatorConstants.SubSystemIDEnum;

public class IntakeSubsysConfig extends SubsystemConfig {
    private final int IntakeRollerId = INTAKE_ROLLERS_CANID.getCanID();
    private final int IntakePivotLeftId = INTAKE_PIVOT_LEFT_CANID.getCanID();
    private final int IntakePivotRightId = INTAKE_PIVOT_RIGHT_CANID.getCanID();

    public IntakeSubsysConfig(boolean isPresent,SubSystemIDEnum subSysId) {
        super(isPresent, subSysId);

    }
    public int getIntakeRollerId() {
        return this.IntakeRollerId;
    }
    public int getIntakePivotLeftId() {
        return this.IntakePivotLeftId;
    }
    public int getIntakePivotRightId() {
        return this.IntakePivotRightId;
    }
    
}
