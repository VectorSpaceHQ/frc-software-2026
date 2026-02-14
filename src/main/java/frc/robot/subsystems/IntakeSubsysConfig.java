package frc.robot.subsystems;
import static frc.robot.Constants.OperatorConstants.MotorCanIDEnum.*;
import frc.robot.Constants.OperatorConstants.SubSystemIDEnum;

public class IntakeSubsysConfig extends SubsystemConfig {
    private final int IntakeRoller1Id = INTAKE_ROLLERS_1_CANID.getCanID();
    private final int IntakeRoller2Id = INTAKE_ROLLERS_2_CANID.getCanID();
    private final int IntakePivotId = INTAKE_PIVOT_CANID.getCanID();

    public IntakeSubsysConfig(boolean isPresent,SubSystemIDEnum subSysId) {
        super(isPresent, subSysId);

    }
    public int getIntakeRoller1Id() {
        return this.IntakeRoller1Id;
    }
    public int getIntakeRoller2Id() {
        return this.IntakeRoller2Id;
    }
    public int getIntakePivotId() {
        return this.IntakePivotId;
    }
    
}
