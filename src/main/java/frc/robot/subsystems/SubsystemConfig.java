package frc.robot.subsystems;
import frc.robot.Constants.OperatorConstants.SubSystemIDEnum;

public class SubsystemConfig {
    private boolean isPresent;
    private SubSystemIDEnum subSysId;
    SubsystemConfig(boolean isPresent,SubSystemIDEnum subSysId) {
        this.isPresent = isPresent;
        this.subSysId = subSysId;

    }
    public boolean getIsPresent() {
        return this.isPresent;
    }
    public SubSystemIDEnum getSubSysId() {
        return this.subSysId;
    }
    
}
