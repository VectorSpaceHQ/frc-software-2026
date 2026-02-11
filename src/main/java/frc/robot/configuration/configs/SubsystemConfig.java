package frc.robot.configuration.configs;

import frc.robot.configuration.Constants.OperatorConstants.SubSystemIDEnum;

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
