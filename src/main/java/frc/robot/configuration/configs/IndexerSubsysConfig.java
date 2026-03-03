package frc.robot.configuration.configs;

import static frc.robot.configuration.Constants.OperatorConstants.MotorCanIDEnum.*;
import frc.robot.configuration.Constants.OperatorConstants.SubSystemIDEnum;

public class IndexerSubsysConfig extends SubsystemConfig {
    private final int IndexerId = WASHING_MACHINE_INDEXER_CANID.getCanID();

    public IndexerSubsysConfig(boolean isPresent,SubSystemIDEnum subSysId) {
        super(isPresent, subSysId);

    }
    public int getIndexerId() {
        return this.IndexerId;
    }
}
