package frc.robot.subsystems;

import frc.robot.Constants.OperatorConstants.SubSystemIDEnum;
import frc.Interfaces.ControllerIfc;

public class SwerveSubsysConfig extends SubsystemConfig {
   private ControllerIfc controller = null;
   private double deadband = 0.0;

    public SwerveSubsysConfig(boolean isPresent,SubSystemIDEnum subSysId, ControllerIfc controller, double deadband){
        super(isPresent, subSysId);
        this.controller = controller;
        this.deadband = deadband;

    } 

    public ControllerIfc getController() {
      return controller;
    }

    public double getDeadband() {
      return deadband;
    }
}
