package frc.robot.configuration.configs;

import frc.robot.configuration.Constants.OperatorConstants.SubSystemIDEnum;
import frc.robot.components.controller.ControllerIfc;

public class SwerveSubsysConfig extends SubsystemConfig {
  private ControllerIfc controller = null;
  private double deadband = 0.0;

  public SwerveSubsysConfig(boolean isPresent, SubSystemIDEnum subSysId, ControllerIfc controller, double deadband) {
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
