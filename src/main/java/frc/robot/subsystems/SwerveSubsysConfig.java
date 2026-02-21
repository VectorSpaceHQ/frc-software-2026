package frc.robot.subsystems;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.SubSystemIDEnum;
import swervelib.SwerveInputStream;

public class SwerveSubsysConfig extends SubsystemConfig {
    public SwerveSubsysConfig(boolean isPresent,SubSystemIDEnum subSysId){
        super(isPresent, subSysId);

    }
      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getY() * -1,
        () -> m_driverController.getX() * -1)
        .withControllerRotationAxis(m_driverController::getTwist)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(false);

    
}
