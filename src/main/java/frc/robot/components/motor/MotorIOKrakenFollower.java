package frc.robot.components.motor;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

public class MotorIOKrakenFollower implements MotorIO {
    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    private final TalonFX follower;
    public MotorIOKrakenFollower(int canIDfollower,int canIDleader, boolean inverted, double StatorCurrentLimit, double SupplyCurrentLimit) {
        follower = new TalonFX(canIDfollower);
        MotorAlignmentValue alignment = MotorAlignmentValue.Aligned;

        if(inverted) {
            alignment = MotorAlignmentValue.Opposed;
        }
        
        talonFXConfig.CurrentLimits.StatorCurrentLimit = StatorCurrentLimit;
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.SupplyCurrentLimit = SupplyCurrentLimit;
        talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        follower.getConfigurator().apply(talonFXConfig);

        follower.setControl(new Follower(canIDleader, alignment));
    }
}
