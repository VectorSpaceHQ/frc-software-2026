package frc.robot.components.motor;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

public class MotorIOKrakenFollower implements MotorIO {
    private final TalonFX follower;
    public MotorIOKrakenFollower(int canIDfollower,int canIDleader, boolean inverted) {
        follower = new TalonFX(canIDfollower);
        MotorAlignmentValue alignment = MotorAlignmentValue.Aligned;

        if(inverted) {
            alignment = MotorAlignmentValue.Opposed;
        }

        follower.setControl(new Follower(canIDleader, alignment));
    }
}
