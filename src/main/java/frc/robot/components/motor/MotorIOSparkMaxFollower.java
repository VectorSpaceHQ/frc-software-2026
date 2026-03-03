package frc.robot.components.motor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel;

public class MotorIOSparkMaxFollower {

    private final SparkMax follower;
    private final SparkLowLevel.MotorType motorType = SparkLowLevel.MotorType.kBrushless; // Brushless motor

    public MotorIOSparkMaxFollower(int canID, SparkMax leader, boolean inverted) {
        follower = new SparkMax(canID, motorType);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.smartCurrentLimit(30);
        followerConfig.follow(leader, inverted); // Mirrors leader, inverted if opposite direction

        follower.configure(
            followerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }
}