package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.Constants;
import frc.robot.configuration.Constants.IndexerConstants;
import frc.robot.configuration.configs.IndexerSubsysConfig;
import frc.robot.components.motor.MotorIOSparkMax;
import frc.robot.components.control.PID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;

public class IndexerSubsystem extends SubsystemBase {

    private IndexerSubsysConfig IndexerConfig = null;
    private PID IndexerPID = null;

    private boolean Indexerstatus = false;
    private boolean lastIndexerstatus = false;

    public IndexerSubsystem(IndexerSubsysConfig config) {

        this.IndexerConfig = config;

        if (this.IndexerConfig.getIsPresent()) {

            // Indexer Motor Mechanism
            IndexerPID = new PID(
                    "Indexer",
                    new MotorIOSparkMax(this.IndexerConfig.getIndexerId(),
                    IndexerConstants.INDEXER_CURRENT_LIMIT),
                    IndexerConstants.MAX_RPM,
                    Constants.MAX_MOTOR_VOLTS,
                    IndexerConstants.GEAR_RATIO,
                    IndexerConstants.kS,
                    IndexerConstants.kP,
                    IndexerConstants.kI,
                    IndexerConstants.kD,
                    IndexerConstants.kV,
                    IndexerConstants.kA);
            Indexerstatus = false;
            lastIndexerstatus = false;

            SmartDashboard.putData("Indexer PID", this.IndexerPID);
        }

        SmartDashboard.putBoolean("Indexer Present", this.IndexerConfig.getIsPresent());
        IndexerPID.setM_RPM(2500);
    }

    public boolean toggleIndexer() {
        Indexerstatus = !Indexerstatus;
        return !Indexerstatus;
    }

    // Place status values here
    public boolean getIndexerStatus() {
        return Indexerstatus;
    }

    public boolean getLastIndexerStatus() {
        return lastIndexerstatus;
    }

    public void setIndexerRPM(double RPM){
        IndexerPID.setM_RPM(RPM); //set the RPM of the Indexer
    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop
        if (this.IndexerConfig.getIsPresent()) {
            IndexerPID.m_updateInputs();

            IndexerPID.processInputs("Indexer/Indexer Motor");
            IndexerPID.PIDPeriodic(Indexerstatus && !lastIndexerstatus, Indexerstatus);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Indexer init sendable called");
        builder.setSmartDashboardType("Indexer Controller");
        IndexerPID.initSendable(builder);
    }

}
