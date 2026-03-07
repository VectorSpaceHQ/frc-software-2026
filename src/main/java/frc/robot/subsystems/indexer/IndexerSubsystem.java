package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
           
            IndexerPID = new PID("Indexer", new MotorIOSparkMax(this.IndexerConfig.getIndexerId(), 20), 6000, 12,25, 0.25, 0.005, 0.0005, 0, 0, 0);

            Indexerstatus = false;
            lastIndexerstatus = false;

            SmartDashboard.putData("Indexer PID", this.IndexerPID);
        }

        SmartDashboard.putBoolean("Indexer Present", this.IndexerConfig.getIsPresent());
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
