package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {
    private DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide,
            KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null); // Assuming default values for gear ratio and wheel size (not ours)

    private double leftAppliedVoltage = 0.0;
    private double rightAppliedVoltage = 0.0;
    private boolean closedLoop = false;

    @Override
    public void updateInputs(DriveInterfaceInputs inputs) {
        // Implementation for updating inputs
        sim.setInputs(leftAppliedVoltage, rightAppliedVoltage); // To be calculated with actual simulated constants
        sim.update(0.02); // 20 ms

        // Rest of simulated calculations
    }
    
    @Override
    public void setVelocity(double leftVelocity, double rightVelocity) {
        // Implementation for setting velocity (will need calculations)
    }

    @Override
    public void setVoltage(double leftVoltage, double rightVoltage) {
        // Implementation for setting voltage
        leftAppliedVoltage = leftVoltage;
        rightAppliedVoltage = rightVoltage;
    }
}