package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {
    private final DifferentialDrivetrainSim sim;
    private final ProfiledPIDController leftPID;
    private final ProfiledPIDController rightPID;
    private final TrapezoidProfile.Constraints constraints;
    private double leftAppliedVoltage = 0.0;
    private double rightAppliedVoltage = 0.0;

    private double leftPosition = 0.0;
    private double rightPosition = 0.0;

    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;

    private double leftPositionRad = 0.0;
    private double rightPositionRad = 0.0;

    private double maxVelocity = 0.0;
    private double maxAcceleration = 0.0;
    private double maxVoltage = 0.0;

    private KitbotMotor motor = KitbotMotor.kDualCIMPerSide;
    private KitbotGearing gearing = KitbotGearing.k10p71;
    private KitbotWheelSize wheelSize = KitbotWheelSize.kSixInch;
    private double wheelRadiusMeters = 0.0; // TBD

    private boolean closedLoop = false;

    public DriveIOSim() {
        sim = DifferentialDrivetrainSim.createKitbotSim(motor, gearing, wheelSize, null); // Assuming default values for
                                                                                          // gear ratio and wheel
        // size (not ours)

        constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        leftPID = new ProfiledPIDController(1.0, 0.0, 0.0, constraints); // Will implement constants
        rightPID = new ProfiledPIDController(1.0, 0.0, 0.0, constraints);

    }

    @Override
    public void setVelocity(double leftVelocity, double rightVelocity) {
        closedLoop = true;
        leftVelocity = sim.getLeftVelocityMetersPerSecond();
        rightVelocity = sim.getRightVelocityMetersPerSecond();
    }

    @Override
    public void update(double dt) {
        if (closedLoop) {
            sim.setInputs(leftAppliedVoltage, rightAppliedVoltage); // Not with PID yet
            leftPID.calculate(sim.getLeftVelocityMetersPerSecond());
            rightPID.calculate(sim.getRightVelocityMetersPerSecond());
            sim.update(0.02); // 20 ms
        }
    }

    @Override
    public void setVoltage(double leftVoltage, double rightVoltage) {
        closedLoop = false;
        leftAppliedVoltage = MathUtil.clamp(leftVoltage, -12, 12);
        rightAppliedVoltage = MathUtil.clamp(rightVoltage, -12, 12);
    }

    @Override
    public void updateInputs(DriveInterfaceInputs leftInputs, DriveInterfaceInputs rightInputs) {
        leftInputs.leftPosition = sim.getLeftPositionMeters() / wheelRadiusMeters;
        leftInputs.leftVelocity = sim.getLeftVelocityMetersPerSecond() / wheelRadiusMeters;
        leftInputs.leftAppliedVoltage = leftAppliedVoltage;
        leftInputs.leftAppliedCurrent = new double[] { sim.getLeftCurrentDrawAmps() };

        rightInputs.rightPosition = sim.getRightPositionMeters() / wheelRadiusMeters;
        rightInputs.rightVelocity = sim.getRightVelocityMetersPerSecond() / wheelRadiusMeters;
        rightInputs.rightAppliedVoltage = rightAppliedVoltage;
        rightInputs.rightAppliedCurrent = new double[] { sim.getRightCurrentDrawAmps() };
    }

}