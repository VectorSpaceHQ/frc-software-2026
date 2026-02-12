package frc.robot.components.control;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public  class SysId {

    private SysId() {}

    public static SysIdRoutine createRoutine(
        SubsystemBase subsystem,
        PID PID,
        String name
    ) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> PID.m_setRawVoltage(voltage.in(Volts)),
                log -> {
                    var inputs = PID.getMotorInputs();
                    log.motor(name)
                        .voltage(Volts.of(inputs.appliedVoltage))
                        .angularPosition(Radians.of(inputs.positionRad))
                        .angularVelocity(RadiansPerSecond.of(inputs.velocityRadPerSec))
                        .current(Amps.of(inputs.currentAmps));
                },
                subsystem
            )
        );
    }
}
