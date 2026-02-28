package frc.robot.components.control;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class SysId {

        private SysId() {
        }

        public static SysIdRoutine createRoutine(
                        SubsystemBase subsystem,
                        PID PID,
                        String name) {
                return new SysIdRoutine(
                                new SysIdRoutine.Config(
                                                Volts.of(0.5).per(Units.Second),
                                                Volts.of(3.5),
                                                Units.Seconds.of(8),
                                                (state) -> Logger.recordOutput("SysIdState", state.toString())
                                ),
                                new SysIdRoutine.Mechanism(
                                                voltage -> PID.m_setRawVoltage(voltage.in(Volts)), // Drive callback
                                                log -> {
                                                        var inputs = PID.getMotorInputs();
                                                        log.motor(name)
                                                                        .voltage(Volts.of(inputs.appliedVoltage))
                                                                        .angularPosition(Radians.of(inputs.positionRad))
                                                                        .angularVelocity(RadiansPerSecond
                                                                                        .of(inputs.velocityRadPerSec))
                                                                        .current(Amps.of(inputs.currentAmps));
                                                }, // Log callback
                                                subsystem));
        }

        public static SysIdRoutine createRoutine(
                        SubsystemBase subsystem,
                        PivotPID pivotPID,
                        String name) {
                return new SysIdRoutine(
                                new SysIdRoutine.Config(
                                                Volts.of(0.5).per(Units.Second),
                                                Volts.of(3.5),
                                                Units.Seconds.of(8),
                                                (state) -> Logger.recordOutput("SysIdState", state.toString())
                                ),
                                new SysIdRoutine.Mechanism(
                                                voltage -> pivotPID.m_setRawVoltage(voltage.in(Volts)), // Drive callback
                                                log -> {
                                                        var inputs = pivotPID.getMotorInputs();
                                                        log.motor(name)
                                                                        .voltage(Volts.of(inputs.appliedVoltage))
                                                                        .angularPosition(Radians.of(inputs.positionRad))
                                                                        .angularVelocity(RadiansPerSecond
                                                                                        .of(inputs.velocityRadPerSec))
                                                                        .current(Amps.of(inputs.currentAmps));
                                                }, // Log callback
                                                subsystem));
        }
}