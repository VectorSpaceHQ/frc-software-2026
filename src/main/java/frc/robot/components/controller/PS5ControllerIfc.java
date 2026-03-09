package frc.robot.components.controller;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5ControllerIfc implements ControllerIfc {

    CommandPS5Controller joystick;

    // Using PS5 Controller
    public PS5ControllerIfc(int port) {
        joystick = new CommandPS5Controller(port);
    }

    // Swerve
    @Override
    public double getX() {
        return joystick.getLeftX(); // Hold Threshold (On driver)
    }

    @Override
    public double getY() {
        return joystick.getLeftY(); // Hold Threshold (On driver)
    }

    @Override
    public double getTwist() {
        return joystick.getRightX(); // Hold Threshold (On driver)
    }

    @Override
    public Trigger toggleOrientation() {
        return joystick.triangle(); // Toggle (On driver)
    }

    @Override
    public Trigger halfSpeedModifier() {
        return joystick.R1();
    }

    // Shooter
    @Override
    public Trigger startShooter() {
        return joystick.L2(); // Unused (for now)
    }

    @Override
    public Trigger stopShooter() {
        return joystick.R2(); // Unused (for now)
    }

    @Override
    public Trigger toggleShooter() {
        return joystick.circle(); // Toggle (On operator)
    }

    // Intake
    @Override
    public Trigger toggleIntakeRollers() {
        return joystick.cross(); // Toggle (On operator)
    }

    @Override
    public Trigger sendPivotUp() {
        return joystick.R1(); // Toggle (On operator)
    }

    @Override
    public Trigger sendPivotDown() {
        return joystick.R2(); // Toggle (On operator)
    }

    // Indexer
    @Override
    public Trigger toggleIndexer() {
        return joystick.square(); // Toggle (On operator)
    }

    // Climb
    @Override
    public Trigger runClimbUp() {
        return joystick.L1(); // Hold or Toggle (On operator)
    }

    @Override
    public Trigger runClimbDown() {
        return joystick.L2(); // Hold or Toggle (On operator)
    }

    @Override
    public Trigger stopClimb() {
        return joystick.R3(); // Toggle (On operator)
    }

    // Vision
    @Override
    public Trigger driveToTarget() {
        return joystick.L1(); // Hold (On driver)
    }

    // Tuning
    @Override
    public Trigger runQuasistatic() {
        return joystick.cross(); // Hold (On testing)
    }

    @Override
    public Trigger runQuasidynamic() {
        return joystick.circle();// Hold (On testing)
    }

    @Override
    public Trigger runQuasistaticReverse() {
        return joystick.square(); // Hold (On testing)
    }

    @Override
    public Trigger runQuasidynamicReverse() {
        return joystick.triangle(); // Hold (On testing)
    }

    // Put Autos here:
    
}