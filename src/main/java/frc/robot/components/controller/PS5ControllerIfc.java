package frc.robot.components.controller;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5ControllerIfc implements ControllerIfc {

    CommandPS5Controller joystick;

    public PS5ControllerIfc(int port) {
        joystick = new CommandPS5Controller(port);
    }

    // Swerve
    @Override
    public double getX() {
        return joystick.getLeftX();
    }

    @Override
    public double getY() {
        return joystick.getLeftY();
    }

    @Override
    public double getTwist() {
        return joystick.getRightX();
    }

    @Override
    public Trigger toggleOrientation() {
        return joystick.square();
    }

    // Shooter
    @Override
    public Trigger runShooter() {
        return joystick.L2();
    }

    @Override
    public Trigger stopShooter() {
        return joystick.R2();
    }

    // Intake
    @Override
    public Trigger toggleIntakeRollers() {
        return joystick.cross();
    }

    @Override
    public Trigger toggleIntakePivot() {
        return joystick.triangle();
    }

    // Indexer
    @Override
    public Trigger toggleIndexer() {
        return joystick.square();
    }

    // Climb
    @Override
    public Trigger runClimb() {
        return joystick.L1();
    }

    @Override
    public Trigger stopClimb() {
        return joystick.R1();
    }

    // Vision
    @Override
    public Trigger driveToTarget() {
        return joystick.circle();
    }

    // Tuning
    @Override
    public Trigger runQuasistatic() {
        return joystick.cross();
    }

    @Override
    public Trigger runQuasidynamic() {
        return joystick.circle();
    }

    @Override
    public Trigger runQuasistaticReverse() {
        return joystick.square();
    }

    @Override
    public Trigger runQuasidynamicReverse() {
        return joystick.triangle();
    }

    // Put Autos here:
}