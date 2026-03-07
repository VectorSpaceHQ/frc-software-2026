package frc.robot.components.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerIfc implements ControllerIfc {

    CommandXboxController joystick;

    public XboxControllerIfc(int port) {
        joystick = new CommandXboxController(port);
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
        return -joystick.getRightX();
    }

    @Override
    public Trigger toggleOrientation() {
        return joystick.x();
    }

    // Shooter
    @Override
    public Trigger runShooter() {
        return joystick.leftTrigger();
    }

    @Override
    public Trigger stopShooter() {
        return joystick.rightTrigger();
    }

    // Intake
    @Override
    public Trigger toggleIntakeRollers() {
        return joystick.a();
    }

    @Override
    public Trigger toggleIntakePivot() {
        return joystick.y();
    }

    // Indexer
    @Override
    public Trigger toggleIndexer() {
        return joystick.x();
    }

    // Climb
    @Override
    public Trigger runClimb() {
        return joystick.leftBumper();
    }

    @Override
    public Trigger stopClimb() {
        return joystick.rightBumper();
    }

    // Vision
    @Override
    public Trigger driveToTarget() {
        return joystick.b();
    }

    // Tuning
    @Override
    public Trigger runQuasistatic() {
        return joystick.a();
    }

    @Override
    public Trigger runQuasidynamic() {
        return joystick.b();
    }

    @Override
    public Trigger runQuasistaticReverse() {
        return joystick.x();
    }

    @Override
    public Trigger runQuasidynamicReverse() {
        return joystick.y();
    }

    // Put Autos here:
}