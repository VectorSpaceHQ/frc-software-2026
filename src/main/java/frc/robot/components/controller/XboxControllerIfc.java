package frc.robot.components.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerIfc implements ControllerIfc {

    CommandXboxController joystick;

    // Using Xbox Controller (Logitech in this case)
    public XboxControllerIfc(int port) {
        joystick = new CommandXboxController(port);
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
        return -joystick.getRightX(); // Hold Threshold (On driver)
    }

    @Override
    public Trigger toggleOrientation() {
        return joystick.y(); // Toggle (On driver)
    }

    @Override
    public Trigger halfSpeedModifier() {
        return joystick.leftTrigger(); // while held (On driver)
    }
    
    // Shooter
    @Override
    public Trigger startShooter() {
        return joystick.leftTrigger(); // Unused (for now)
    }

    @Override
    public Trigger stopShooter() {
        return joystick.rightTrigger(); // Unused (for now)
    }

    @Override
    public Trigger toggleShooter() {
        return joystick.rightTrigger(); // Toggle (On operator)
    }

    @Override
    public Trigger closeShot() {
        return joystick.a(); // Toggle (On operator)
    }
    
    @Override
    public Trigger farShot() {
        return joystick.b(); // Toggle (On operator)
    }

    // Intake
    @Override
    public Trigger toggleIntakeRollers() {
        return joystick.povUp(); // Toggle (On operator)
    }

    @Override
    public Trigger sendPivotUp() {
        return joystick.leftBumper(); // Toggle (On operator)
    }

    @Override
    public Trigger sendPivotDown() {
        return joystick.leftTrigger(); // Toggle (On operator)
    }

    // Indexer
    @Override
    public Trigger toggleIndexer() {
        return joystick.rightBumper(); // Toggle (On operator)
    }

    // Climb
    @Override
    public Trigger runClimbUp() {
        return joystick.leftBumper(); // Hold or Toggle (On operator)
    }

    @Override
    public Trigger runClimbDown() {
        return joystick.leftTrigger(); // Hold or Toggle (On operator)
    }

    @Override
    public Trigger stopClimb() {
        return joystick.rightStick(); // Toggle (On operator)
    }

    // Vision
    @Override
    public Trigger driveToTarget() {
        return joystick.x(); // Hold (On driver)
    }

    // Tuning
    @Override
    public Trigger runQuasistatic() {
        return joystick.a(); // Hold (On testing)
    }

    @Override
    public Trigger runQuasidynamic() {
        return joystick.b(); // Hold (On testing)
    }

    @Override
    public Trigger runQuasistaticReverse() {
        return joystick.x(); // Hold (On testing)
    }

    @Override
    public Trigger runQuasidynamicReverse() {
        return joystick.y(); // Hold (On testing)
    }

    // Put Autos here:
    
}