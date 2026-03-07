package frc.robot.components.controller;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class JoystickControllerIfc implements ControllerIfc {

    private CommandJoystick joystick;

    public enum ExtendedButtonType {
        Button3(3), Button4(4), Button5(5), Button6(6),
        Button7(7), Button8(8), Button9(9), Button10(10),
        Button11(11), Button12(12), Button13(13);

        public final int value;

        ExtendedButtonType(int value) {
            this.value = value;
        }
    }

    public JoystickControllerIfc(int port) {
        joystick = new CommandJoystick(port);
    }

    // Swerve
    @Override
    public double getX() {
        return joystick.getX();
    }

    @Override
    public double getY() {
        return joystick.getY();
    }

    @Override
    public double getTwist() {
        return joystick.getTwist();
    }

    @Override
    public Trigger toggleOrientation() {
        return getButton(ExtendedButtonType.Button3);
    }

    // Shooter
    @Override
    public Trigger runShooter() {
        return joystick.trigger();
    }

    @Override
    public Trigger stopShooter() {
        return joystick.top();
    }

    // Intake
    @Override
    public Trigger toggleIntakeRollers() {
        return getButton(ExtendedButtonType.Button4);
    }

    @Override
    public Trigger toggleIntakePivot() {
        return getButton(ExtendedButtonType.Button5);
    }

    // Indexer
    @Override
    public Trigger toggleIndexer() {
        return getButton(ExtendedButtonType.Button6);
    }

    // Climb
    @Override
    public Trigger runClimb() {
        return getButton(ExtendedButtonType.Button7);
    }

    @Override
    public Trigger stopClimb() {
        return getButton(ExtendedButtonType.Button8);
    }

    // Vision
    @Override
    public Trigger driveToTarget() {
        return getButton(ExtendedButtonType.Button9);
    }

    // Tuning
    @Override
    public Trigger runQuasistatic() {
        return getButton(ExtendedButtonType.Button10);
    }

    @Override
    public Trigger runQuasidynamic() {
        return getButton(ExtendedButtonType.Button11);
    }

    @Override
    public Trigger runQuasistaticReverse() {
        return getButton(ExtendedButtonType.Button12);
    }

    @Override
    public Trigger runQuasidynamicReverse() {
        return getButton(ExtendedButtonType.Button13);
    }

    // Put autos here:

    // Helper
    private Trigger getButton(ExtendedButtonType button) {
        return joystick.button(button.value, CommandScheduler.getInstance().getDefaultButtonLoop()); // Return button
                                                                                                     // loop
    }
    // Still needs to be tuned for diagram
}