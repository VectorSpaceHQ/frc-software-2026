package frc.robot.components.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerIfc implements ControllerIfc {

    double swerveX;
    double swerveY;
    double rotate;
    public Trigger shoot;
    CommandXboxController exampleJoystick;

    public XboxControllerIfc(int port) {

        exampleJoystick = new CommandXboxController(port);
    }

    public double getX() {

        return exampleJoystick.getLeftX();
    };

    public double getY() {

        return exampleJoystick.getLeftY();
    }

    public double getTwist() {

        return exampleJoystick.getRightX();

    }

    public double controlMotorSpeed() {

        return exampleJoystick.getLeftTriggerAxis();
    }

    public Trigger runShooter() {

        return exampleJoystick.b();
    }

    public Trigger runIntake() {

        return exampleJoystick.a();
    }

    public Trigger stopIntake() {

        return exampleJoystick.y();

    }

    // Button mapping needs to be changed since this has the same binding as runClimb()
    public Trigger toggleOrientation() {

        return exampleJoystick.rightBumper();
    }

    public Trigger runClimb() {

        return exampleJoystick.x();
    }

    public Trigger startShooter() {

        return exampleJoystick.start();
    }
}
