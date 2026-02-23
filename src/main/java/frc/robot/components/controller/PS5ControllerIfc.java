package frc.robot.components.controller;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5ControllerIfc implements ControllerIfc {

    double swerveX;
    double swerveY;
    double rotate;
    public Trigger shoot;
    CommandPS5Controller exampleJoystick;

    public PS5ControllerIfc(int port) {

        exampleJoystick = new CommandPS5Controller(port);
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

        return exampleJoystick.getL2Axis();
    }

    public Trigger runShooter() {

        return exampleJoystick.circle();
    }

    public Trigger runIntake() {

        return exampleJoystick.cross();
    }

    public Trigger stopIntake() {

        return exampleJoystick.triangle();
    }

    // Button mapping needs to be changed since this has the same binding as runClimb()
    public Trigger toggleOrientation() {

        return exampleJoystick.square();
    }

    public Trigger runClimb() {

        return exampleJoystick.square();
    }

    public Trigger startShooter() {

        return exampleJoystick.options();
    }
}
