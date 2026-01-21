package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxControllerIfc extends ControllerIfc {

    public XboxControllerIfc(int port) {
        final CommandXboxController exampleJoystick = new CommandXboxController(port);
        swerveX = exampleJoystick.getLeftX();
        swerveY = exampleJoystick.getLeftY();
        rotate = exampleJoystick.getRightX();
        shoot = exampleJoystick.b();
    }
    

}
