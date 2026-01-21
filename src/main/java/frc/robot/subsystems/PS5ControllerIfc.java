package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class PS5ControllerIfc extends ControllerIfc {

    public PS5ControllerIfc(int port) {
        final CommandPS5Controller exampleJoystick = new CommandPS5Controller(port);
        swerveX = exampleJoystick.getLeftX();
        swerveY = exampleJoystick.getLeftY();
        rotate = exampleJoystick.getRightX();
        shoot = exampleJoystick.circle();
    }
    

}
