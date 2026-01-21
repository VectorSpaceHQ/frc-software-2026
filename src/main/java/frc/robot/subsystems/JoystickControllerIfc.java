package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class JoystickControllerIfc extends ControllerIfc {

    public JoystickControllerIfc(int port) {
        final CommandJoystick exampleJoystick = new CommandJoystick(port);
        swerveX = exampleJoystick.getX();
        swerveY = exampleJoystick.getY();
        rotate = exampleJoystick.getTwist();
        shoot = exampleJoystick.top();
    }
    

}
