//COMMANDJOYSTICK DOESN'T HAVE ENOUGH TRIGGERS TO INTERFACE CONTROLLER IFC

package frc.Interfaces;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControllerIfc implements ControllerIfc {

  private CommandJoystick joystick;
    
    public JoystickControllerIfc(int port) {
        
        joystick = new CommandJoystick(port);
    }
    
    public double getX() {
        
        return joystick.getX();
    };
      
    public double getY() {

        return joystick.getY();
    }

    public double getTwist(){

        return joystick.getTwist();
    }

    public Trigger runShooter(){

        return joystick.trigger();
    }

    public Trigger runIntake(){

        return joystick.top();
    }

    public Trigger stopIntake(){

        return joystick.top();
    }

 
}