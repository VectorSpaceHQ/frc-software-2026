//COMMANDJOYSTICK DOESN'T HAVE ENOUGH TRIGGERS TO INTERFACE CONTROLLER IFC

package frc.Interfaces;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class JoystickControllerIfc implements ControllerIfc {

  private CommandJoystick joystick;

  public enum ExtendedButtonType {
    Button3(3),
    Button4(4),
    Button5(5);
    public final int value;

    ExtendedButtonType(int value) {
      this.value = value;
    }
  }
    
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

        return getButton3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public double  controlMotorSpeed(){

        return joystick.getZ();
    }

    private boolean getButton3() {
        return joystick.getHID().getRawButton(ExtendedButtonType.Button3.value);
    }

    private Trigger getButton3(EventLoop loop) {
        return joystick.button(ExtendedButtonType.Button3.value, loop);
    }
}