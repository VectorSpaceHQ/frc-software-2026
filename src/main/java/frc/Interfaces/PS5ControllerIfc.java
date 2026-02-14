package frc.Interfaces;

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

    public double getTwist(){

        return exampleJoystick.getRightX();
    }
    
    public double controlMotorSpeed(){
        
        return exampleJoystick.getL2Axis();
    }

    public Trigger runShooter(){

        return exampleJoystick.circle();
    }

    public Trigger runIntake(){

        return exampleJoystick.cross();
    }
    public Trigger stopIntake(){

        return exampleJoystick.triangle(); 
    } 

}

