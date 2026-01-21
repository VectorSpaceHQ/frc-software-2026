// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants.ControllerEnum;
//import edu.wpi.first.wpilibj.Joystick;


public class ControllerIfc extends SubsystemBase{
  //Instantiates our variables for the controller inputs
  double swerveX;
  double swerveY;
  double rotate;
  public Trigger shoot;
  private ControllerIfc ctlIfc; 
  public ControllerIfc() {}

  public ControllerIfc(ControllerEnum controllerType, int port) {
    //create a controller interface based on the controller type connected. look at the other controllerIfcs to see specifics
   switch (controllerType) {
    case XBOX:
      ctlIfc = new XboxControllerIfc(port);  
        break;
    case FLIGHTSTICK:
      ctlIfc = new JoystickControllerIfc(port);
        break;        
    case PS4:
      ctlIfc = new PS4ControllerIfc(port);  
        break;
    case PS5:
      ctlIfc = new PS4ControllerIfc(port);  
        break;
    case Auto:
      ctlIfc = new AutoControllerIfc(port);        
      
    default:
        break;
   }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command shootCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          //run a motor or do something
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean commandShooter() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  public double getX() {
    // Query some boolean state, such as a digital sensor.
    
    return swerveX;
  }

    public double getY() {
    // Query some boolean state, such as a digital sensor.
    
    return swerveY;
  }

    public double getTwist() {
    // Query some boolean state, such as a digital sensor.
    
    return rotate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command ControllerCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ControllerCommand'");
  }
}
