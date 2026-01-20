// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants.Controller;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class ControllerSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  double swerveX;
  double swerveY;
  double rotate;
  boolean shoot;
  public ControllerSubsystem() {}

  public ControllerSubsystem(Controller controllerType) {
   switch (controllerType) {
    case XBOX:
        
        break;
    case FLIGHTSTICK:
      Joystick exampleJoystick = new Joystick(0);
      swerveX = exampleJoystick.getX();
      swerveY = exampleJoystick.getY();
      rotate = exampleJoystick.getTwist();
      shoot = exampleJoystick.getTop();
        break;        
    case PS4:
        
        // break;
    case PS5:
        
        // break;        
      
    default:
        break;
   }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean commandShooter() {
    // Query some boolean state, such as a digital sensor.
    if(shoot){

    }
    return false;
  }
  public boolean yButtonPressed() {
    // Query some boolean state, such as a digital sensor.
    
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
