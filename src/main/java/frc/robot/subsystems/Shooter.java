// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.util.Units;
// import frc.Interfaces.ControllerIfc;
// import frc.Interfaces.XboxControllerIfc;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.components.motor.MotorIO;
// import frc.robot.components.motor.MotorIOKraken;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// public class Shooter extends SubsystemBase {
//   private final MotorIO t_motor;
//   private final MotorIO b_motor;
//   private final double MAX_RPM = 6000;
//   private final ControllerIfc m_driverController;
//   private final ControllerIfc m_operatorController;
//   private boolean shooterstatus;
//   private SimpleMotorFeedforward feedforward;

//     double  t_motorspeed;
//     double  b_motorspeed; 
    
//     double t_RPM;
//     double b_RPM;

//     double getY = 1;
//     double t_targetRPM = (getY * MAX_RPM);
//     double t_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(t_targetRPM);
//     double t_volts = feedforward.calculate(t_targetRadsPerSec);


//   /** Creates a new Shooter. */
//   public Shooter() {

//     m_driverController = new XboxControllerIfc(OperatorConstants.controllerPort1);
//     m_operatorController = new XboxControllerIfc(OperatorConstants.controllerPort2);
//     t_motor = new MotorIOKraken(2);
//     b_motor = new MotorIOKraken(0);
//     feedforward = new SimpleMotorFeedforward(0.2, 12/509.3);

//   }

//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command RunShooterCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return this.runOnce(
//         () -> {
//           /* one-time action goes here */
//           t_motor.setVoltage(1);
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean shooterstatus() {
//     // Query some boolean state, such as a digital sensor.
    
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     getY = m_driverController.getY();
//     t_targetRPM = (getY * MAX_RPM);
//     t_targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(t_targetRPM);
//     t_volts = feedforward.calculate(t_targetRadsPerSec);
//     t_motor.setVoltage(t_motorspeed);
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }
