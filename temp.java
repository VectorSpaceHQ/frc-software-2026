// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //private final RobotContainer m_robotContainer;

  //private static final CANBus kCANBus = new CANBus("canivore");

  private final TalonFX m_leftLeader = new TalonFX(19);
  //private final TalonFX m_rightLeader = new TalonFX(1, kCANBus);
  private final TalonFX m_leftFollower = new TalonFX(20);
  //private final TalonFX m_rightFollower = new TalonFX(3, kCANBus);

   private final DutyCycleOut m_leftOut = new DutyCycleOut(0);
  // private final DutyCycleOut m_rightOut = new DutyCycleOut(0);
    
    /* private final CANBus canbus = new CANBus("canivore");

    */

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    

   //private final XboxController m_driverJoy = new XboxController(0);

   
      // start with factory-default configs
      var currentConfigs = new MotorOutputConfigs();

      // The left motor is CCW+
      currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
      m_leftLeader.getConfigurator().apply(currentConfigs);

      // The right motor is CW+
      currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
      //m_rightLeader.getConfigurator().apply(currentConfigs);

      // Ensure our followers are following their respective leader
      m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), MotorAlignmentValue.Opposed));
      //m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
      //m_robotContainer = new RobotContainer();
      /* 
      // retrieve joystick inputs
      //var fwd = 0.1;
      var rot = 0.1;

      // modify control requests
      m_leftOut.Output = fwd + rot;
      m_rightOut.Output = fwd - rot;

      // send control requests
      m_leftLeader.setControl(m_leftOut);
      m_rightLeader.setControl(m_rightOut); */

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

      // send control requests
      //m_leftLeader.setControl();
      //m_rightLeader.setControl(1);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // retrieve joystick inputs
      var fwd = .06;//-m_driverJoy.getLeftY();
      var rot = .06;///m_driverJoy.getRightX();

      // modify control requests
      m_leftOut.Output = fwd + rot;
      //m_rightOut.Output = fwd - rot;

      // send control requests
      m_leftLeader.setControl(m_leftOut);
      //m_rightLeader.setControl(m_rightOut);
      TalonFX.getVelocity();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
