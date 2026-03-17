package frc.robot.subsystems.shooter;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.pathplanner.lib.util.FlippingUtil;

import frc.robot.subsystems.drive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveDrive;
import frc.robot.subsystems.drive.SwerveSubsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.components.motor.MotorIOKraken;
import frc.robot.components.motor.MotorIOSparkMax;
import frc.robot.configuration.Constants.ShooterConstants;
import frc.robot.configuration.Constants;
import frc.robot.configuration.Constants.OperatorConstants;
import frc.robot.configuration.Constants.SysIdEnums;
import frc.robot.configuration.Constants.SysIdEnums.SysIdTarget;
import frc.robot.configuration.configs.ShooterSubsysConfig;
import frc.robot.components.control.PID;
import frc.robot.components.control.SysId;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ShooterSubsystem extends SubsystemBase {

    private ShooterSubsysConfig shooterConfig = null;

    private PID english_PID = null;
    private PID main_PID = null;
    private PID feeder_PID = null;
    private SysIdRoutine englishSysId = null;
    private SysIdRoutine mainSysId = null;
    private SysIdRoutine feederSysId = null;

    private SysIdTarget sysIdTarget = SysIdTarget.MAIN;

    // private final double velocity_MOTOR =
    // Units.rotationsPerMinuteToRadiansPerSecond(509.3); // 53.33 rads/s
    // https://www.reca.lc/motors

    // private final ControllerIfc m_driverController;
    // private final ControllerIfc m_operatorController;

    private boolean shooterConfigPresent;
    private boolean shooterStatus;
    private boolean lastShooterStatus;
    private boolean runningSysId;

    final SlewRateLimiter mainRpmSlew;
    final SlewRateLimiter englishRpmSlew;
    final SlewRateLimiter intakeRpmSlew;

    final SwerveSubsystem mSwerveSubsystem;
    private Pose2d goalPosition = new Pose2d(4.620419, 4.034631, new Rotation2d());
    public ShooterSubsystem(ShooterSubsysConfig config, SwerveSubsystem swerveSubsystem) {
        this.mSwerveSubsystem = swerveSubsystem;
        this.shooterConfig = config;
        shooterConfigPresent = shooterConfig.getIsPresent();
        shooterStatus = false;
        lastShooterStatus = false;
        runningSysId = ShooterConstants.RUNNING_SYS_ID;

        mainRpmSlew = new SlewRateLimiter(400.0);
        englishRpmSlew = new SlewRateLimiter(1000.0);
        intakeRpmSlew = new SlewRateLimiter(500.0);        
        // mainRpmSlew = new SlewRateLimiter(99900.0);
        // englishRpmSlew = new SlewRateLimiter(99000.0);
        // intakeRpmSlew = new SlewRateLimiter(99900.0);           

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(Commands.runOnce(() -> {
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    goalPosition = FlippingUtil.flipFieldPose(goalPosition);
                }
            }
        }));        

        if (shooterConfigPresent) {
            // English Flywheel Mechanism
            english_PID = new PID(
                    "English",
                    new MotorIOKraken(this.shooterConfig.getShooterEnglishId()),
                    ShooterConstants.ENGLISH_MAX_RPM,
                    ShooterConstants.MAX_VOLTAGE,
                    0.75,
                    ShooterConstants.ENGLISH_kS,
                    ShooterConstants.ENGLISH_kP,
                    ShooterConstants.ENGLISH_kI,
                    ShooterConstants.ENGLISH_kD,
                    ShooterConstants.ENGLISH_kV,
                    ShooterConstants.ENGLISH_kA);

            // Main Flywheel Mechanism
            //main_motor = new MotorIOKraken(this.)
            main_PID = new PID(
                    "Main",
                    new MotorIOKraken(this.shooterConfig.getShooterMainId()),
                    ShooterConstants.MAIN_MAX_RPM,
                    ShooterConstants.MAX_VOLTAGE,
                    1,
                    ShooterConstants.MAIN_kS,
                    ShooterConstants.MAIN_kP,
                    ShooterConstants.MAIN_kI,
                    ShooterConstants.MAIN_kD,
                    ShooterConstants.MAIN_kV,
                    ShooterConstants.MAIN_kA);

            // Feeder Flywheel Mechanism
            feeder_PID = new PID(
                    "Feeder",
                    new MotorIOSparkMax(this.shooterConfig.getFeederId(), ShooterConstants.FEEDER_CURRENT_LIMIT),
                    ShooterConstants.FEEDER_MAX_RPM,
                    ShooterConstants.MAX_VOLTAGE,
                    0.67,
                    ShooterConstants.FEEDER_kS,
                    ShooterConstants.FEEDER_kP,
                    ShooterConstants.FEEDER_kI,
                    ShooterConstants.FEEDER_kD,
                    ShooterConstants.FEEDER_kV,
                    ShooterConstants.FEEDER_kA);

            englishSysId = SysId.createRoutine(this, english_PID, "English");
            mainSysId = SysId.createRoutine(this, main_PID, "Main");
            feederSysId = SysId.createRoutine(this, feeder_PID, "Feeder");
            // t_motorInputs = new MotorIOInputs();
            // b_motorInputs = new MotorIOInputs();

            SmartDashboard.putData("Shooter/English PID", english_PID);
            SmartDashboard.putData("Shooter/Main PID", main_PID);
            SmartDashboard.putData("Shooter/Feeder PID", feeder_PID);
        }

        SmartDashboard.putBoolean("Shooter Present", shooterConfig.getIsPresent());
    }

    // Just in case
    public boolean startShooter() {
        shooterStatus = true;
        return shooterStatus;
    }

    // Just in case
    public boolean stopShooter() {
        shooterStatus = false;
        return shooterStatus;
    }

    public boolean toggleShooter() {
        shooterStatus = !shooterStatus;
        return shooterStatus;
    }

    public void setCloseShot() {
        english_PID.setM_RPM(-1250);
        main_PID.setM_RPM(-750);
        feeder_PID.setM_RPM(1700);
        getMainVelocity();
        getEnglishVelocity();
    }

    public void setFarShot() {
        english_PID.setM_RPM(-2750);
        main_PID.setM_RPM(-1750);
        feeder_PID.setM_RPM(1700);
        getMainVelocity();
        getEnglishVelocity();        
    }

    public void zeroRPM() {
        english_PID.setM_RPM(0);
        main_PID.setM_RPM(0);
        feeder_PID.setM_RPM(0);
    }

    // public void setAutoShot() {
    //     english_PID.setM_RPM(-2750);
    //     main_PID.setM_RPM(-1750);
    //     feeder_PID.setM_RPM(-1700);
    // }    

    // private Command setShooter = run(()-> RPM.of(rpm)).ignoringDisable(true);
    // private Command stopShooter = setVoltage(Volts.of(0)).ignoringDisable(true);

    public void setSysIdTarget(SysIdTarget target) {
        sysIdTarget = target;
    }

    public SysIdTarget getSysIdTarget() {
        return sysIdTarget;
    }

    // Place status values here
    public boolean getShooterStatus() {
        return shooterStatus;
    }

    public boolean getLastShooterStatus() {
        return lastShooterStatus;
    }

    public SysIdRoutine getActiveSysIdRoutine() {
        switch (sysIdTarget) {
            case ENGLISH:
                return englishSysId;
            case MAIN:
                return mainSysId;
            case FEEDER:
                return feederSysId;
            default:
                return feederSysId;
        }
    }

    public boolean atSpeed() {
        return english_PID.atSpeed(ShooterConstants.SHOOTER_SPEED_TOLERANCE_RPM)
                && main_PID.atSpeed(ShooterConstants.SHOOTER_SPEED_TOLERANCE_RPM)
                && feeder_PID.atSpeed(ShooterConstants.SHOOTER_SPEED_TOLERANCE_RPM);
    }

    public void setMainVelocity(double target_wheel_vel){
        // Set the main wheel's angular velocity in ft/s
        // use an angular acceleration limit to avoid motor damage
        float d_main = 6; // in
        double gear_ratio = 1.0;
        double target_motor_rpm = (target_wheel_vel * 60 * 12) / (Math.PI * d_main * gear_ratio);
        //mainRpmSlew.reset(main_PID.getM_realRPM());
        double rampedMainRPM = mainRpmSlew.calculate(target_motor_rpm);
        main_PID.setM_RPM(rampedMainRPM);  
        SmartDashboard.putNumber("target main rpm", rampedMainRPM);
    }
    
    public void setEnglishVelocity(double target_wheel_vel){
        // Set the english wheel's angular velocity in ft/s
        float d_english = 4; // in
        double gear_ratio = 0.75;
        double target_motor_rpm = (target_wheel_vel * 60 * 12) / (Math.PI * d_english * gear_ratio);
        //englishRpmSlew.reset(english_PID.getM_realRPM());
        double rampedEnglishRPM = englishRpmSlew.calculate(target_motor_rpm);
        english_PID.setM_RPM(rampedEnglishRPM);
        SmartDashboard.putNumber("target english rpm", rampedEnglishRPM);
    }

    public double getMainVelocity(){
        // get angular velocity of main wheel in ft/s
        float d_main = 6; // in
        double gear_ratio = 1.0;
        // double motor_rpm = main_PID.getM_realRPM();
        double motor_rpm = main_PID.getM_RPM(); //setter NOT GETTER values for testing
        SmartDashboard.putNumber("mainRealRPM", motor_rpm);
        double wheel_rpm = motor_rpm * gear_ratio; // Note: gear ratio is set in shooter constants and passed into the 
        // PID constructor, which is being used to calculate rpm separately. You would either need to remove its implementation
        // from the PID class (which after thought would be better since it was used as a temporary measure) or remove its implementation
        // here since you would be accounting for gear ratio twice, not once.
        // Additional note: Pretty sure you would divide the motor rpm by the gear ratio to get the wheel rpm, not multiply.

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // We need actual rpm, not PID calculated rpm
        //double v_main = (main_PID.getM_realRPM() * Math.PI * d_main) / (60 * 12); 
        double v_main = (wheel_rpm * Math.PI * d_main) / (12 * 60); 
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        SmartDashboard.putNumber("mainRealVelocity", v_main); 
        return v_main;
    }
    public double getEnglishVelocity(){
        // get angular velocity of main wheel in ft/s
        float d_english = 4; // in
        double gear_ratio = 0.75;
        // double motor_rpm = english_PID.getM_realRPM();
        double motor_rpm = english_PID.getM_RPM(); //setter NOT GETTER values for testing
        SmartDashboard.putNumber("englishRealRPM", motor_rpm);
        double wheel_rpm = motor_rpm * gear_ratio;
        double v_english = (wheel_rpm * Math.PI * d_english) / (12 * 60);
        SmartDashboard.putNumber("englishRealVelocity", v_english);        
        return v_english;
    }

    public void setShooterVelocity(double main_vel, double english_vel){
        setMainVelocity(main_vel);
        setEnglishVelocity(english_vel);
    }

    public double getLaunchAngle(double main_vel, double english_vel){
        // given current main wheel velocity and english wheel velocity
        // return launch angle
        // This is an empirical formula, current formula is a guess, needs tuning
        double launchAngle = (5 * ShooterConstants.MAIN_MAX_RPM * main_vel) / (ShooterConstants.ENGLISH_MAX_RPM * english_vel);
        return launchAngle;
    }

    public double getLaunchVelocity(){
        // given current main wheel velocity and english wheel velocity
        // return launch velocity in ft/s

        // This is an empirical formula, current formula is a guess, needs tuning
        // The assumption is that the ball velocity is the average of the wheel velocities.
        // It is also assumed that there is a loss factor in transferring momentum from wheel to ball
        // and that the english wheel transfers less than the main

        //double launchVelocity = (5 * ShooterConstants.MAIN_MAX_RPM * main_vel) / (ShooterConstants.ENGLISH_MAX_RPM * english_vel);
        double english_vel = getEnglishVelocity();
        double main_vel = getMainVelocity();
        double L_main = 0.97; // loss factor
        double L_english = 0.85; // loss factor
        double launchVelocity = (L_english * english_vel + L_main * main_vel) / 2;
        SmartDashboard.putNumber("launchVelocity", launchVelocity);
        return launchVelocity;
    }

    public double calcTrajectory(double launch_v, double launch_alpha){
        // Projectile motion calculation including air resistance
        // Returns distance at which ball enters center of hub,
        //units are ft, lb, and s
        // returns distance in meters
        double dist_in_meters; // distance at which ball would enter hub
        double rho = 0.075; // lb/ft^3
        double C = 0.6;
        double A = Math.PI * Math.pow(5.9 / 24, 2); // ft^2
        double D = (rho * C * A) /2;
        D = 0.004271829698103934;
        double m = 0.5; // lb
        double g = 32.2; // ft/s2
        double y_hub = 60 / 12; // ft
        double dt = 0.001; // sec

        double vx = launch_v * Math.cos(Math.toRadians(launch_alpha));
        double x = 0;
        double vy = launch_v * Math.sin(Math.toRadians(launch_alpha));
        double y = 0;
        double t = 0;
        double v = Math.sqrt(vx * vx + vy * vy);
        double vy_old = vy;
        double y_old = 0;
        double vx_old = vx;
        double x_old = 0;
        double ax = 0;
        double ay = 0;
        double s = 0; // acceleration due to spin


        for (int i=0; i<5000; i++)
        {
            t = t + dt;
            v = Math.sqrt(vx * vx + vy * vy);

            ax = -D / m * v * vx;
            ay = (-D / m) * v * vy - g + s;
            vx = vx_old + ax * dt;
            x = x_old + vx * dt + (0.5 * ax * dt * dt);
            vy = vy_old + ay * dt;
            y = y_old + vy * dt + (0.5 * ay * dt * dt);



            // break condition
            // if ball new y less than old y, falling
            // and if y <= y_hub
            if (y < y_old && y <= y_hub){
                SmartDashboard.putBoolean("Solver converged", true);
                SmartDashboard.putNumber("SolverY", y);
               
                dist_in_meters = x * 0.3048;
                SmartDashboard.putNumber("solverX", dist_in_meters);
                return dist_in_meters; // return in meters
            }

            if(y >= y_old){
                SmartDashboard.putNumber("apogee", y);
            }

            vx_old = vx;
            x_old = x;
            vy_old = vy;
            y_old = y;
        }
        SmartDashboard.putBoolean("Solver converged", false);
        return 3; // failed to converge
    }

    public void solver(){
        // get new pose
        shooterStatus = true;
        Pose2d robotPose = mSwerveSubsystem.getEstimatedPose();
        double launchAngle;
        double launchVelocity;
        double error;
        double tolerance;
        double K;
        double targetMainWheelVelocity;
        double targetEnglishWheelVelocity;
        // translations taken from camera in Constants
        Pose2d shooterPose = robotPose.transformBy(new Transform2d(new Translation2d(-0.1778, 0.3302), 
                                                    new Rotation2d(Math.toRadians(90))));
        double distanceToHub = new Translation2d(goalPosition.getX(), goalPosition.getY()).getDistance(new Translation2d(shooterPose.getX(), shooterPose.getY()));
        launchAngle = 75; //degrees, temporary value for comp 1
        launchVelocity = getLaunchVelocity(); //LOSS NEEDS TUNING
        double launch_distance = calcTrajectory(launchVelocity, launchAngle); //D value needs tuning
        error = launch_distance - distanceToHub;
        tolerance = 0.1524; // meters
        K = 30.48; // rpm/m, NEEDS TUNING
        SmartDashboard.putNumber("launch distance", launch_distance);
        SmartDashboard.putNumber("distance to hub", distanceToHub);
        SmartDashboard.putNumber("Shooter Error", error);

        if(Math.abs(error) < tolerance){
            SmartDashboard.putString("Target", "locked");
            targetMainWheelVelocity = getMainVelocity();
            targetEnglishWheelVelocity = getEnglishVelocity();
        } 
        else {
            targetMainWheelVelocity = getMainVelocity() - K * error;
            targetEnglishWheelVelocity = getEnglishVelocity() - K * error;
            setMainVelocity(targetMainWheelVelocity);
            setEnglishVelocity(targetEnglishWheelVelocity);
            if (error > tolerance){
                //decrease wheel speeds
                SmartDashboard.putString("Target", "too close");
            }
            else if(error < 0 && Math.abs(error) > tolerance){
                // increase wheel speeds
                SmartDashboard.putString("Target", "too far");
            }
        }
        SmartDashboard.putNumber("target english velocity", targetEnglishWheelVelocity);
        SmartDashboard.putNumber("target main velocity", targetMainWheelVelocity);

    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop

        english_PID.m_updateInputs();
        main_PID.m_updateInputs();
        feeder_PID.m_updateInputs();

        //english_PID.processInputs("Shooter/English");
        //main_PID.processInputs("Shooter/Main");
        //feeder_PID.processInputs("Shooter/Feeder");

        if (runningSysId == false & shooterConfig.getIsPresent()) {
            english_PID.PIDPeriodic(shooterStatus && !lastShooterStatus, shooterStatus);
            main_PID.PIDPeriodic(shooterStatus && !lastShooterStatus, shooterStatus);
            feeder_PID.PIDPeriodic(shooterStatus && !lastShooterStatus, shooterStatus);
        }

        /*
         * This line changes the shooter status of last to the shooter status of current
         * (so lastShooterStatus turns to true when shooterStatus is true).
         * But lastShooterStatus is initialized to false, so when lastShooterStatus
         * equals
         * shooterStatus initially, !lastShooterStatus does not equal true, meaning that
         * the PID does not reset.
         * * Furthermore, this happens every initialization of the shooter becoming
         * true.
         * If shooterStatus is false, !lastShooterStatus is true; if shooterStatus is
         * true, !lastShooterStatus remains true and resets the PID before becoming
         * false again. This should fix the problem that the integral term is building
         * error before shooter turns on, which needs to be reset (not periodically but
         * after every time the shooter turns on)
         */

        lastShooterStatus = shooterStatus;
        SmartDashboard.putBoolean("shooterStatus", shooterStatus);

    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return getActiveSysIdRoutine().quasistatic(dir)
                .beforeStarting(() -> runningSysId = true)
                .finallyDo(() -> runningSysId = false);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return getActiveSysIdRoutine().dynamic(dir)
                .beforeStarting(() -> runningSysId = true)
                .finallyDo(() -> runningSysId = false);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Shooter init sendable called");
        builder.setSmartDashboardType("Shooter Controller");
        builder.addBooleanProperty("Shooter Status", this::getShooterStatus, null);
        builder.addBooleanProperty("Last Shooter Status", this::getLastShooterStatus, null);
        builder.addBooleanProperty("At speed", this::atSpeed, null);
        super.initSendable(builder);
        english_PID.initSendable(builder);
        main_PID.initSendable(builder);
        feeder_PID.initSendable(builder);

    }

}