package frc.robot.subsystems.shooter;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.components.motor.MotorIOKraken;
import frc.robot.components.motor.MotorIOSparkMax;
import frc.robot.configuration.Constants.ShooterConstants;
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

    final SlewRateLimiter mainRpmSlew = new SlewRateLimiter(200.0);
    final SlewRateLimiter englishRpmSlew = new SlewRateLimiter(500.0);
    final SlewRateLimiter intakeRpmSlew = new SlewRateLimiter(300.0);

    final SwerveSubsystem mSwerveSubsystem = null;
    final double g = 9.8;

    private final VisionSubsystem visionSubsystem = null;
    final Translation3d hubPose = new Translation3d();

    MotorIOKraken main_motor;


    public ShooterSubsystem(ShooterSubsysConfig config) {
        this.shooterConfig = config;
        //this.visionSubsystem = visionSubsystem;
        //this.hubPose = this.visionSubsystem.getTargetHubCenter();
        shooterConfigPresent = shooterConfig.getIsPresent();
        shooterStatus = false;
        lastShooterStatus = false;
        runningSysId = ShooterConstants.RUNNING_SYS_ID;
        

        if (shooterConfigPresent) {
            // English Flywheel Mechanism
            english_PID = new PID(
                    "English",
                    new MotorIOKraken(this.shooterConfig.getShooterEnglishId()),
                    ShooterConstants.ENGLISH_MAX_RPM,
                    ShooterConstants.MAX_VOLTAGE,
                    ShooterConstants.GEAR_RATIO,
                    ShooterConstants.ENGLISH_kS,
                    ShooterConstants.ENGLISH_kP,
                    ShooterConstants.ENGLISH_kI,
                    ShooterConstants.ENGLISH_kD,
                    ShooterConstants.ENGLISH_kV,
                    ShooterConstants.ENGLISH_kA);

            // Main Flywheel Mechanism
            main_motor = new MotorIOKraken(this.shooterConfig.getShooterMainId());
            main_PID = new PID(
                    "Main",
                    main_motor,
                    ShooterConstants.MAIN_MAX_RPM,
                    ShooterConstants.MAX_VOLTAGE,
                    ShooterConstants.GEAR_RATIO,
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
                    ShooterConstants.GEAR_RATIO,
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
            SmartDashboard.putNumber("Main RPM", main_motor.motor.getVelocity().getValueAsDouble());
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

    public void setMainVelocity(double target_vel){
        // Set the main wheel's angular velocity in ft/s
        // use an angular acceleration limit to avoid motor damage
        float d_main = 6; // in
        double target_rpm = (target_vel * 60 * 12) / (Math.PI * d_main);
        //mainRpmSlew.reset(main_PID.getM_realRPM());
        double rampedMainRPM = mainRpmSlew.calculate(target_rpm);
        main_PID.setM_RPM(rampedMainRPM);
    }
    
    public void setEnglishVelocity(double target_vel){
        // Set the english wheel's angular velocity in ft/s
        float d_english = 4; // in
        double target_rpm = (target_vel * 60 * 12) / (Math.PI * d_english);
        //englishRpmSlew.reset(english_PID.getM_realRPM());
        double rampedEnglishRPM = englishRpmSlew.calculate(target_rpm);
        english_PID.setM_RPM(rampedEnglishRPM);
    }

    public double getMainVelocity(){
        // get angular velocity of main wheel in ft/s
        float d_main = 6; // in
        double gear_ratio = 1.5;
        double motor_rpm = main_motor.motor.getVelocity().getValueAsDouble();
        double wheel_rpm = motor_rpm * gear_ratio;
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // We need actual rpm, not PID calculated rpm
        //double v_main = (main_PID.getM_realRPM() * Math.PI * d_main) / (60 * 12); 
        double v_main = (wheel_rpm * Math.PI * d_main) / (12); 
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        return v_main;
    }
    public double getEnglishVelocity(){
        // get angular velocity of main wheel in ft/s
        float d_english = 4; // in
        double gear_ratio = 1.5;
        double motor_rpm = english_PID.getM_realRPM();
        double wheel_rpm = motor_rpm * gear_ratio;
        double v_english = (wheel_rpm * Math.PI * d_english) / (12); 
        return v_english;
    }

    public void setShooterVelocity(double main_vel, double english_vel){
        setMainVelocity(main_vel);
        setEnglishVelocity(english_vel);
    }

    public double getLaunchAngle(){
        // given current main wheel velocity and english wheel velocity
        // return launch angle in degrees
        // angle is set by shooter design
        double launchAngle = 75; // deg
        //double launchAngle = (5 * ShooterConstants.MAIN_MAX_RPM * main_vel) / (ShooterConstants.ENGLISH_MAX_RPM * english_vel);
        return launchAngle;
    }

    public double getLaunchVelocity(double main_vel, double english_vel){
        // given current main wheel velocity and english wheel velocity
        // return launch velocity in ft/s
        // This is an empirical formula, current formula is a guess, needs tuning
        // The assumption is that the ball velocity is the average of the wheel velocities.
        // It is also assumed that there is a loss factor in transferrign momentum from wheel to ball
        // and that the english wheel transfers less than the main

        //double launchVelocity = (5 * ShooterConstants.MAIN_MAX_RPM * main_vel) / (ShooterConstants.ENGLISH_MAX_RPM * english_vel);

        double L_main = 0.95; // loss factor
        double L_english = 0.5; // loss factor
        double launchVelocity = (L_english * english_vel + L_main * main_vel) / 2;

        return launchVelocity;
    }

    public void setLaunchAngle(Double launch_alpha){
        // given a target launch angle
        // adjust main and english wheel velocities
        // this is an empirical formula
    }

/*     This is without drag. Can be removed.
        public double calcLaunchVelocity(double launch_alpha, Pose2d pose){
        // Return launch velocity vector as a function of pose and current launch angle
        double launchAngle = launch_alpha;
        double translationX = pose.getX();
        double translationY = pose.getY();
        double initialVelocity;
        initialVelocity = (translationX * Math.sqrt(g)) 
                            / Math.sqrt(2 * Math.cos(launchAngle) * Math.cos(launchAngle) * (translationX * Math.tan(launchAngle) - translationY));
        return initialVelocity;
    }

    public double calcLaunchAngle(Double launchVelocity, Pose2d pose){
        // Return launch angle as a function of pose and current launch velocity
        
        // Distances from hub to robot. Need to translate shooter pose to account for orientation and placement on robot
        double translationX = pose.getX();
        double translationY = pose.getY();
        double initialVelocity = launchVelocity;
        double launchAngle;
        launchAngle = Math.atan((initialVelocity * initialVelocity + Math.sqrt(initialVelocity * initialVelocity * initialVelocity * initialVelocity - g * (g * translationX * translationX + 2 * initialVelocity * initialVelocity * translationY))) 
                                / (g * translationX));
        return launchAngle;
    } */

    public double calcTrajectory(double launch_v, double launch_alpha){
        // Projectile motion calculation including air resistance
        // Returns distance at which ball enters center of hub,
        //units are in, lb, and s

        double dist; // distance at which ball would enter hub
        double rho = 0.075 / 1728; // lb/in^3 0.075/12^3
        double C = 0.6;
        double A = Math.PI * (5.9 / 2) * (5.9 / 2); // in^2
        double D = (rho * C * A) /2;
        double m = 0.5; // lb
        double g = 32.2 * 12; // in/s2
        double y_hub = 60; // in

        double dt = 0.1; // sec

        double vx = launch_v * Math.cos(launch_alpha);
        double x = 0;
        double vy = launch_v * Math.sin(launch_alpha);
        double y = 0;
        double t = 0;
        double v = Math.sqrt(vx * vx + vy * vy);
        double vy_old = 0;
        double y_old = 0;
        double vx_old = 0;
        double x_old = 0;
        double ax = 0;
        double ay = 0;
        double s = 0; // acceleration due to spin

        if (launch_v < 1){
            System.out.println("wheels too slow");
            return 0;
        }

        for (int i=0; i<50; i++)
        {
            t = t + dt;
            v = Math.sqrt(vx * vx + vy * vy);

            ax = -D / m * v * vx;
            ay = -D / m * v * vy - g + s;
            vx = vx_old + ax * dt;
            x = x_old + vx * dt + (0.5 * ax * dt * dt);
            vy = vy_old + ay * dt;
            y = y_old + vy * dt + (0.5 * ay * dt * dt);

            vx_old = vx;
            x_old = x;
            vy_old = vy;
            y_old = y;

            // break condition
            // if ball new y less than old y, falling
            // and if y <= y_hub
            if (y < y_old && y <= y_hub){
                System.out.println("Solver converged");
                return x;
            }
        }
        System.out.println("Solver failed to converge");
        return 99; // failed to converge
    }

    public void solver(){
        SmartDashboard.putBoolean("solver status", true);

        // get new pose
        //Pose2d robotPose = mSwerveSubsystem.getEstimatedPose();
        //Translation3d hubPose = this.visionSubsystem.getTargetHubCenter();
        double launchAngle;
        double launchVelocity;
        double error;
        double tolerance;
        double K;
        // translations taken from camera in Constants
       // Pose2d shooterPose = robotPose.transformBy(new Transform2d(new Translation2d(-0.1778, -0.3302), 
        //                                            new Rotation2d(Math.toRadians(-90))))
        //double dist_to_hub = this.visionSubsystem.getDistanceToHub(robotPose, hubPose);
        launchAngle = getLaunchAngle();
        launchVelocity = getLaunchVelocity(getMainVelocity(), getEnglishVelocity());
        // return error between calculated shot distance and current robot distance
        // iterate until convergence. Nmax = 20.
        double launch_distance = calcTrajectory(launchVelocity, launchAngle);
        double dist_to_hub = 96;
        // ---------
        error = launch_distance - dist_to_hub; // double check these units! Both inches?
        //
        tolerance = 6; // in
        K = 10; // rpm/in
        SmartDashboard.putString("Target", "acquiring");

        if (error > tolerance){
            //decrease wheel speeds
            setMainVelocity(getMainVelocity() - K * error);
            setEnglishVelocity(getEnglishVelocity() - K * error);
        }
        else if(error < 0 && Math.abs(error) > tolerance){
            // increase wheel speeds
            System.out.print("increase wheel speed: ");
            System.out.print(getMainVelocity());
            System.out.print(", by ");
            System.out.println(K * error);
            setMainVelocity(getMainVelocity() - K * error);
            setEnglishVelocity(getEnglishVelocity() - K * error);
        }
        else{
            System.out.print("Target locked at RPM = ");
            System.out.println(getMainVelocity());
            SmartDashboard.putString("Target", "locked");
        }

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
            this.startShooter();
            solver();
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