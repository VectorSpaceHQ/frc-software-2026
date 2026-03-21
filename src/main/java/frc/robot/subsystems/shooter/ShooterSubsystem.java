package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
//spark max imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.components.control.PID;
import frc.robot.components.control.SysId;
import frc.robot.components.motor.MotorIOKraken;
import frc.robot.configuration.Constants.ShooterConstants;
import frc.robot.configuration.Constants.SysIdEnums.SysIdTarget;
import frc.robot.configuration.configs.ShooterSubsysConfig;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    private ShooterSubsysConfig shooterConfig = null;

    // removed PIDS


    // spark max feeder motor vars
    private SparkMax feederWheel;
    private SparkMaxConfig feederConfig;
    private SparkClosedLoopController feederClosedLoopController;
    private RelativeEncoder feederRelativeEncoder;
    private double feederRPM = 0;

    //Main Kraken motor vars
    private TalonFX mainWheel;
    private TalonFXConfiguration mainConfig;
    private final VelocityVoltage mainVelocityRequest = new VelocityVoltage(0);
    private double mainRPMGoal = 0;

    //English Kraken motor vars
    private TalonFX englishWheel;
    private TalonFXConfiguration englishConfig;
    private final VelocityVoltage englishVelocityRequest = new VelocityVoltage(0);
    private double englishRPMGoal = 0;

    private boolean shooterConfigPresent;
    private boolean shooterStatus;
    private boolean lastShooterStatus;

    final SlewRateLimiter mainRpmSlew;
    final SlewRateLimiter englishRpmSlew;
    final SlewRateLimiter feederRpmSlew;

    final SwerveSubsystem mSwerveSubsystem;
    private Pose2d goalPosition = ShooterConstants.blueHubCenter;
    private double solverMainVelocity = 0;
    private double solverEnglishVelocity = 0;

    public ShooterSubsystem(ShooterSubsysConfig config, SwerveSubsystem swerveSubsystem) {
        this.mSwerveSubsystem = swerveSubsystem;
        this.shooterConfig = config;
        shooterConfigPresent = shooterConfig.getIsPresent();
        shooterStatus = false;
        lastShooterStatus = false;

        mainRpmSlew = new SlewRateLimiter(1000.0); //rpm/s
        englishRpmSlew = new SlewRateLimiter(1000.0); //rpm/s
        feederRpmSlew = new SlewRateLimiter(1000); //rpm/s

        if (shooterConfigPresent) {
            //if we have a shooter config, configure the motors
            configureShooterMotors();
            configureFeederMotor();
        }

        SmartDashboard.putBoolean("Shooter Present", shooterConfig.getIsPresent());
    }

    public void configureShooterMotors() {
        // English Flywheel Mechanism
        englishWheel = new TalonFX(this.shooterConfig.getShooterEnglishId());
        englishConfig = new TalonFXConfiguration();
        englishConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // PID gains for velocity control (slot 0)
        // COULD SET CONTANTS WHEN FIGURE OUT GOOD VALUES OR SMARTDASHBOARD
        englishConfig.Slot0.kP = ShooterConstants.ENGLISH_kP;   // volts per rotation/sec error
        englishConfig.Slot0.kI = ShooterConstants.ENGLISH_kI;
        englishConfig.Slot0.kD = ShooterConstants.ENGLISH_kD;  // volts per rotation/sec²
        englishConfig.Slot0.kS = ShooterConstants.ENGLISH_kS;   // static friction feedforward (volts)
        englishConfig.Slot0.kV = ShooterConstants.ENGLISH_kV;   // velocity feedforward (volts per rot/sec)
        englishConfig.Slot0.kA = ShooterConstants.ENGLISH_kA;    // acceleration feedforward

        // set current limit from constants value
        // removed stator (torque) current limit
        // limits current supplied from battery
        englishConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.ENGLISH_CURRENT_LIMIT;
        englishConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        //inversion on motor, set in constants
        englishConfig.MotorOutput.Inverted = ShooterConstants.ENGLISH_INVERSION;
        
        //apply configuration
        englishWheel.getConfigurator().apply(englishConfig);

        // Main Flywheel Mechanism
        mainWheel = new TalonFX(this.shooterConfig.getShooterMainId());
        mainConfig = new TalonFXConfiguration();
        mainConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // PID gains for velocity control (slot 0)
        // COULD SET CONTANTS WHEN FIGURE OUT GOOD VALUES OR SMARTDASHBOARD
        mainConfig.Slot0.kP = ShooterConstants.MAIN_kP;   // volts per rotation/sec error
        mainConfig.Slot0.kI = ShooterConstants.MAIN_kI;
        mainConfig.Slot0.kD = ShooterConstants.MAIN_kD;  // volts per rotation/sec²
        mainConfig.Slot0.kS = ShooterConstants.MAIN_kS;   // static friction feedforward (volts)
        mainConfig.Slot0.kV = ShooterConstants.MAIN_kV;   // velocity feedforward (volts per rot/sec)
        mainConfig.Slot0.kA = ShooterConstants.MAIN_kA;    // acceleration feedforward

        // set current limit from constants value
        // removed stator (torque) current limit
        // limits current supplied from battery
        mainConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.MAIN_CURRENT_LIMIT;
        mainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        //inversion on motor, set in constants
        mainConfig.MotorOutput.Inverted = ShooterConstants.MAIN_INVERSION;
        
        //apply configuration
        mainWheel.getConfigurator().apply(mainConfig);

    }

    public void configureFeederMotor(){
        // Feeder Flywheel Mechanism
        // test motor init for sparkmax
        feederWheel = new SparkMax(shooterConfig.getFeederId(), SparkLowLevel.MotorType.kBrushless);
        feederClosedLoopController = feederWheel.getClosedLoopController();
        feederRelativeEncoder = feederWheel.getEncoder();
        feederConfig = new SparkMaxConfig();

        feederConfig.inverted(ShooterConstants.FEEDER_INVERSION);

        feederConfig.encoder
                .velocityConversionFactor(1);
        // if we want to convert from rpm to another unit, do so here

        // PID NEEDS TUNING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        feederConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for velocity control in slot 0
                // ALWAYS USE SLOT 0
                .p(ShooterConstants.FEEDER_kP, ClosedLoopSlot.kSlot0)
                .i(ShooterConstants.FEEDER_kI, ClosedLoopSlot.kSlot0)
                .d(ShooterConstants.FEEDER_kD, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0).feedForward
                // kV is now in Volts, so we multiply by the nominal voltage (12V)
                .kV(ShooterConstants.FEEDER_kV, ClosedLoopSlot.kSlot0);

        feederConfig.idleMode(IdleMode.kCoast);
        // how the motor behaves when it has 0v written to it

        feederConfig.smartCurrentLimit(ShooterConstants.FEEDER_CURRENT_LIMIT);

        // Example: Set ramp rate to 0.5 seconds (0 to 100% in 0.5s)
        // Acts as slew rate for sparkmax
        feederConfig.closedLoopRampRate(ShooterConstants.FEEDER_SLEW_RATE);

        feederWheel.configure(
                feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
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
        startShooter();
        mainRPMGoal = 750;
        englishRPMGoal = 1250;
        // takes velocity in RPM
        feederRPM = 1700; // arbitrary value currently
    }

    public void setFarShot() {
        startShooter();
        mainRPMGoal = 1250;
        englishRPMGoal = 2250;
        feederRPM = 1700;
    }

    public void setAutoShot() {
        startShooter();
        mainRPMGoal = calcMainRPM(solverMainVelocity);
        englishRPMGoal = calcEnglishRPM(solverEnglishVelocity);
        feederRPM = 1700;
    }

    public void zeroRPM() {
        mainRPMGoal = 0;
        englishRPMGoal = 0;
        feederRPM = 0;
    }

    // Place status values here
    public boolean getShooterStatus() {
        return shooterStatus;
    }

    public boolean getLastShooterStatus() {
        return lastShooterStatus;
    }

    public boolean atSpeed() {
        return Math.abs(mainWheel.getVelocity().getValueAsDouble() * 60 - mainRPMGoal) < (ShooterConstants.SHOOTER_SPEED_TOLERANCE_RPM)
                && Math.abs(englishWheel.getVelocity().getValueAsDouble() * 60 - englishRPMGoal) < (ShooterConstants.SHOOTER_SPEED_TOLERANCE_RPM);
    }

    public double calcMainRPM(double target_wheel_vel) {
        // takes desired velocity in ft/s and returns desired velocity in rpm
        // ft/s to rpm is 38.1971863421 * ft/s = rpm
        double d_main = ShooterConstants.MAIN_WHEEL_DIAMETER; // in
        double gear_ratio = ShooterConstants.MAIN_GEAR_RATIO;
        double target_motor_rpm = (target_wheel_vel * 60 * 12) / (Math.PI * d_main * gear_ratio);
        return target_motor_rpm;

    }

    public void setMainRPM(double target_motor_rpm){
        // use an angular acceleration limit to avoid motor damage
        // Keep rpm within max and min
        double clampedMotorRPM = MathUtil.clamp(target_motor_rpm, 0, ShooterConstants.MAIN_MAX_RPM);
        double rampedMainRPM = mainRpmSlew.calculate(clampedMotorRPM);
        mainWheel.setControl(mainVelocityRequest.withVelocity(rampedMainRPM / 60));
        SmartDashboard.putNumber("ramped main rpm", rampedMainRPM);
    }

    public double calcEnglishRPM(double target_wheel_vel) {
        // takes desired velocity in ft/s and returns desired velocity in rpm
        // ft/s to rpm is 57.2957795131 * ft/s = rpm
        double d_english = ShooterConstants.ENGLISH_WHEEL_DIAMETER; // in
        double gear_ratio = ShooterConstants.ENGLISH_GEAR_RATIO;
        double target_motor_rpm = (target_wheel_vel * 60 * 12) / (Math.PI * d_english * gear_ratio);
        return target_motor_rpm;

    }

    public void setEnglishRPM(double target_motor_rpm){
        // use an angular acceleration limit to avoid motor damage
        // Keep rpm within max and min
        double clampedMotorRPM = MathUtil.clamp(target_motor_rpm, 0, ShooterConstants.ENGLISH_MAX_RPM);        
        double rampedEnglishRPM = englishRpmSlew.calculate(clampedMotorRPM);
        englishWheel.setControl(englishVelocityRequest.withVelocity(rampedEnglishRPM / 60));
        SmartDashboard.putNumber("ramped english rpm", rampedEnglishRPM);
    }

    public double getMainVelocity() {
        // get angular velocity of main wheel in ft/s
        double gear_ratio = ShooterConstants.MAIN_GEAR_RATIO;
        double motorRPM = mainWheel.getVelocity().getValueAsDouble() * 60;
        SmartDashboard.putNumber("mainRealRPM", motorRPM);
        double wheel_rpm = motorRPM * gear_ratio; // Note: gear ratio is set in shooter constants and passed into the
        // PID constructor, which is being used to calculate rpm separately. You would
        // either need to remove its implementation
        // from the PID class (which after thought would be better since it was used as
        // a temporary measure) or remove its implementation
        // here since you would be accounting for gear ratio twice, not once.
        // Additional note: Pretty sure you would divide the motor rpm by the gear ratio
        // to get the wheel rpm, not multiply.
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // We need actual rpm, not PID calculated rpm
        double v_main = (wheel_rpm * Math.PI * ShooterConstants.MAIN_WHEEL_DIAMETER) / (12 * 60);
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        SmartDashboard.putNumber("mainRealVelocity", v_main);
        return v_main;
    }

    public double getEnglishVelocity() {
        // get angular velocity of main wheel in ft/s
        double gear_ratio = ShooterConstants.ENGLISH_GEAR_RATIO;
        double motor_rpm = englishWheel.getVelocity().getValueAsDouble() * 60;
        SmartDashboard.putNumber("englishRealRPM", motor_rpm);
        double wheel_rpm = motor_rpm * gear_ratio;
        double v_english = (wheel_rpm * Math.PI * ShooterConstants.ENGLISH_WHEEL_DIAMETER) / (12);
        return v_english;
    }

    public double getLaunchAngle(double main_vel, double english_vel) {
        // given current main wheel velocity and english wheel velocity
        // return launch angle
        // This is an empirical formula, current formula is a guess, needs tuning
        double launchAngle = (5 * ShooterConstants.MAIN_MAX_RPM * main_vel)
                / (ShooterConstants.ENGLISH_MAX_RPM * english_vel);
        return launchAngle;
    }

    public double getLaunchVelocity() {
        // given current main wheel velocity and english wheel velocity
        // return launch velocity in ft/s

        // This is an empirical formula, current formula is a guess, needs tuning
        // The assumption is that the ball velocity is the average of the wheel
        // velocities.
        // It is also assumed that there is a loss factor in transferring momentum from
        // wheel to ball
        // and that the english wheel transfers less than the main

        double english_vel = getEnglishVelocity();
        double main_vel = getMainVelocity();
        double launchVelocity = calcLaunchVelocity(english_vel, main_vel);
        SmartDashboard.putNumber("launchVelocity", launchVelocity);
        return launchVelocity;
    }

    public double calcLaunchVelocity(double englishVelocity, double mainVelocity) {
        // given current main wheel velocity and english wheel velocity
        // return launch velocity in ft/s

        // This is an empirical formula, current formula is a guess, needs tuning
        // The assumption is that the ball velocity is the average of the wheel
        // velocities.
        // It is also assumed that there is a loss factor in transferring momentum from
        // wheel to ball
        // and that the english wheel transfers less than the main

        double L_main = 0.97; // loss factor
        double L_english = 0.85; // loss factor
        double launchVelocity = (L_english * englishVelocity + L_main * mainVelocity) / 2;
        return launchVelocity;
    }



    public double calcTrajectory(double launch_v, double launch_alpha) {
        // Projectile motion calculation including air resistance
        // Returns distance at which ball enters center of hub,
        // units are ft, lb, and s
        // returns distance in meters
        double dist_in_meters; // distance at which ball would enter hub
        double rho = 0.075; // lb/ft^3 ball density
        double C = 0.6; // drag constant
        double A = Math.PI * Math.pow(5.9 / 24, 2); // ft^2
        double D = (rho * C * A) / 2;
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

        for (int i = 0; i < 5000; i++) {
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
            if (y < y_old && y <= y_hub) {
                SmartDashboard.putBoolean("Solver converged", true);
                SmartDashboard.putNumber("SolverY", y);

                dist_in_meters = x * 0.3048;
                SmartDashboard.putNumber("solverX", dist_in_meters);
                return dist_in_meters; // return in meters
            }

            if (y >= y_old) {
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

    public void solver() {
        // get new pose
        Pose2d robotPose = mSwerveSubsystem.getEstimatedPose();
        double launchAngle;
        double launchVelocity;
        double error;
        double tolerance;
        double K;
        // ensure the alliance hub pose is correct
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                goalPosition = ShooterConstants.redHubCenter;
            } else {
                goalPosition = ShooterConstants.blueHubCenter;
            }
        }
        // translations taken from camera in Constants
        Pose2d shooterPose = robotPose.transformBy(new Transform2d(new Translation2d(-0.1778, 0.3302),
                new Rotation2d(Math.toRadians(90))));
        double[] shooterArray = { shooterPose.getX(), shooterPose.getY() };
        SmartDashboard.putNumberArray("shooterPose", shooterArray);
        double distanceToHub = new Translation2d(goalPosition.getX(), goalPosition.getY())
                .getDistance(new Translation2d(shooterPose.getX(), shooterPose.getY()));
        if (distanceToHub >= 6) {
            // if the shooter is 6m+ from the hub, don't run solver since it's too far.
            SmartDashboard.putString("Target", "out of range");
            return;
        }
        launchAngle = 75; // degrees, temporary value for comp 1
        //launch velocity isn't the actual velocity of the wheels
        //the solver performs math with simulated values
        //it sets the motor rpms to the simulated values once they're within tolerance
        launchVelocity = calcLaunchVelocity(solverEnglishVelocity, solverMainVelocity); // LOSS NEEDS TUNING
        double launch_distance = calcTrajectory(launchVelocity, launchAngle); // D value needs tuning
        error = launch_distance - distanceToHub;
        tolerance = 0.0254; // meters
        K = 2; // ft/s / m NEEDS TUNING
        SmartDashboard.putNumber("launch distance", launch_distance);
        SmartDashboard.putNumber("distance to hub", distanceToHub);
        SmartDashboard.putNumber("Shooter Error", error);

        // Main and English wheels' positive rotation direction needs to be inverted.
        if (Math.abs(error) < tolerance) {
            // if the launch distance is within tolerance, keep the wheel velocities the
            // same
            // don't need to set new velocities to wheels
            SmartDashboard.putString("Target", "locked");
            
            //removed setter statements since velocity is already correct
        } else {
            solverMainVelocity -= K * error;
            solverEnglishVelocity -= K * error;
            if (error > tolerance) {
                // decrease wheel speeds, overshooting
                SmartDashboard.putString("Target", "too close");
            } else if (error < 0 && Math.abs(error) > tolerance) {
                // increase wheel speeds, undershooting
                SmartDashboard.putString("Target", "too far");
            }
        }
        SmartDashboard.putNumber("target english velocity", solverEnglishVelocity);
        SmartDashboard.putNumber("target main velocity", solverMainVelocity);

    }

    @Override
    public void periodic() { // Update inputs, calculate, then set voltages every loop

        if (shooterConfig.getIsPresent()) {
            // reset pid if shooterstatus just became true, toggle if shooter status is
            // true.
            solver();
            getMainVelocity();
            getEnglishVelocity();
            getFeederVelocity();
            if (shooterStatus) {
                // if shooter is enabled
                setFeederRPM(feederRPM);
                setMainRPM(mainRPMGoal);
                setEnglishRPM(englishRPMGoal);
            } else {
                // if shooter is disabled
                setFeederRPM(0);
                setMainRPM(0);
                setEnglishRPM(0);
            }

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

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("Shooter init sendable called");
        builder.setSmartDashboardType("Shooter Controller");
        builder.addBooleanProperty("Shooter Status", this::getShooterStatus, null);
        builder.addBooleanProperty("Last Shooter Status", this::getLastShooterStatus, null);
        builder.addBooleanProperty("At speed", this::atSpeed, null);
        super.initSendable(builder);
    }

    public void setFeederRPM(double feederRPM) {
        // sends the RPM to the motor
            // use an angular acceleration limit to avoid motor damage
        // Keep rpm within max and min
        double clampedMotorRPM = MathUtil.clamp(feederRPM, 0, ShooterConstants.ENGLISH_MAX_RPM);        
        double rampedFeederRPM = feederRpmSlew.calculate(clampedMotorRPM);
        SmartDashboard.putNumber("ramped feeder RPM", rampedFeederRPM);
        feederClosedLoopController.setSetpoint(rampedFeederRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public double getFeederVelocity() {
        // returns the current velocity of the motor
        SmartDashboard.putNumber("feederRealRPM", feederRelativeEncoder.getVelocity());
        return feederRelativeEncoder.getVelocity();
    }
}