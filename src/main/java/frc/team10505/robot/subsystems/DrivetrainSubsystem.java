package frc.team10505.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team10505.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import static frc.team10505.robot.Constants.DrivetrainConstants.*;

public class DrivetrainSubsystem extends TunerSwerveDrivetrain implements Subsystem {
    private SwerveRequest.ApplyRobotSpeeds robotDrive = new SwerveRequest.ApplyRobotSpeeds();

    private CommandJoystick joystick = new CommandJoystick(0);

    // SwerveRequest.ApplyChassisSpeeds();

    private final LaserCan leftLaser = new LaserCan(53);
    private final LaserCan rightLaser = new LaserCan(52);

    public final Spark blinkyLight = new Spark(0);

    double turnDistance = 0;
    double strafeDistance = 0;
    double skewDistance = 0;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    // Three constructors that use different parameters
    public DrivetrainSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public DrivetrainSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public DrivetrainSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public DrivetrainSubsystem(
            CommandJoystick joystick,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            this.joystick = joystick;
            startSimThread();
        }
    }

    public DrivetrainSubsystem(
            CommandJoystick joystick,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            this.joystick = joystick;
            startSimThread();
        }
    }

    public DrivetrainSubsystem(
            CommandJoystick joystick,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            this.joystick = joystick;
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    // The command we referense to make the drivetrain move
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command stop() {
        return runOnce(() -> this.setControl(robotDrive.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))));
    }

    public Command setRobotSpeeds(double xSpeed, double ySpeed, double rotSpeed) {
        return runOnce(() -> this.setControl(robotDrive.withSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed))));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public boolean seesLeftSensor() {
        if (Utils.isSimulation()) {
            return joystick.button(3).getAsBoolean();

        } else {
            try {
                LaserCan.Measurement leftMeas = leftLaser.getMeasurement();
                return (leftMeas.distance_mm < leftDriveLaserDistance
                        && leftMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
            } catch (NullPointerException l) {
                // DriverStation.reportError("left sensor is null", l.getStackTrace());
                return false;
            }
        }

    }

    public boolean seesRightSensor() {
        if (Utils.isSimulation()) {
            return joystick.button(4).getAsBoolean();

        } else {
            try {
                LaserCan.Measurement RightMeas = rightLaser.getMeasurement();
                return (RightMeas.distance_mm < rightDriveLaserDistance); // && RightMeas.status ==
                                                                          // LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);

            } catch (NullPointerException r) {
                // DriverStation.reportError("right sensor is null", r.getStackTrace());
                return false;
            }
        }
    }

    public boolean autonSeesLeftSensor() {
        if (Utils.isSimulation()) {
            return joystick.button(3).getAsBoolean();

        } else {
            try {
                LaserCan.Measurement leftMeas = leftLaser.getMeasurement();
                return (leftMeas.distance_mm < autonLeftDriveLaserDistance
                        && leftMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
            } catch (NullPointerException l) {
                // DriverStation.reportError("left sensor is null", l.getStackTrace());
                return false;
            }
        }

    }

    public boolean autonSeesRightSensor() {
        if (Utils.isSimulation()) {
            return joystick.button(4).getAsBoolean();

        } else {
            try {
                LaserCan.Measurement RightMeas = rightLaser.getMeasurement();
                return (RightMeas.distance_mm < autonRightDriveLaserDistance); // && RightMeas.status ==
                                                                               // LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);

            } catch (NullPointerException r) {
                // DriverStation.reportError("right sensor is null", r.getStackTrace());
                return false;
            }
        }
    }

    public boolean seesLeftSensorClose() {
        if (Utils.isSimulation()) {
            return joystick.button(3).getAsBoolean();

        } else {
            try {
                LaserCan.Measurement leftMeas = leftLaser.getMeasurement();
                return (leftMeas.distance_mm < closeLeftDriveLaserDistance
                        && leftMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
            } catch (NullPointerException l) {
                // DriverStation.reportError("left sensor is null", l.getStackTrace());
                return false;
            }
        }

    }

    public boolean seesRightSensorClose() {
        if (Utils.isSimulation()) {
            return joystick.button(4).getAsBoolean();

        } else {
            try {
                LaserCan.Measurement RightMeas = rightLaser.getMeasurement();
                return (RightMeas.distance_mm < closeRightDriveLaserDistance); // && RightMeas.status ==
                                                                               // LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);

            } catch (NullPointerException r) {
                // DriverStation.reportError("right sensor is null", r.getStackTrace());
                return false;
            }
        }
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });

        }

        // if (seesLeftSensorClose() && seesRightSensorClose() ) {
        // blinkyLight.set(0.35);//flashy color 2(blue)
        // } else if (seesLeftSensor() && seesRightSensor()) {
        // blinkyLight.set(-0.11);//strobe red
        // } else if (!seesLeftSensor() && !seesRightSensor()) {
        // blinkyLight.set(0.61);//red
        // } else if (seesLeftSensor() | seesRightSensor()) {
        // blinkyLight.set(0.77);//green
        // }

        try {
            SmartDashboard.putNumber("left Laser Distance", leftLaser.getMeasurement().distance_mm);
        } catch (NullPointerException r) {
            // DriverStation.reportError("left sensor is null", r.getStackTrace());
        }

        try {
            SmartDashboard.putNumber("right Laser Distance", rightLaser.getMeasurement().distance_mm);
        } catch (NullPointerException r) {
            // DriverStation.reportError("right sensor is null", r.getStackTrace());
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void configPathplanner() {
        try {
            var config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    () -> getState().Pose,
                    this::resetPose,
                    () -> getState().Speeds,
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            new PIDConstants(10, 0, 0), // drive
                            new PIDConstants(7, 0, 0)), // Rotation
                    config,
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception ex) {
            DriverStation.reportError("something may or may not be broken, idk", ex.getStackTrace());
        }
    }

      @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {

        Matrix<N3, N1> tagStdDevs = VecBuilder.fill(8.0, 8.0, 8.0);
       // visionMeasurementStdDevs.set(m_drivetrainId, kNumConfigAttempts, timestampSeconds);
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), tagStdDevs);
       // super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // same thing as before, but could be used in place of it if we use the standard deviation of vision measurments(I have no idea how to do that!)
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        visionMeasurementStdDevs.set(m_drivetrainId, kNumConfigAttempts, timestampSeconds);
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

}
