package frc.team10505.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team10505.robot.Vision;
import frc.team10505.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import static frc.team10505.robot.Constants.HardwareConstants.*;
import static frc.team10505.robot.Constants.*;

public class DrivetrainSubsystem extends TunerSwerveDrivetrain implements Subsystem {
    private SwerveRequest.ApplyRobotSpeeds autoRobotDrive = new SwerveRequest.ApplyRobotSpeeds();

    private SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private CommandJoystick joystick;
    private Vision vision;

    private final LaserCan rightLaser = new LaserCan(DRIVETRAIN_RIGHT_LASER_ID);
    private final LaserCan leftLaser = new LaserCan(DRIVETRAIN_LEFT_LASER_ID);

    public final Spark blinkyLight = new Spark(DRIVETRAIN_BLINKY_LIGHT_CHANNEL);

    //Sim vars to simulate the color changes of the LEDs
    private final Mechanism2d mech = new Mechanism2d(1, 1);
    private final MechanismRoot2d ledRoot = mech.getRoot("ledRoot", 0.1, 0.1);
    private final MechanismLigament2d ledViz = ledRoot.append(new MechanismLigament2d("LED Viz", 0.8, 0, 80, new Color8Bit(Color.kBlack)));

    private final Color8Bit red = new Color8Bit(Color.kRed);
    private final Color8Bit blue = new Color8Bit(Color.kLightBlue);
    private final Color8Bit green = new Color8Bit(Color.kGreen);
    private final Color8Bit black = new Color8Bit(Color.kBlack);

    private boolean blueOn = true;
    private boolean redOn = true;

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
    @SuppressWarnings("unused")
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
    @SuppressWarnings("unused")
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

    // constructors
    public DrivetrainSubsystem(
            Vision vision,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        this.vision = vision;
        if (Utils.isSimulation() || Utils.isReplay()) {
            startSimThread();
            SmartDashboard.putData("LED Viz", mech);
        }
    }

    public DrivetrainSubsystem(
            Vision vision,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        this.vision = vision;
        if (Utils.isSimulation() || Utils.isReplay()) {
            startSimThread();
            SmartDashboard.putData("LED Viz", mech);
        }
    }

    public DrivetrainSubsystem(
            Vision vision,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        this.vision = vision;
        if (Utils.isSimulation() || Utils.isReplay()) {
            startSimThread();
            SmartDashboard.putData("LED Viz", mech);
        }
    }

    //*Sim constructor */
    public DrivetrainSubsystem(
            Vision vision,
            CommandJoystick joystick,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        this.vision = vision;
        if (Utils.isSimulation() || Utils.isReplay()) {
            this.joystick = joystick;
            SmartDashboard.putData("LED Viz", mech);
            startSimThread();
        }
    }

    //*Sim constructor */
    public DrivetrainSubsystem(
            Vision vision,
            CommandJoystick joystick,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        this.vision = vision;
        SmartDashboard.putData("LED Viz", mech);
        if (Utils.isSimulation() || Utils.isReplay()) {
            this.joystick = joystick;
            startSimThread();
        }
    }

    //*Sim constructor */
    public DrivetrainSubsystem(
            Vision vision,
            CommandJoystick joystick,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        this.vision = vision;
        if (Utils.isSimulation() || Utils.isReplay()) {
            this.joystick = joystick;
            SmartDashboard.putData("LED Viz", mech);
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

    /** FOR THE LOVE OF EVEYTHING GOOD, ONLY USE IN AUTONS */
    public Command autoStop() {
        return runOnce(() -> this.setControl(autoRobotDrive.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))));
    }

    // // public Command setRobotSpeeds(double xSpeed, double ySpeed, double
    // rotSpeed) {
    // // return runOnce(() -> this.setControl(robotDrive.withSpeeds(new
    // ChassisSpeeds(xSpeed, ySpeed, rotSpeed))));
    // // }
    // private double skewInit = 0;
    // private double yawInit = 0;

    // public Command bruhPt1(){
    // return runOnce(()->
    // {
    // if(vision.getTargetSkew(vision.backCamSim) > 10.0
    // ||vision.getTargetSkew(vision.backCamSim) ==0){
    // skewInit = 10.0;
    // }else{
    // skewInit = Math.abs(vision.getTargetSkew(vision.backCamSim));
    // }

    // if(vision.getTargetYaw(vision.backCamSim) > 10.0 ||
    // vision.getTargetYaw(vision.backCamSim) == 0){
    // yawInit = 10.0;
    // }else{
    // yawInit = Math.abs(vision.getTargetYaw(vision.backCamSim));
    // }
    // yaw = 0;
    // skew = 0;
    // });
    // }

    // private Command bruhPrt2(){
    // return run(() ->{
    // for (double y = yawInit+skewInit; y>0; --y){
    // yaw = yawInit + y;
    // skew = skewInit + y;
    // if(yaw<0){
    // yaw = 0;
    // }
    // if(skew<0){
    // skew = 0;
    // }

    // applyRequest(() -> robotCentricDrive.withVelocityY(yaw * 10+
    // joystick.getRawAxis(1)).withVelocityX(skew * 10 +
    // joystick.getRawAxis(2)).withRotationalRate(joystick.getRawAxis(3)));
    // waitSeconds(0.5);
    // }});
    // }

    // private double yaw = 0.0;
    // private double skew = 0.0;

    // public boolean finished = false;

    // public Command goToLeftStation() {

    // return Commands.sequence(
    // bruhPt1(),
    // bruhPrt2()

    // );
    // }

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
        if (Utils.isSimulation() || Utils.isReplay()) {
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
        if (Utils.isSimulation() || Utils.isReplay()) {
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
        if (Utils.isSimulation() || Utils.isReplay()) {
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
        if (Utils.isSimulation() || Utils.isReplay()) {
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
        if (Utils.isSimulation() || Utils.isReplay()) {
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
        if (Utils.isSimulation() || Utils.isReplay()) {
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

        SmartDashboard.putBoolean("left drive sensor", seesLeftSensor());
        SmartDashboard.putBoolean("right drive sensor", seesRightSensor());

        if (Utils.isSimulation()) {
            if (seesLeftSensorClose() && seesRightSensorClose()) {
                if(blueOn){
                    blueOn = false;
                    ledViz.setColor(black);
                }else{
                    blueOn = true;
                    ledViz.setColor(blue);
                }
            } else if (seesLeftSensor() && seesRightSensor()) {
                if(redOn){
                    redOn = false;
                    ledViz.setColor(black);
                }else{
                    redOn = true;
                    ledViz.setColor(red);
                }
            } else if (!seesLeftSensor() && !seesRightSensor()) {
                ledViz.setColor(red);
            } else if (seesLeftSensor() | seesRightSensor()) {
                ledViz.setColor(green);
            }
        } else {
            if (seesLeftSensorClose() && seesRightSensorClose()) {
                blinkyLight.set(0.35);// flashy color 2(blue)
            } else if (seesLeftSensor() && seesRightSensor()) {
                blinkyLight.set(-0.11);// strobe red
            } else if (!seesLeftSensor() && !seesRightSensor()) {
                blinkyLight.set(0.61);// red
            } else if (seesLeftSensor() | seesRightSensor()) {
                blinkyLight.set(0.77);// green
            }

            try {
                SmartDashboard.putNumber("left Laser Distance", leftLaser.getMeasurement().distance_mm);
            } catch (NullPointerException r) {
                DriverStation.reportError("left sensor is null", r.getStackTrace());
            }
    
            try {
                SmartDashboard.putNumber("right Laser Distance", rightLaser.getMeasurement().distance_mm);
            } catch (NullPointerException r) {
                DriverStation.reportError("right sensor is null", r.getStackTrace());
            }
        }
        // SmartDashboard.putNumber("skew", skew);
        // SmartDashboard.putNumber("skewinit", skewInit);
        // SmartDashboard.putNumber("yaw", yaw);
        // SmartDashboard.putNumber("yawinit", yawInit);

        
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
        if (Utils.isSimulation() || Utils.isReplay()) {
            try {
                var config = RobotConfig.fromGUISettings();

                AutoBuilder.configure(
                        () -> getState().Pose, // betterPose,
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
        } else {
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
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {

        Matrix<N3, N1> tagStdDevs = VecBuilder.fill(8.0, 8.0, 8.0);
        // visionMeasurementStdDevs.set(m_drivetrainId, kNumConfigAttempts,
        // timestampSeconds);
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), tagStdDevs);
        // super.addVisionMeasurement(visionRobotPoseMeters,
        // Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // same thing as before, but could be used in place of it if we use the standard
    // deviation of vision measurments(I have no idea how to do that!)
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        visionMeasurementStdDevs.set(m_drivetrainId, kNumConfigAttempts, timestampSeconds);
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

}
