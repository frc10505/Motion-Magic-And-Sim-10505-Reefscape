package frc.team10505.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import static frc.team10505.robot.Constants.CoralConstants.*;
import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import au.grapplerobotics.LaserCan;

public class CoralSubsystem extends SubsystemBase {

    private CommandJoystick monkeyJoystick;// = new CommandJoystick(0);
    // Motor controllers
    private final SparkMax intakeLeft = new SparkMax(kLeftMotorId, MotorType.kBrushless);
    private final SparkMax intakeRight = new SparkMax(kRightMotorID, MotorType.kBrushless);
    private SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeRightConfig = new SparkMaxConfig();

    // Laser sensors
    private final LaserCan inLaser = new LaserCan(60);
    private final LaserCan outLaser = new LaserCan(61);

    // Sim Flying Wheeels
    private final FlywheelSim intakeLeftSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 5), DCMotor.getNEO(1));

    private final FlywheelSim intakeRightSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 5), DCMotor.getNEO(1));

    private final Mechanism2d coralIntakeMech = new Mechanism2d(3, 1.2);

    private final MechanismRoot2d leftSimRoot = coralIntakeMech.getRoot("leftRoot", .6, 0.6);
    private final MechanismRoot2d rightSimRoot = coralIntakeMech.getRoot("rightRoot", 2.4, 0.6);
    private final MechanismRoot2d sigmaSimRoot = coralIntakeMech.getRoot("sigmaRoot", .6, 0.6);
    private final MechanismRoot2d skibidiSimRoot = coralIntakeMech.getRoot("skbidiRoot", 2.4, 0.6);
    private final MechanismRoot2d donnysFreakySimRoot = coralIntakeMech.getRoot("donnysRoot", 0.8, 0.6);
    private final MechanismRoot2d waltersApeishSimRoot = coralIntakeMech.getRoot("waltersRoot", 2.2, 0.6);

    private final MechanismLigament2d leftIntakeViz = leftSimRoot
            .append(new MechanismLigament2d("leftIntakeLigament", 0.4, 000));
    private final MechanismLigament2d rightIntakeViz = rightSimRoot
            .append(new MechanismLigament2d("rightIntakeLigament", 0.4, 180));
    private final MechanismLigament2d sigmaIntakeViz = sigmaSimRoot
            .append(new MechanismLigament2d("sigmaIntakeLigament", 0.4, 000));
    private final MechanismLigament2d skibidiIntakeViz = skibidiSimRoot
            .append(new MechanismLigament2d("skibidiIntakeLigament", 0.4, 180));
    private final MechanismLigament2d walterIntakeViz = waltersApeishSimRoot
            .append(new MechanismLigament2d("walterIntakeLigament", 0.4, 180));
    private final MechanismLigament2d donnyIntakeViz = donnysFreakySimRoot
            .append(new MechanismLigament2d("leftIntakeLigament", 0.4, 000));

    private double simMotorSpeed = 0;
    private double simSecondaryMotorSpeed = 0;

    /* Constructor */
    public CoralSubsystem() {
        SmartDashboard.putData("coralIntake", coralIntakeMech);
        configCoralSubsys();
    }

    public CoralSubsystem(CommandJoystick monkeyJoystick) {
        this.monkeyJoystick = monkeyJoystick;
        SmartDashboard.putData("coralIntake", coralIntakeMech);
        configCoralSubsys();
    }

    /* Calculations */
    public boolean inSensor() {
        if (Utils.isSimulation()) {
            return monkeyJoystick.button(1).getAsBoolean();

        } else {
            LaserCan.Measurement inMeas = inLaser.getMeasurement();
            return (inMeas.distance_mm < 50.0 && inMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);

        }
    }

    public boolean outSensor() {
        if (Utils.isSimulation()) {
            return monkeyJoystick.button(2).getAsBoolean();

        } else {
            LaserCan.Measurement outMeas = outLaser.getMeasurement();
            return (outMeas.distance_mm < 100.0 && outMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
        }

    }

    /* BETTER commands to referense */
    public Command runIntake(double speed) {
        if (Utils.isSimulation()) {
            return runEnd(() -> {
                simMotorSpeed = speed;

            }, () -> {

                simMotorSpeed = 0;

            });
        } else {
            return runEnd(() -> {
                intakeLeft.set(speed);
                intakeRight.set(speed);
            }, () -> {
                intakeLeft.set(0);
                intakeRight.set(0);
            });
        }
    }

    public Command setIntake(double speed) {
        if (Utils.isSimulation()) {
            return runOnce(() -> {
                simMotorSpeed = speed;
            });
        } else {
            return runOnce(() -> {
                intakeLeft.set(speed);
                intakeRight.set(speed);
            });
        }
    }

    public Command trough() {
        if (Utils.isSimulation()) {

            return runEnd(() -> {
                simMotorSpeed = kLeftL1Speed;
                simSecondaryMotorSpeed = kRightL1Speed * kTroughRightMotorPercentage;

            },
                    () -> {
                        simMotorSpeed = 0;
                        simSecondaryMotorSpeed = 0;
                    });
        } else {
            return runEnd(() -> {
                intakeLeft.set(kLeftL1Speed);
                intakeRight.set(kRightL1Speed * kTroughRightMotorPercentage);
            },
                    () -> {
                        intakeLeft.set(0);
                        intakeRight.set(0);
                    });
        }
    }

    public Command slowEndIntake() {
        if (Utils.isSimulation()) {
            return runEnd(() -> {
                simMotorSpeed = kIntakeSpeed;

            },
                    () -> {
                        runIntake(0.05).until(() -> (outSensor() && !inSensor()));
                    });
        } else {

            return runEnd(() -> {
                intakeLeft.set(kIntakeSpeed);
                intakeRight.set(kIntakeSpeed);
            },
                    () -> {
                        runIntake(0.05).until(() -> (outSensor() && !inSensor()));
                    });
        }
    }

    // var to change to true if we are goofing around
    // keep false for serious stuff
    private boolean runningStupidity = false;

    @Override
    public void periodic() {

        if (Utils.isSimulation()) {
            if (runningStupidity) {
                var sigmaCurrentPos = sigmaIntakeViz.getAngle();
                var skibidiCurrentPos = skibidiIntakeViz.getAngle();
                var donnyCurrentPos = donnyIntakeViz.getAngle();
                var walterCurrentPos = walterIntakeViz.getAngle();

                var sigmaChange = simMotorSpeed * Math.random() * 14;
                var skibidiChange = simMotorSpeed * Math.random() * 8;
                var donnyChange = simMotorSpeed * Math.random() * 5;
                var walterChange = simMotorSpeed * Math.random() * 5;

                sigmaIntakeViz.setAngle(sigmaCurrentPos + sigmaChange);
                skibidiIntakeViz.setAngle(skibidiCurrentPos - skibidiChange);
                donnyIntakeViz.setAngle(donnyCurrentPos / (donnyChange * 180));
                walterIntakeViz.setAngle((walterCurrentPos * walterChange * 180));
            }

            SmartDashboard.putBoolean("inSensor", inSensor());
            SmartDashboard.putBoolean("outSensor", outSensor());

            SmartDashboard.putNumber("Sim left intake viz angle", leftIntakeViz.getAngle());
            SmartDashboard.putNumber("Sim right intake viz angle", rightIntakeViz.getAngle());
            SmartDashboard.putNumber("Sim motor speed", simMotorSpeed);

            var leftCurrentPos = leftIntakeViz.getAngle();
            var rightCurrentPos = rightIntakeViz.getAngle();

            intakeLeftSim.setInput(simMotorSpeed);
            intakeLeftSim.update(0.001);

            intakeRightSim.setInput(simMotorSpeed);
            intakeRightSim.update(0.001);

            leftIntakeViz.setAngle(leftCurrentPos + (intakeLeftSim.getAngularVelocityRPM() * 0.04));
            rightIntakeViz.setAngle(rightCurrentPos - (intakeRightSim.getAngularVelocityRPM() * 0.04));

        } else {
            SmartDashboard.putBoolean("inSensor", inSensor());
            SmartDashboard.putBoolean("outSensor", outSensor());
            SmartDashboard.putNumber("left intake motor applied output", intakeLeft.getAppliedOutput());
            SmartDashboard.putNumber("right intake motor applied output", intakeRight.getAppliedOutput());

        }
    }

    /*
     * configurations to be called in the constructor,
     * runs once during init,
     * here it is used to configure motor settings
     */
    private void configCoralSubsys() {
        // Left intake config
        intakeLeftConfig.idleMode(IdleMode.kBrake);
        intakeLeftConfig.smartCurrentLimit(kLeftMotorCurrentLimit, kLeftMotorCurrentLimit);
        intakeLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right intake config
        intakeRightConfig.idleMode(IdleMode.kBrake);
        intakeRightConfig.smartCurrentLimit(kRightMotorCurrentLimit, kRightMotorCurrentLimit);
        intakeRightConfig.inverted(true);
        intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}
