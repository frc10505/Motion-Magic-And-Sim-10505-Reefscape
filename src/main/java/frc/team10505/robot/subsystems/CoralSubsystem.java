package frc.team10505.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import au.grapplerobotics.LaserCan;

import static frc.team10505.robot.subsystems.HardwareConstants.*;
import static frc.team10505.robot.Constants.CoralConstants.*;

public class CoralSubsystem extends SubsystemBase {
    // Motor controllers
    private final SparkMax intakeLeft = new SparkMax(CORAL_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax intakeRight = new SparkMax(CORAL_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeRightConfig = new SparkMaxConfig();

    // Laser sensors
    private final LaserCan inLaser = new LaserCan(CORAL_IN_LASER_ID);
    private final LaserCan outLaser = new LaserCan(CORAL_OUT_LASER_ID);

    // Sim Flying Wheeels
    private final Color8Bit red = new Color8Bit(Color.kFirstRed);
    private final Color8Bit green = new Color8Bit(Color.kGreen);

    private final FlywheelSim intakeLeftSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 5), DCMotor.getNEO(1));

    private final FlywheelSim intakeRightSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 5), DCMotor.getNEO(1));

    private final Mechanism2d coralIntakeMech = new Mechanism2d(3, 1.2);

    private final MechanismRoot2d leftSimRoot = coralIntakeMech.getRoot("leftRoot", .6, 0.6);
    private final MechanismRoot2d rightSimRoot = coralIntakeMech.getRoot("rightRoot", 2.4, 0.6);

    private final MechanismRoot2d inSensorRoot = coralIntakeMech.getRoot("inSensorRoot", 1.5, 1);
    private final MechanismRoot2d outSensorRoot = coralIntakeMech.getRoot("outSensorRoot", 1.5, 0.2);


    private final MechanismRoot2d sigmaSimRoot = coralIntakeMech.getRoot("sigmaRoot", .6, 0.6);
    private final MechanismRoot2d skibidiSimRoot = coralIntakeMech.getRoot("skbidiRoot", 2.4, 0.6);
    private final MechanismRoot2d donnysFreakySimRoot = coralIntakeMech.getRoot("donnysRoot", 0.8, 0.6);
    private final MechanismRoot2d waltersApeishSimRoot = coralIntakeMech.getRoot("waltersRoot", 2.2, 0.6);

    private final MechanismLigament2d leftIntakeViz = leftSimRoot
            .append(new MechanismLigament2d("leftIntakeLigament", 0.4, 000));
    private final MechanismLigament2d rightIntakeViz = rightSimRoot
            .append(new MechanismLigament2d("rightIntakeLigament", 0.4, 180));

    private final MechanismLigament2d inSensor = inSensorRoot
            .append(new MechanismLigament2d("inSensorLigament", 0.15, 90, 40, red));
    private final MechanismLigament2d outSensor = outSensorRoot
            .append(new MechanismLigament2d("outSensorLigament", 0.15, -90, 40, red));

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

    private CommandJoystick monkeyJoystick;

    /* Constructor */
    public CoralSubsystem() {
        SmartDashboard.putData("coralIntake", coralIntakeMech);

        // Left intake config
        intakeLeftConfig.idleMode(IdleMode.kBrake);
        intakeLeftConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT, CORAL_MOTOR_CURRENT_LIMIT);
        intakeLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right intake config
        intakeRightConfig.idleMode(IdleMode.kBrake);
        intakeRightConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT, CORAL_MOTOR_CURRENT_LIMIT);
        intakeRightConfig.inverted(true);
        intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    }

    /*Other constructor, intended for use in simulation */
    public CoralSubsystem(CommandJoystick monkeyJoystick) {
        this.monkeyJoystick = monkeyJoystick;
        SmartDashboard.putData("coralIntake", coralIntakeMech);

        // Left intake config
        intakeLeftConfig.idleMode(IdleMode.kBrake);
        intakeLeftConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT, CORAL_MOTOR_CURRENT_LIMIT);
        intakeLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right intake config
        intakeRightConfig.idleMode(IdleMode.kBrake);
        intakeRightConfig.smartCurrentLimit(CORAL_MOTOR_CURRENT_LIMIT, CORAL_MOTOR_CURRENT_LIMIT);
        intakeRightConfig.inverted(true);
        intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
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
                simMotorSpeed = CORAL_TROUGH_LEFT_SPEED;
                simSecondaryMotorSpeed = CORAL_TROUGH_RIGHT_SPEED;

            },
                    () -> {
                        simMotorSpeed = 0;
                        simSecondaryMotorSpeed = 0;
                    });
        } else {
            return runEnd(() -> {
                intakeLeft.set(CORAL_TROUGH_LEFT_SPEED);
                intakeRight.set(CORAL_TROUGH_RIGHT_SPEED);
            },
                    () -> {
                        intakeLeft.set(0);
                        intakeRight.set(0);
                    });
        }
    }

    public Command slowEndIntake(double firstSpeed) {
        if (Utils.isSimulation()) {
            return runEnd(() -> {
                simMotorSpeed = firstSpeed;

            },
                    () -> {
                        runIntake(CORAL_SLOW_SPEED).until(() -> (outSensor() && !inSensor()));
                    });
        } else {

            return runEnd(() -> {
                intakeLeft.set(firstSpeed);
                intakeRight.set(firstSpeed);
            },
                    () -> {
                        runIntake(CORAL_SLOW_SPEED).until(() -> (outSensor() && !inSensor()));
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

            if(simSecondaryMotorSpeed == 0){
                intakeRightSim.setInput(simSecondaryMotorSpeed);
            } else {
                intakeRightSim.setInput(simMotorSpeed);
            }

            if(inSensor()){
                inSensor.setColor(green);
            } else{
                inSensor.setColor(red);
            }

            if(outSensor()){
                outSensor.setColor(green);
            } else{
                outSensor.setColor(red);
            }

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
}
