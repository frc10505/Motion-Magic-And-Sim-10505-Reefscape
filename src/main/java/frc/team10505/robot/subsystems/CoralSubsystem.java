package frc.team10505.robot.subsystems;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.team10505.robot.Robot;
import static frc.team10505.robot.Constants.CoralConstants.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import au.grapplerobotics.LaserCan;

public class CoralSubsystem extends SubsystemBase {

    private  CommandJoystick monkeyJoystick = new CommandJoystick(0);
    // Motor controllers
    private final SparkMax intakeLeft = new SparkMax(kLeftMotorId, MotorType.kBrushless);
    private final TalonFX simIntakeLeft = new TalonFX(4);
    private SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    private final SparkMax intakeRight = new SparkMax(kRightMotorID, MotorType.kBrushless);
    private final TalonFX intakeSigma = new TalonFX(7);
    private final TalonFX intakeSkibidi = new TalonFX(8);
    private final TalonFX simIntakeRight = new TalonFX(5);
    private final TalonFX simIntakeSigma = new TalonFX(6);
    private final TalonFX simIntakeSkibidi = new TalonFX(9);
    private SparkMaxConfig intakeRightConfig = new SparkMaxConfig();

    // Laser sensors
    private final LaserCan inLaser = new LaserCan(60);
    private final LaserCan outLaser = new LaserCan(61);

    // Sim Flying Wheeels

    private final FlywheelSim intakeLeftSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.3, 5), DCMotor.getNEO(1));

    private final FlywheelSim intakeRightSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.3, 5), DCMotor.getNEO(1));

    private final Mechanism2d coralIntakeMech = new Mechanism2d(3, 1.2);

    private final MechanismRoot2d leftSimRoot = coralIntakeMech.getRoot("leftRoot", .6, 0.6);// x = 2.5, y = 6

    private final MechanismRoot2d rightSimRoot = coralIntakeMech.getRoot("rightRoot", 0.7, 0.6);// x = 5, y = 6

    private final MechanismRoot2d sigmaSimRoot = coralIntakeMech.getRoot("sigmaRoot", .6, 0.6);// x = 2.5, y = 6

    private final MechanismRoot2d skibidiSimRoot = coralIntakeMech.getRoot("skbidiRoot", 0.7, 0.6);

    private final MechanismLigament2d leftIntakeViz = leftSimRoot
            .append(new MechanismLigament2d("leftIntakeLigament", 0.4, 000));

    private final MechanismLigament2d rightIntakeViz = rightSimRoot
            .append(new MechanismLigament2d("rightIntakeLigament", 0.4, 000));

    private final MechanismLigament2d sigmaIntakeViz = sigmaSimRoot
            .append(new MechanismLigament2d("sigmaIntakeLigament", 0.4, 000));

    private final MechanismLigament2d skibidiIntakeViz = skibidiSimRoot
            .append(new MechanismLigament2d("skibidiIntakeLigament", 0.4, 000));

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
                simIntakeLeft.setVoltage(speed);
                simIntakeRight.setVoltage(speed);
            }, () -> {
                simIntakeLeft.setVoltage(0);
                simIntakeRight.setVoltage(0);
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
                simIntakeLeft.setVoltage(speed);
                simIntakeRight.setVoltage(speed);
            });
        } else {
            return runOnce(() -> {
                intakeLeft.set(speed);
                intakeRight.set(speed);
            });
        }
    }



    public Command trough() {
        if(Utils.isSimulation()){

            return runEnd(() -> {
                simIntakeLeft.setVoltage(kLeftL1Speed);
                simIntakeRight.setVoltage(kRightL1Speed * kTroughRightMotorPercentage);
            },
                    () -> {
                        simIntakeLeft.setVoltage(0);
                        simIntakeRight.setVoltage(0);
                    });
         } else{
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
        if(Utils.isSimulation()){
            return runEnd(() -> {
                simIntakeLeft.setVoltage(kIntakeSpeed);
                simIntakeRight.setVoltage(kIntakeSpeed);
            },
                    () -> {
                        setIntake(0.05).until(() -> (outSensor() && !inSensor()));
                    });
        }else{

        return runEnd(() -> {
            intakeLeft.set(kIntakeSpeed);
            intakeRight.set(kIntakeSpeed);
        },
                () -> {
                    setIntake(0.05).until(() -> (outSensor() && !inSensor()));
                });
            }
    }



   

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("inSensor", inSensor());
        SmartDashboard.putBoolean("outSensor", outSensor());

        var leftCurrentPos = leftIntakeViz.getAngle();
        var rightCurrentPos = rightIntakeViz.getAngle();
        var sigmaCurrentPos = sigmaIntakeViz.getAngle();
        var skibidiCurrentPos = skibidiIntakeViz.getAngle();

        var leftChange = simIntakeLeft.getMotorVoltage().getValueAsDouble() * 10;
        var rightChange = simIntakeRight.getMotorVoltage().getValueAsDouble() * 10;
        var sigmaChange = simIntakeSigma.getMotorVoltage().getValueAsDouble() * 7;
        var skibidiChange = simIntakeSkibidi.getMotorVoltage().getValueAsDouble() * 4;

        leftIntakeViz.setAngle(leftCurrentPos + leftChange);
        rightIntakeViz.setAngle(rightCurrentPos + rightChange);
        sigmaIntakeViz.setAngle(sigmaCurrentPos + sigmaChange);
        skibidiIntakeViz.setAngle(skibidiCurrentPos + skibidiChange);
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
