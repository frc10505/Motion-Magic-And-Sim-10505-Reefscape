/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.team10505.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    public final TalonFX elevatorMotor = new TalonFX(kElevatorMotorId, "kingKan");
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public final TalonFX elevatorFollowerMotor = new TalonFX(kElevatorFollowerMotorId, "kingKan");

    // Encoders, Real and Simulated
    private double elevatorEncoderValue = 0.0;
    private double simEncoder = 0.0;

    private double totalEffort;
    // Operator interface
    public final SendableChooser<Double> elevaterHeight = new SendableChooser<>();
    private double height = 0.0;

    // Controls, Actual
    private final PIDController elevatorController = new PIDController(KP, KI,
            KD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KS,
            KG, KV, KA);

    public boolean usePID = true;

    private final ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getFalcon500(2), 12, 10, 0.05, 0.0, 3.0, true, 2);
    public final Mechanism2d elevSimMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d elevRoot = elevSimMech.getRoot("elevRoot", 0.75, 0.1);
    public final MechanismLigament2d elevatorViz = elevRoot
            .append(new MechanismLigament2d("elevatorLigament", 2, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));

    /* Constructor, runs everything inside during initialization */
    public ElevatorSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 12;// TODO check gearstack irl

        MotionMagicConfigs motionMagic = cfg.MotionMagic;
        motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(24))// 5//10
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(48))// 10//20
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));// 100

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kG = 0.5297854;// .528
        slot0.kP = 240;// 60; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0;// 0.5; // A velocity error of 1 rps results in 0.5 V output

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = elevatorMotor.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        SmartDashboard.putData("elevatorViz", elevSimMech);
        // elevatorMotor.setPosition(0.0);

        var motorConfig = new MotorOutputConfigs();

        // // // set current limits
        // var limitConfigs = new CurrentLimitsConfigs();
        // limitConfigs.StatorCurrentLimit = kElevatorMotorCurrentLimit;
        // limitConfigs.StatorCurrentLimitEnable = true;

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorMotor.getConfigurator().apply(motorConfig);
        // // elevatorMotor.getConfigurator().apply(limitConfigs);

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorFollowerMotor.getConfigurator().apply(motorConfig);
        // elevatorFollowerMotor.getConfigurator().apply(limitConfigs);
        elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));
    }

    /* commands to referense */
    // changes our setpoint, which changes our pid calcuations therefore effort to
    // the motor, which happens periodically
    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight// * (Math.PI * 1.751 * 2) / 12.0 * -1.0
            ;
        });
    }

    public Command setMotor(double voltage) {
        return runEnd(() -> {
            usePID = false;
            elevatorMotor.setVoltage(voltage);
        },
                () -> {
                    elevatorMotor.setVoltage(0);
                    usePID = true;
                });
    }

    // ONLY to use for testing motor direction
    // public Command testElevator(double voltage){
    // return runEnd(() -> {
    // elevatorMotor.setVoltage(voltage);
    // }, () -> {
    // elevatorMotor.setVoltage(0.0);
    // });
    // }

    /* Calculations */
    public double getElevatorEncoder() {
        return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751 * 2) / 12.0) * -1.0;
    }

    public boolean isNearGoal() {
        if (Utils.isSimulation()) {
            return MathUtil.isNear(height / 30, elevatorViz.getLength(), 2);

        } else {
            return MathUtil.isNear(height, getElevatorEncoder(), 2);
        }
    }

    public boolean issGigh() {
        if (Utils.isSimulation()) {
            return elevatorViz.getLength() > 30 / 30;
        } else
            return getElevatorEncoder() > 30;
    }

    public boolean isAbove(double heightOfChoice) {
        if (Utils.isSimulation()) {
            return elevatorViz.getLength() > heightOfChoice / 30;
        } else {
            return getElevatorEncoder() > heightOfChoice;
        }
    }

    public double getEffort() {
        return totalEffort = ((elevatorFeedforward.calculate(0, 0))
                + (elevatorController.calculate(getElevatorEncoder(), height)));
    }

    @Override
    public void periodic() {
        if (Utils.isSimulation()) {
            simEncoder = elevatorViz.getLength();

            // motor stuff
            elevatorMotor.setPosition(simEncoder, 0.015);
            elevatorMotor.setControl(motionMagicVoltage.withPosition(height / 6).withSlot(0));

            // simulation & visualization stuff
            elevatorSim.setInputVoltage(elevatorMotor.getMotorVoltage().getValueAsDouble());
            elevatorSim.update(0.01);
            elevatorViz.setLength(elevatorSim.getPositionMeters());

            // dashboard stuff
            SmartDashboard.putNumber("Elevator Encoder", simEncoder);
            SmartDashboard.putNumber("Elevator set voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator follower set voltage",
                    elevatorFollowerMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Height", height / 6);
            SmartDashboard.putNumber("sim elev position", elevatorSim.getPositionMeters());
            SmartDashboard.putNumber("sim elev motor position", elevatorMotor.getPosition().getValueAsDouble());

        } else {
            // motor stuff
            elevatorMotor.setControl(motionMagicVoltage.withPosition(height / 6).withSlot(0));
            elevatorMotor.setPosition(simEncoder);

            // elevatorMotor.setVoltage(getEffort() * -1.0);//the old way (Using PID &
            // feedforward)

            // dashboard stuff
            SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());
            SmartDashboard.putNumber("Elevator Effort", totalEffort);
            SmartDashboard.putNumber("Elevator Height", height);
            SmartDashboard.putBoolean("issGigh", issGigh());
        }
    }

}