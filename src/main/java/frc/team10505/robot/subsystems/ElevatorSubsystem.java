/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import static frc.team10505.robot.subsystems.HardwareConstants.*;


//TODO -- when testing IRL, add back current limits??
public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    public final TalonFX elevatorMotor;
    public final TalonFX elevatorFollowerMotor;

    private MotionMagicVoltage motionMagicVoltage;

    private final double startingHeight = 0;
    private double height = startingHeight;

    /* Simulation variables */
    // Variables to create a visualization
    public final Mechanism2d elevSimMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d elevRoot = elevSimMech.getRoot("elevRoot", 0.75, 0.1);
    public final MechanismLigament2d elevViz = elevRoot
            .append(new MechanismLigament2d("elevatorLigament", startingHeight, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));

    // simulation of the PHYSICS of the mechanisms (this is what does the calculations/makes the sim useful & cool)
    private final ElevatorSim elevSim = new ElevatorSim(DCMotor.getKrakenX60(2), 12, 10, 0.05, 0.0, 3.0, true, startingHeight);

    /* Constructor */
    public ElevatorSubsystem() {
        if (Utils.isSimulation() || Utils.isReplay()) {
            SmartDashboard.putData("elevSimMech", elevSimMech);
            elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
            elevatorFollowerMotor = new TalonFX(ELEVATOR_FOLLOWER_MOTOR_ID);
            motionMagicVoltage = new MotionMagicVoltage(startingHeight);// use starting position(position is
                                                                             // changed periodically)
        } else {
            elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID, "kingKan");
            elevatorFollowerMotor = new TalonFX(ELEVATOR_FOLLOWER_MOTOR_ID, "kingKan");
            motionMagicVoltage = new MotionMagicVoltage(startingHeight);// use starting position(position is changed periodically)
        }

        /* Motor configs */
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ELEVATOR_GEARSTACK;

        MotionMagicConfigs motionMagic = cfg.MotionMagic;
        motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(20))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(40))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));
        Slot0Configs slot0 = cfg.Slot0;

        /* gains for pid and ffe */
        if (Utils.isSimulation() || Utils.isReplay()) {
            slot0.kS = 0.0; // Add 0.25 V output to overcome static friction
            slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
            slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
            slot0.kG = 0.5297854 / 2.947; /* intended to counter gravity, added by us(not in documentation) */
            slot0.kP = 26;// A position error of 0.2 rotations results in 12 V output
            slot0.kI = 0; // No output for integrated error
            slot0.kD = 0.5;// 1.9// A velocity error of 1 rps results in 0.5 V output
        } else {
            slot0.kS = 0.0; // Add 0.25 V output to overcome static friction
            slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
            slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
            slot0.kG = 0.5297854 / 2.947; /* intended to counter gravity, added by us(not in documentation) */
            slot0.kP = 26;// A position error of 0.2 rotations results in 12 V output
            slot0.kI = 0; // No output for integrated error
            slot0.kD = 0.5;// 1.9// A velocity error of 1 rps results in 0.5 V output
        }

        // CurrentLimitsConfigs currentLimits = cfg.CurrentLimits;
        // currentLimits.StatorCurrentLimit = ELEVATOR_MOTOR_CURRENT_LIMIT;

        MotorOutputConfigs motorOutput = cfg.MotorOutput;
        motorOutput.NeutralMode = NeutralModeValue.Brake;

        StatusCode leaderStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            leaderStatus = elevatorMotor.getConfigurator().apply(cfg);
            if (leaderStatus.isOK())
                break;
        }
        if (!leaderStatus.isOK()) {
            System.out.println("Could not configure Elevator Motor. Error: " + leaderStatus.toString());
        }

        StatusCode followerStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            followerStatus = elevatorFollowerMotor.getConfigurator().apply(cfg);
            if (followerStatus.isOK())
                break;
        }
        if (!followerStatus.isOK()) {
            System.out.println("Could not configure Elevator Leader Motor. Error: " + followerStatus.toString());
        }

        elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));
    }

    /* commands to referense */
    /** changes our setpoint, which changes our pid calcuations therefore effort to
    the motor, which happens periodically*/
    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    /**run end command that sets the elev motors' voltages to 0 once ending */
    public Command runMotor(double voltage) {
        return runEnd(() -> {
            elevatorMotor.setVoltage(voltage);
        },
                () -> {
                    elevatorMotor.setVoltage(0);
                });
    }

    /* Calculations */
    public double getElevatorEncoder() {
        if (Utils.isSimulation() || Utils.isReplay()) {
            return elevViz.getLength();
        } else {
            return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751 * 2) / 12.0) * -1.0;
        }
    }

    /**returns true if the encoder is within 2 units of our setpoint */
    public boolean isNearGoal() {
        return MathUtil.isNear(height, getElevatorEncoder(), 2);
    }

    /**returns true if the encoder is above 30 */
    public boolean issGigh() {
        return getElevatorEncoder() > 30;
    }

    /**returns true if the encoder is above the inputted parameter */
    public boolean isAbove(double heightOfChoice) {
        return getElevatorEncoder() > heightOfChoice;
    }

    @Override
    public void periodic() {
        //dashboard stuff
        SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());
        SmartDashboard.putNumber("Elevator Height", height);
        SmartDashboard.putNumber("Elevator set voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator follower set voltage", elevatorFollowerMotor.getMotorVoltage().getValueAsDouble());

        //motor controlling stuff
        var change = height - getElevatorEncoder();
        elevatorMotor.setControl(motionMagicVoltage.withPosition(change).withSlot(0));

        //sim updating stuff
        if (Utils.isSimulation() || Utils.isReplay()) {
            elevSim.setInputVoltage(elevatorMotor.getMotorVoltage().getValueAsDouble() * 2);
            elevSim.update(0.01);
            elevViz.setLength(elevSim.getPositionMeters());
        }
    }

}