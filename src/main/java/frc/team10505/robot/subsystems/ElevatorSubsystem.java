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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import static frc.team10505.robot.Constants.HardwareConstants.*;

//TODO -- when testing IRL, add back current limits??
public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    public final TalonFX elevatorMotor;
    public final TalonFX elevatorFollowerMotor;

    private MotionMagicVoltage motionMagicVoltage;

    public final double startingHeight = 0;
    private double height = startingHeight;
    private double change = 0;

    // simulation of the PHYSICS of the mechanisms (this is what does the
    // calculations/makes the sim useful & cool)
    private final ElevatorSim elevSim = new ElevatorSim(DCMotor.getKrakenX60(2), 12, 10, 0.05, 0.0, 3.0, true,
            startingHeight);
    public double simElevEncoder = startingHeight;

    /* Constructor */
    public ElevatorSubsystem() {
        if (Utils.isSimulation() || Utils.isReplay()) {
            elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
            elevatorFollowerMotor = new TalonFX(ELEVATOR_FOLLOWER_MOTOR_ID);
            motionMagicVoltage = new MotionMagicVoltage(startingHeight);// use starting position(position is
                                                                        // changed periodically)
        } else {
            elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID, "kingKan");
            elevatorFollowerMotor = new TalonFX(ELEVATOR_FOLLOWER_MOTOR_ID, "kingKan");
            motionMagicVoltage = new MotionMagicVoltage(startingHeight);// use starting position(position is changed
                                                                        // periodically)
        }

        /* Motor configs */
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // motor output configs. sets the neutral mode to brake (could set it to coast)
        MotorOutputConfigs motorOutput = cfg.MotorOutput;
        motorOutput.NeutralMode = NeutralModeValue.Brake;

        // TODO maybe add back IRL
        // current limit configs. prevents the motors from using more current that theb
        // limit(saves our battery!)
        // CurrentLimitsConfigs currentLimits = cfg.CurrentLimits;
        // currentLimits.StatorCurrentLimit = ELEVATOR_MOTOR_CURRENT_LIMIT;

        // feedback configs. informs the system of the gearstack (could do more stuff
        // relevant to sensor and mechanism coorelation)
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ELEVATOR_GEARSTACK;

        // Motion magic configs. sets up the motion magic for us to referense
        MotionMagicConfigs motionMagic = cfg.MotionMagic;
        motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(20))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(40));

        // Slot 0 configs. Creates one set of configs that can be applied to motion
        // magic
        Slot0Configs slot0 = cfg.Slot0;

        /* gains for pid and ffe */
        if (Utils.isSimulation() || Utils.isReplay()) {

            slot0.kS = 0.0; // counters static friction(there is none in a simulation)
            slot0.kV = 0.12; // for velocity /*KARTER - this number should ALWAYS BE LOW */
            slot0.kA = 0.01; // for acceleration /*KARTER - this number should ALWAYS BE LOW */
            slot0.kG = 0.5297854 / 2.947; /* counters gravity, added by us(not in documentation) */
            slot0.kP = 26;
            slot0.kI = 0; // No output for integrated error /*COOPER- this number should ALWAYS BE ZERO */
            slot0.kD = 0.5; /* COOPER - this number should ALWAYS BE SUPER LOW OR ZERO */
        } else {
            slot0.kS = 0.05; // counters static friction
            slot0.kV = 0.12; // for velocity /*KARTER - this number should ALWAYS BE LOW */
            slot0.kA = 0.01; // for acceleration /*KARTER - this number should ALWAYS BE LOW */
            slot0.kG = 0.0; /* counters gravity, added by us(not in documentation) */
            slot0.kP = 26;
            slot0.kI = 0; // No output for integrated error/*COOPER- this number should ALWAYS BE ZERO */
            slot0.kD = 0.0; /* COOPER - this number should ALWAYS BE SUPER LOW OR ZERO */
        }

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
    /**
     * changes our setpoint, which changes our pid calcuations therefore effort to
     * the motor, which happens periodically
     */
    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    /** run end command that sets the elev motors' voltages to 0 upon ending */
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
            return simElevEncoder;//elevSim.getPositionMeters();
        } else {
            return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751 * 2) / 12.0) * -1.0;
        }
    }

    /** returns true if the encoder is within 2 units of our setpoint */
    public boolean isNearGoal() {
        return MathUtil.isNear(height, getElevatorEncoder(), 2);
    }

    /** returns true if the encoder is above 30 */
    public boolean issGigh() {
        return getElevatorEncoder() > 30;
    }

    /** returns true if the encoder is above the inputted parameter */
    public boolean isAbove(double heightOfChoice) {
        return getElevatorEncoder() > heightOfChoice;
    }

    @Override
    public void simulationPeriodic() {
       elevSim.setInput(elevatorMotor.getMotorVoltage().getValueAsDouble() * 2);
       elevSim.update(0.001);
       simElevEncoder = elevSim.getPositionMeters();
    }

    @Override
    public void periodic() {
        // dashboard stuff
        SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());
        SmartDashboard.putNumber("Elevator Height", height);
        SmartDashboard.putNumber("change", change);

        if (!Utils.isSimulation()) {
            SmartDashboard.putNumber("Elevator set voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator follower set voltage", elevatorFollowerMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Lead Motor Position", elevatorMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Follower Motor Position", elevatorFollowerMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Lead Motor Rotor Position", elevatorMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Follower Motor Rotor Position", elevatorFollowerMotor.getPosition().getValueAsDouble());
        }

        // motor controlling stuff
        change = height - getElevatorEncoder();
        elevatorMotor.setControl(motionMagicVoltage.withPosition(change).withSlot(0));
    }

}