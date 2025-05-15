/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static frc.team10505.robot.Constants.ALGAE_PIVOT_DOWN;
import static frc.team10505.robot.Constants.HardwareConstants.*;

public class AlgaeSubsystem extends SubsystemBase {
    /* Variables */
    // motor controllers
    private final SparkMax intakeMotor = new SparkMax(ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(ALGAE_PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    // Encoder
    private double absoluteOffset = 180.0;
    private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

    // controllers
    private final PIDController pivotController;
    private final ArmFeedforward pivotFeedforward;

    public final double startingAngle = ALGAE_PIVOT_DOWN;
    private double pivotSetpoint = startingAngle;
    public boolean coasting = false;
    public double intakeSpeed = 0;// ONLY USED FOR LOGGING AND SIM

    // simulation of the PHYSICS of the mechanisms (this is what does the
    // calculations/makes the sim useful & cool)
    public final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 80,
            SingleJointedArmSim.estimateMOI(0.305, 2), 0.305, Units.degreesToRadians(-120), Units.degreesToRadians(120),
            true, Units.degreesToRadians(startingAngle));

    public double simPivotEncoder = startingAngle;
    
    /* Our constructor */
    public AlgaeSubsystem() {
        if (Utils.isSimulation() || Utils.isReplay()) {
            pivotController = new PIDController(1.6, 0, 0.0);
            pivotFeedforward = new ArmFeedforward(0, 0.1719, 0);//040, 040);
        } else {
            pivotController = new PIDController(0, 0, 0);// TODO TUNE IRL
            pivotFeedforward = new ArmFeedforward(0.01, 0.1, 0.4, 0.1);// TODO TUNE IRL
        }

        // pivot motor and encoder configs
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(ALGAE_PIVOT_MOTOR_CURRENT_LIMIT, ALGAE_PIVOT_MOTOR_CURRENT_LIMIT);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(ALGAE_PIVOT_ENCODER_SCALE);
        pivotMotorConfig.absoluteEncoder.zeroOffset(ALGAE_PIVOT_ENCODER_OFFSET);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Intake motor config
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.smartCurrentLimit(ALGAE_INTAKE_MOTOR_CURRENT_LIMIT, ALGAE_INTAKE_MOTOR_CURRENT_LIMIT);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* Methods */
    // calculations
    /**returns the pivot's position in degrees */
    public double getPivotEncoder() {
        if (Utils.isSimulation() || Utils.isReplay()) {
            return simPivotEncoder;
        } else {
            return (-pivotEncoder.getPosition() + absoluteOffset);
        }
    }

    public double getEffort() {
        if(Utils.isSimulation()){
            return pivotFeedforward.calculateWithVelocities(Units.degreesToRadians(getPivotEncoder()), 0, 0)//TODO Mess with/figure out
            + pivotController.calculate(getPivotEncoder(), pivotSetpoint);
        }else {
            return pivotFeedforward.calculate(Units.degreesToRadians(getPivotEncoder()), 0)
                + pivotController.calculate(getPivotEncoder(), pivotSetpoint);
        }
    }

    /* pivot commands to reference */
    /** Standard command. Run once command that changes the setpoint of the pivot */
    public Command setAngle(double angle) {
        return runOnce(() -> {
            pivotSetpoint = angle;
        });
    }

    /**
     * Run command that continually sets the pivot motor voltage to the inputted
     * parameter
     */
    public Command setVoltage(double voltage) {
        return run(() -> {
            pivotMotor.setVoltage(voltage);
        });
    }

    /** run once command that stops the pivot motor and "disables" PID */
    public Command stopPivot() {
        return runOnce(() -> {
            coasting = true;// stops pivot motor from being set to calculated pid effort
            pivotMotor.stopMotor();
        });
    }

    /** Changes the pivot idle mode to coast and "disables" PID */
    public Command coastPivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(IdleMode.kCoast);
            coasting = true;// stops pivot motor from being set to calculated pid effort
        });
    }

    /**
     * Undoes the coastPivot command. Sets the pivot idle mode to brake and
     * "enables" PID
     */
    public Command brakePivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(IdleMode.kBrake);
            pivotSetpoint = getPivotEncoder();
            coasting = false;
        });
    }

    /* intake commands to reference */
    /**
     * runOnce command that sets the intake motor to a speed of the inputted
     * parameter
     */
    public Command setIntake(double speed) {
        return runOnce(() -> {
            intakeMotor.set(speed);
            intakeSpeed = speed;
        });
    }

    /** runOnce command that sets the intake motor to a speed of ZERO */
    public Command stopIntake() {
        return runOnce(() -> {
            intakeMotor.set(0);
            intakeSpeed = 0;
        });
    }

    @Override
    public void simulationPeriodic(){
        if (!coasting) {
             pivotSim.setInput(getEffort());
        }
        pivotSim.update(0.001);
        simPivotEncoder = Units.radiansToDegrees(pivotSim.getAngleRads());
    }

    @Override
    public void periodic() {
        // dashboard stuff
        SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Pivot Encoder", getPivotEncoder());
        SmartDashboard.putNumber("Pivot Calculated Effort", getEffort());
        SmartDashboard.putNumber("Algae Intake Speed", intakeSpeed);

        if(!Utils.isSimulation()){
            if (!coasting) {
                pivotMotor.setVoltage(getEffort());
            }
            SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
            SmartDashboard.putNumber("Algae Intake Motor Output", intakeMotor.getAppliedOutput());    
        }
        
    }
}