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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team10505.robot.Constants.AlgaeConstants.*;

import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeSubsystem extends SubsystemBase {

    // motor controllers
    public final static SparkMax intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(kAlgaePivotMotorId, MotorType.kBrushless);
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    // Encoder
    private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
    private double encoderValue;
    private double absoluteOffset = 180.0;
    private double simEncoder = 60.0;

    private final Mechanism2d pivotMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d pivotRoot = pivotMech.getRoot("pivotRoot", 0.75, 0.75);
    private final MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("pivotViz", 0.56, 0));
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 80,
            SingleJointedArmSim.estimateMOI(0.305, 2), 0.305, Units.degreesToRadians(-120), Units.degreesToRadians(120),
            true, Units.degreesToRadians(60));

    private final PIDController pivotController = new PIDController(KP, KI, KD);
    private final PIDController simPivotController = new PIDController(1.6, 0, 0.01);
    private final ArmFeedforward simPivotFeedforward = new ArmFeedforward(0, 0.1719, 0.4, 0.1);// kg 1.2 too low
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(0.01, 0.1, 0.4, 0.1);

    private double pivotSetpoint = -90; // 90

    public boolean coasting = false;

    /* Our constructor */
    public AlgaeSubsystem() {
        configAlgaeSubsys();
        SmartDashboard.putNumber("pivotEncoder", encoderValue);
        SmartDashboard.putData("pivotViz", pivotMech);

    }

    /* Pivot calculation methods */
    // Get encoder
    public double getPivotEncoder() {
        return (-pivotEncoder.getPosition() + absoluteOffset);
    }

    // Calc PID
    public double getEffort() {
        return pivotFeedforward.calculate(Units.degreesToRadians(getPivotEncoder()), 0)
                + pivotController.calculate(getPivotEncoder(), pivotSetpoint);
    }

    public double simGetEffort() {
        return simPivotFeedforward.calculate(Units.degreesToRadians(pivotViz.getAngle()), 0)
                + simPivotController.calculate(simEncoder, pivotSetpoint);
    }

    /* pivot commands to reference */
    public Command setAngle(double angle) {
        return runOnce(() -> {
            pivotSetpoint = angle;
        });
    }

    public Command setVoltage(double voltage) {
        return run(() -> {
            pivotMotor.setVoltage(voltage);
        });
    }

    // command we set to as the default command
    /*
     * NOTE that defaul command has been problematic in past years -
     * if it becomes an issue, set the pivot motor to pid effort periodically
     */
    // public Command holdAngle() {
    // return run(() -> {
    // if (!coasting) {
    // //pivotMotor.setVoltage(PIDEffort());
    // }
    // });
    // }

    // Only use when testing motor direction, not useful with pid(the motor will
    // only stop for 0.02 s)
    public Command stopPivot() {
        return runOnce(() -> {
            pivotMotor.stopMotor();
        });
    }

    public Command coastPivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(IdleMode.kCoast);// stops pivot motor from being set to calculated pid effort
            coasting = true;
        });
    }

    public Command brakePivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(IdleMode.kBrake);
            pivotSetpoint = getPivotEncoder();
            coasting = false;
        });
    }

    /* intake commands to reference */
    public Command intakeForward() {
        return runOnce(() -> {
            intakeMotor.set(intakeSpeed);
        });
    }

    public Command intakeForwardSlower() {
        return runOnce(() -> {
            intakeMotor.set(intakeSlowSpead);
        });
    }

    public Command intakeForwardSlowest() {
        return runOnce(() -> {
            intakeMotor.set(sigmaSpead);
        });
    }

    public Command intakeReverse() {
        return runOnce(() -> {
            intakeMotor.set(-intakeSpeed);
        });
    }

    public Command intakeStop() {
        return runOnce(() -> {
            intakeMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        // encoderValue = getPivotEncoder();
        // SmartDashboard.putNumber(" Pivot Encoder", encoderValue);
        // SmartDashboard.putNumber("Intake Motor Output",
        // intakeMotor.getAppliedOutput());
        // SmartDashboard.putNumber("Pivot Motor Output",
        // pivotMotor.getAppliedOutput());
        if (Utils.isSimulation()) {
            simEncoder = pivotViz.getAngle();

            // pivotMotor.setVoltage(simGetEffort());
            pivotSim.setInput(simGetEffort());
            pivotSim.update(0.01);
            pivotViz.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));
            SmartDashboard.putNumber(("simPivotAngle"), pivotViz.getAngle());

        } else {
            if (!coasting) {
                pivotMotor.setVoltage(getEffort());
            }

        }

    }

    /*
     * configurations to be called in the constructor,
     * runs once during init,
     * here it is used to configure motor settings
     */
    private void configAlgaeSubsys() {
        // Pivot motor config
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(kPivotMotorCurrentLimit,
                kPivotMotorCurrentLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale); // Angle encoder scale
        pivotMotorConfig.absoluteEncoder.zeroOffset(pivotEncoderOffset); // Angle encoder offset
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Intake motor config
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.smartCurrentLimit(kIntakeMotorCurrentLimit,
                kIntakeMotorCurrentLimit);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}