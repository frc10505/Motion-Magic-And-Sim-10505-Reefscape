/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static frc.team10505.robot.subsystems.HardwareConstants.*;


public class AlgaeSubsystem extends SubsystemBase {
    /* Variables */
    // motor controllers
    private final SparkMax intakeMotor = new SparkMax(ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(ALGAE_PIVOT_MOTOR_ID, MotorType.kBrushless);
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    // Encoder
    private double absoluteOffset = 180.0;
    private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

    //controllers
    private final PIDController pivotController;
    private final ArmFeedforward pivotFeedforward;

    private double pivotSetpoint = -90; // 90
    public boolean coasting = false;
    private double intakeSpeed = 0;//ONLY USED FOR LOGGING

    /* Sim Variables*/ 
    private final double simStartingAngle = 0;

    //Variables to create a visualization
    private final Mechanism2d algaeMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d pivotRoot = algaeMech.getRoot("pivotRoot", 0.75, 0.75);
    private final MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("pivotViz", 0.56, simStartingAngle));
   
    private final MechanismRoot2d intakeRoot = algaeMech.getRoot("intakeRoot", 1.31, 0.75);
    private final MechanismLigament2d intakeViz = intakeRoot.append(new MechanismLigament2d("intakeViz", 0.07, simStartingAngle, 10, new Color8Bit(Color.kTomato)));

    //simulation of the PHYSICS of the mechanisms (this is what does the calculations/makes the sim useful & cool)
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 80,
            SingleJointedArmSim.estimateMOI(0.305, 2), 0.305, Units.degreesToRadians(-120), Units.degreesToRadians(120),
            true, Units.degreesToRadians(simStartingAngle));

    private final FlywheelSim intakeSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 5), DCMotor.getNEO(1));

  

    /* Our constructor */
    public AlgaeSubsystem() {
        if(Utils.isSimulation()){
            pivotController = new PIDController(1.6, 0, 0.01);
            pivotFeedforward = new ArmFeedforward(0, 0.1719, 0.4, 0.1);
        }else{
            pivotController = new PIDController(0.1, 0, 0);
            pivotFeedforward = new ArmFeedforward(0.01, 0.1, 0.4, 0.1);
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

        // Sim visualization
        SmartDashboard.putData("Algae Subsys Viz", algaeMech);
    }

    /* Methods */

    //calculations
    public double getPivotEncoder() {
        if(Utils.isSimulation()){
            return pivotViz.getAngle();
        }else{
            return (-pivotEncoder.getPosition() + absoluteOffset);
        }
    }

    public double getEffort() {
        return pivotFeedforward.calculate(Units.degreesToRadians(getPivotEncoder()), 0)
                + pivotController.calculate(getPivotEncoder(), pivotSetpoint);
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

    public Command stopPivot() {
        return runOnce(() -> {
            pivotMotor.stopMotor();
        });
    }

    public Command coastPivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(IdleMode.kCoast);
            coasting = true;// stops pivot motor from being set to calculated pid effort
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
    public Command setIntake(double speed){
        return runOnce(() ->{
            intakeMotor.set(speed);
            intakeSpeed = speed;
        });
    }

    // public Command intakeReverse() {
    //     return runOnce(() -> {
    //         intakeMotor.set(-intakeSpeed);
    //         intakeSpeed = -intakeSpeed;
    //     });
    // }

    public Command stopIntake() {
        return runOnce(() -> {
            intakeMotor.set(0);
            intakeSpeed = 0;
        });
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Pivot Encoder", getPivotEncoder());
        SmartDashboard.putNumber("Pivot Calculated Effort", getEffort());
        SmartDashboard.putNumber("Algae Intake Speed", intakeSpeed);

        if (Utils.isSimulation()) {
            pivotSim.setInput(getEffort());
            pivotSim.update(0.01);
            pivotViz.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));
    
            intakeSim.setInput(intakeSpeed);
            intakeSim.update(0.01);
            intakeRoot.setPosition((Math.cos(Units.degreesToRadians(pivotViz.getAngle())) * 0.56) + 0.75, (Math.sin(Units.degreesToRadians(pivotViz.getAngle())) * 0.56) + 0.75);
            intakeViz.setAngle(Units.radiansToDegrees(intakeViz.getAngle() + (intakeSpeed * 0.05)));

            SmartDashboard.putNumber("Sim Algae Intake Viz Angle", intakeViz.getAngle());

        } else {
            if (!coasting) {
                pivotMotor.setVoltage(getEffort());
            }
            SmartDashboard.putNumber("Intake Motor Output", intakeMotor.getAppliedOutput());
            SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
        }
    }
}