package frc.team10505.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import static frc.team10505.robot.Constants.HardwareConstants.*;
import static frc.team10505.robot.Constants.*;

public class CoralSubsystem extends SubsystemBase {
    // Motor controllers
    private final SparkMax intakeLeft = new SparkMax(CORAL_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax intakeRight = new SparkMax(CORAL_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig intakeRightConfig = new SparkMaxConfig();

    // Laser sensors
    private final LaserCan inLaser = new LaserCan(CORAL_IN_LASER_ID);
    private final LaserCan outLaser = new LaserCan(CORAL_OUT_LASER_ID);

    public double motorSpeed = 0;// ONLY USED FOR LOGGING AND SIM
    public double secondaryMotorSpeed = 0;// ONLY USED FOR LOGGING AND SIM

    private CommandJoystick joystick;

    /** Constructor for IRL */
    public CoralSubsystem() {
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

    /** Other constructor, intended for use in simulation */
    public CoralSubsystem(CommandJoystick joystick) {
        this.joystick = joystick;

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
    /** returns true if the laser reads a distance less than 50mm, or in sim, if joystick button 1 is pressed */
    public boolean inSensor() {
        if (Utils.isSimulation()) {
            return joystick.button(1).getAsBoolean();
        } else {
            LaserCan.Measurement inMeas = inLaser.getMeasurement();
            return (inMeas.distance_mm < 50.0 && inMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
        }
    }

    /**returns true if the laser reads a distance less than 50mm, or in sim, if joystick button 2 is pressed*/
    public boolean outSensor() {
        if (Utils.isSimulation()) {
            return joystick.button(2).getAsBoolean();
        } else {
            LaserCan.Measurement outMeas = outLaser.getMeasurement();
            return (outMeas.distance_mm < 100.0 && outMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
        }
    }

    /* BETTER commands to referense */
    /** run end command that will stop the intake upon ending */
    public Command runIntake(double speed) {
        return runEnd(() -> {
            intakeLeft.set(speed);
            intakeRight.set(speed);
            motorSpeed = speed;
            secondaryMotorSpeed = speed;
        }, () -> {
            intakeLeft.set(0);
            intakeRight.set(0);
            motorSpeed = 0;
            secondaryMotorSpeed = 0;
        });
    }

    /** run once command that sets the intake speeds */
    public Command setIntake(double speed) {
        return runOnce(() -> {
            intakeLeft.set(speed);
            intakeRight.set(speed);
            motorSpeed = speed;
            secondaryMotorSpeed = speed;
        });
    }

    /**run end command that is intended for L1 coral shots. Runs the left and right intake at different speeds. Stops both motors upon ending */
    public Command trough() {
        return runEnd(() -> {
            intakeLeft.set(CORAL_TROUGH_LEFT_SPEED);
            intakeRight.set(CORAL_TROUGH_RIGHT_SPEED);
            motorSpeed = CORAL_TROUGH_LEFT_SPEED;
            secondaryMotorSpeed = CORAL_TROUGH_RIGHT_SPEED;

        },
                () -> {
                    intakeLeft.set(0);
                    intakeRight.set(0);
                    motorSpeed = 0;
                    secondaryMotorSpeed = 0;
                });
    }

    /** run end command that intakes. Upon ending, it will use the runIntake command to run slowly until only the outSensor boolean is true*/
    public Command slowEndIntake(double firstSpeed) {

        return runEnd(() -> {
            intakeLeft.set(firstSpeed);
            intakeRight.set(firstSpeed);
            motorSpeed = firstSpeed;
            secondaryMotorSpeed = firstSpeed;

        },
                () -> {
                    runIntake(CORAL_SLOW_SPEED).until(() -> (outSensor() && !inSensor()));
                });
    }

    @Override
    public void periodic() {
        // dashboard stuff
        SmartDashboard.putNumber("Motor speed", motorSpeed);
        SmartDashboard.putNumber("Secondary motor speed", secondaryMotorSpeed);
        SmartDashboard.putBoolean("inSensor", inSensor());
        SmartDashboard.putBoolean("outSensor", outSensor());
        if (!Utils.isSimulation()) {
            SmartDashboard.putNumber("left intake motor applied output", intakeLeft.getAppliedOutput());
            SmartDashboard.putNumber("right intake motor applied output", intakeRight.getAppliedOutput());
        }
    }
}
